#!/usr/bin/env python3

from enum import Enum

import numpy as np
import ros_numpy
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

from lunabot_msgs.msg import RobotEffort, RobotState


def in_range(val, limits):
    return val > limits[0] and val < limits[1]


class DepositState(Enum):
    HOMING = 0  # align with sieve using apriltags
    DRIVE = 1  # once aligned, drive backwards to sieve until current limit is reached
    OPEN_EXC = 2  # open excavation tool to make space for depositing
    DEPOSIT = 3  # extend deposition system and deposit
    RETRACT = 4  # after depositing, retract deposition system to closed configuration
    CLOSE_EXC = 5  # close excavation
    RESET = 6  # drive forward until apriltags are visible
    DONE = 7  # done terminal state


class HomingController:
    # clips input is within limits bounds
    linear_threshold = 0.05
    angular_threshold = 3 * np.pi / 180

    linear_setpoint = 0.8
    angular_setpoint = 0

    KP = np.array([4, 4])
    KI = np.array([0.0, 0.0])
    KD = np.array([0, 0.0])

    ALPHA = 0.5  # diff drive
    SCALE = 0.5
    MIN_VEL = 0.2

    DRIVE_CTRL = 0.5
    DRIVE_CURR_RANGE = (5000, 18000)
    DEP_CURR_RANGE = (12900, 13400)  # TODO: find

    TIME_FROM_RETRACT_TO_EXTEND = 25.0
    TIME_TO_CLEAR_BUCKETS = 7
    DEP_CURR_STARTING = 2

    def __init__(self):
        self._apriltag_sub = rospy.Subscriber(
            "/d455_front/camera/color/tag_detections",
            AprilTagDetectionArray,
            self.apritag_cb,
        )

        self._drive_left_curr = None
        self._drive_right_curr = None
        self._dep_curr = None

        self._state_sub = rospy.Subscriber("/state", RobotState, self._robot_state_cb)
        self._effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)

        self._effort_msg = RobotEffort()
        self._state = DepositState.OPEN_EXC

        self._curr_time_act = None

        self.T_camera_april_msg = None
        # self._effort_msg = Twist()
        # self._effort_msg = Float32()
        self._prev_error = np.zeros(2)
        self._curr_error = np.zeros(2)
        self._error_total = np.zeros(2)

        r = rospy.Rate(20)

        rospy.on_shutdown(self.shutdown_hook)

        while not rospy.is_shutdown():
            if (
                self._drive_left_curr is not None
                and self._drive_right_curr is not None
                and self._dep_curr is not None
            ):
                if self._state == DepositState.DONE:
                    break
                else:
                    self.loop()
            r.sleep()

    def shutdown_hook(self):
        self.stop()
        print("stopping homing control")

    def apritag_cb(self, msg):
        if len(msg.detections) != 0:
            # self._effort_msg.linear.x = 0
            # self._effort_msg.angular.z = 0

            self.T_camera_april_msg = msg.detections[
                0
            ].pose.pose.pose  # transformation from apriltag to camera frame (check header frame_id to be sure)
        else:
            self.T_camera_april_msg = None

    def _robot_state_cb(self, msg):
        self._drive_left_curr = msg.drive_left_curr
        self._drive_right_curr = msg.drive_right_curr
        self._dep_curr = msg.dep_curr

    def homing(self):
        if self.T_camera_april_msg is None:
            return
        T_camera_april = ros_numpy.numpify(
            self.T_camera_april_msg
        )  # Converting the transformation matrix to a numpy array

        print(T_camera_april)

        # Assigning the unit vector values for the different vectors
        v_1A = np.array([0, 0, -1, 1])
        # V_1A refers to the april tag reference in the april tag frame
        v_2C = np.array([1, 0, 0, 1])
        # V_2C refers to the unit vector for the camera frame of the robot

        # compare = np.array([0,0,0,0])

        # Computing the location of point V that is in frame A to transformed to its location in frame C
        v_1C = T_camera_april @ v_1A
        p_A_zero = np.array([0, 0, 0, 1])
        v_C_april = T_camera_april @ p_A_zero
        # if (T_camera_april == compare):
        #     self._effort_msg.left_drive = 0
        #     self._effort_msg.right_drive = 0

        v_1C[2] = 0
        v_C_april[2] = 0
        v_C_april[3] = 0

        # Finding the error angle between the two points, v_1C and v_2C, using dot product (in radians)

        # v_1C = v_1C[0:2]
        # v_2C = v_2C[0:2]

        angle = np.arctan2(
            v_1C[0] * v_2C[1] - v_2C[0] * v_1C[1], v_1C[0] * v_1C[1] + v_2C[0] * v_2C[1]
        )

        angle += 3 * np.pi / 2 + 0.3
        if angle > np.pi:
            angle -= 2 * np.pi

        # determining the linear translational error (in x and y) of the robot considered

        self.v_C_april = np.array(
            [
                self.T_camera_april_msg.position.x,
                self.T_camera_april_msg.position.z,
                0,
                0,
            ]
        )

        print(v_C_april)
        distance = np.linalg.norm(v_C_april)
        # Replaced v_2c with 0,0,0,1 for the sim because the distance being calculated
        # Was seemingly off from the apriltag (based on mapping out values, was 0,0,0)

        # write PD controller here with the output being lin, ang velocity
        # Define the K constants for P, I, and D
        # For sim tuning, lin and ang are flipped.
        # For Testing angular tuning only change the first value

        # Define the errors
        # lin_error_dist = np.sqrt((lin_error[0])**2 + (lin_error[1])**2)
        self._curr_error = np.array(
            [distance - self.linear_setpoint, angle - self.angular_setpoint]
        )

        print("self._curr_error: ", self._curr_error)
        self._error_total += self._curr_error

        # Computing PID errors
        ctrl = self._curr_error * self.KP
        ctrl += self._error_total * self.KI
        ctrl += (self._curr_error - self._prev_error) * self.KD

        # Set current error to previous error
        self._prev_error = self._curr_error

        # Converting errors into the lin_vel, ang_vel

        twist = np.zeros(2)  # lin, ang velocity

        # Setting thresholds to stop movement

        if np.abs(self._curr_error[0]) < self.linear_threshold:
            ctrl[0] = 0
        if np.abs(self._curr_error[1]) < self.angular_threshold:
            ctrl[1] = 0

        twist[0] = -ctrl[0]
        twist[1] = ctrl[1]

        # print("twist", twist)

        ctrl = self.diff_drive_model(
            twist
        )  # computes wheel velocities from lin, ang vel (twist)

        self._effort_msg.left_drive = self.constrain(ctrl[0])
        self._effort_msg.right_drive = self.constrain(ctrl[1])

        # print("left: ", self.constrain(ctrl[0]))
        # print("right: ", self.constrain(ctrl[1]))

    def loop(self):
        """

        DepositStates:

        """
        self._effort_msg == RobotEffort()
        if self._state == DepositState.HOMING:
            self.homing()
            if (
                np.abs(self._curr_error[0]) < self.linear_threshold
                and np.abs(self._curr_error[1]) < self.angular_threshold
            ):
                self._state = DepositState.DRIVE
        elif self._state == DepositState.DRIVE:
            if in_range(self._drive_right_curr, self.DRIVE_CURR_RANGE) and in_range(
                self._drive_left_curr, self.DRIVE_CURR_RANGE
            ):
                self._effort_msg.left_drive = self.constrain(-self.DRIVE_CTRL)
                self._effort_msg.right_drive = self.constrain(-self.DRIVE_CTRL)
            else:
                self._effort_msg.left_drive = 0
                self._effort_msg.right_drive = 0
                self._state = DepositState.OPEN_EXC
        elif self._state == DepositState.OPEN_EXC:
            diff = 0

            if self._curr_time_act is None:
                self._curr_time_act = rospy.Time.now().to_sec()
            else:
                diff = rospy.Time.now().to_sec() - self._curr_time_act

            if diff > self.TIME_TO_CLEAR_BUCKETS:
                self._effort_msg.excavate = self.constrain(-1, 1)

            if diff > self.TIME_FROM_RETRACT_TO_EXTEND:
                self._effort_msg.lin_act = 0
                self._effort_msg.excavate = 0
                self._curr_time_act = None
                self._state = DepositState.DEPOSIT
            else:
                self._effort_msg.lin_act = self.constrain(0.5, 1)

        elif self._state == DepositState.DEPOSIT:
            diff = 0
            if self._curr_time_act is None:
                self._curr_time_act = rospy.Time.now().to_sec()
            else:
                diff = rospy.Time.now().to_sec() - self._curr_time_act
            if (
                in_range(self._dep_curr, self.DEP_CURR_RANGE)
                or diff < self.DEP_CURR_STARTING
            ):
                self._effort_msg.deposit = self.constrain(1, 1)
            else:
                self._curr_time_act = None
                self._effort_msg.deposit = 0
                self._state = DepositState.RETRACT
        elif self._state == DepositState.RETRACT:
            diff = 0
            if self._curr_time_act is None:
                self._curr_time_act = rospy.Time.now().to_sec()
            else:
                diff = rospy.Time.now().to_sec() - self._curr_time_act
            if (
                in_range(self._dep_curr, self.DEP_CURR_RANGE)
                or diff < self.DEP_CURR_STARTING
            ):
                self._effort_msg.deposit = self.constrain(-1, 1)
            else:
                self._effort_msg.deposit = 0
                self._curr_time_act = None
                self._state = DepositState.CLOSE_EXC
        elif self._state == DepositState.CLOSE_EXC:
            diff = 0

            if self._curr_time_act is None:
                self._curr_time_act = rospy.Time.now().to_sec()
            else:
                diff = rospy.Time.now().to_sec() - self._curr_time_act
            if diff > self.TIME_FROM_RETRACT_TO_EXTEND:
                self._effort_msg.lin_act = 0
                self._effort_msg.excavate = 0
                self._curr_time_act = None
                self._state = DepositState.DONE
            else:
                self._effort_msg.lin_act = self.constrain(-0.5, 1)
        elif self._state == DepositState.RESET:
            if self.T_camera_april_msg is None:
                self._effort_msg.left_drive = self.constrain(self.DRIVE_CTRL)
                self._effort_msg.right_drive = self.constrain(self.DRIVE_CTRL)
            else:
                self._effort_msg.left_drive = 0
                self._effort_msg.right_drive = 0
                self._state = DepositState.DONE
        self._effort_pub.publish(self._effort_msg)  # Sends ctrl to robot

    def stop(self):
        self._effort_msg.left_drive = 0
        self._effort_msg.right_drive = 0
        self._effort_pub.publish(self._effort_msg)  # Sends ctrl to robot

    def constrain(self, unconstr_input, scale=None):
        if scale is None:
            scale = self.SCALE
        unconstr_input = np.clip(unconstr_input, -scale, scale)

        if unconstr_input != 0 and np.abs(unconstr_input) < self.MIN_VEL:
            unconstr_input = np.sign(unconstr_input) * self.MIN_VEL
        print(unconstr_input)

        return np.int8(min(unconstr_input * 127, 127))

    def diff_drive_model(self, twist):
        wheel_vel = np.zeros(2)
        wheel_vel[0] = twist[0] * self.ALPHA + twist[1] * (1 - self.ALPHA)
        wheel_vel[1] = twist[0] * self.ALPHA - twist[1] * (1 - self.ALPHA)
        return wheel_vel


if __name__ == "__main__":
    rospy.init_node("homing_controller_node")

    ctrl = HomingController()
