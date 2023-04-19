#!/usr/bin/env python3

"""
Design Homing Controller Using ROS:

Step 1: Get angular error from Apriltag (most of the work!)
1. Understand how coordinate frames work:
    https://w3.cs.jmu.edu/molloykp/teaching/cs354/cs354_2020Fall/resources/frames.pdf
    https://manipulation.mit.edu/pick.html#monogram
2. Understand how a transformation data format is represented in ROS for an apriltag
    http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
    https://github.com/AprilRobotics/apriltag_ros/blob/master/apriltag_ros/msg/AprilTagDetection.msg
    https://github.com/AprilRobotics/apriltag_ros/blob/master/apriltag_ros/msg/AprilTagDetectionArray.msg
3. Extract *relative* z angle between apriltag and camera frame such that at zero radians, the apriltag and camera face are parallel

Step 2: Write PD Controllor using angular error in apriltag_cb fn

Step 3: Run it!

"""


import numpy as np
import ros_numpy
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

# John added
from lunabot_msgs.msg import RobotEffort

# from geometry_msgs.msg import Twist


# clips input is within limits bounds
SCALE = 0.5
setpoint = 0.8
KP = np.array([4, 4])
KI = np.array([0.0, 0.0])
KD = np.array([0, 0.0])


def constrain(unconstr_input):
    global SCALE

    unconstr_input = np.clip(unconstr_input, -SCALE, SCALE)
    print(unconstr_input)

    return np.int8(min(unconstr_input * 127, 127))


def diff_drive_model(twist):
    wheel_vel = np.zeros(2)
    alpha = 0.5
    wheel_vel[0] = twist[0] * alpha + twist[1] * (1 - alpha)
    wheel_vel[1] = twist[0] * alpha - twist[1] * (1 - alpha)
    return wheel_vel


class HomingController:
    ctrl_scaler = 0.5

    def __init__(self):
        self._apriltag_sub = rospy.Subscriber(
            "/d455_front/camera/color/tag_detections",
            # "/d435_backward/color/tag_detections",
            AprilTagDetectionArray,
            self.apritag_cb,
        )

        self._effort_pub = rospy.Publisher("effort", RobotEffort, queue_size=1)
        # self._effort_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # self._effort_pub_ = rospy.Publisher('linear', Float32, queue_size=1);
        # self._effort_pub_ = rospy.Publisher('Angle', Float32, queue_size=1)

        self._effort_msg = RobotEffort()
        # self._effort_msg = Twist()
        # self._effort_msg = Float32()
        self._prev_error = np.zeros(2)
        self._error_total = np.zeros(2)

    def apritag_cb(self, msg):
        if len(msg.detections) == 0:
            # self._effort_msg.linear.x = 0
            # self._effort_msg.angular.z = 0

            self._effort_msg.left_drive = 0
            self._effort_msg.right_drive = 0

            # print("No AprilTag Dected\n")
            return

        T_camera_april_msg = msg.detections[
            0
        ].pose.pose.pose  # transformation from apriltag to camera frame (check header frame_id to be sure)

        T_camera_april = ros_numpy.numpify(
            T_camera_april_msg
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

        ang_error = np.arctan2(
            v_1C[0] * v_2C[1] - v_2C[0] * v_1C[1], v_1C[0] * v_1C[1] + v_2C[0] * v_2C[1]
        )

        ang_error += 3 * np.pi / 2 + 0.3
        if ang_error > np.pi:
            ang_error -= 2 * np.pi
        # print(ang_error * 180 / np.pi)

        # if(trig_result > 0 and sin > 0):
        #     ang_error = arc_cos
        # elif(trig_result > 0 and sin < 0):
        #     ang_error = -arc_cos
        # elif(sin > 0 and trig_result < 0):
        #     ang_error = arc_cos
        # else:
        #     ang_error = -arc_cos

        # if (arc_cos <= (np.pi) / 2 and 0 <= arc_sin <= (np.pi) / 2) or (
        #     (np.pi) / 2 < arc_cos <= np.pi and 0 < arc_sin <= (np.pi) / 2
        # ):
        #     ang_error = arc_cos
        # elif (arc_cos <= (np.pi) / 2 and 0 > arc_sin >= -(np.pi) / 2) or (
        #     (np.pi) / 2 < arc_cos <= np.pi and 0 > arc_sin >= -(np.pi) / 2
        # ):
        #     ang_error = -arc_cos
        # print(f"\nError Cosine: {trig_result}")
        # print(f"\nCalculated Error: {ang_error * 180 / np.pi}")

        # determining the linear translational error (in x and y) of the robot considered

        v_C_april = np.array(
            [T_camera_april_msg.position.x, T_camera_april_msg.position.z, 0, 0]
        )

        print(v_C_april)
        distance = np.linalg.norm(v_C_april)
        # Replaced v_2c with 0,0,0,1 for the sim because the distance being calculated
        # Was seemingly off from the apriltag (based on mapping out values, was 0,0,0)

        # lin_error = np.array([distance * math.cos(ang_error), distance * math.sin(ang_error)])

        # write PD controller here with the output being lin, ang velocity
        # Define the K constants for P, I, and D
        # For sim tuning, lin and ang are flipped.
        # For Testing angular tuning only change the first value

        # Define the errors
        # lin_error_dist = np.sqrt((lin_error[0])**2 + (lin_error[1])**2)
        curr_error = np.array([distance - setpoint, ang_error])

        print("curr_error: ", curr_error)
        self._error_total += curr_error

        # Computing PID errors
        ctrl = curr_error * KP
        ctrl += self._error_total * KI
        ctrl += (curr_error - self._prev_error) * KD

        # Set current error to previous error
        self._prev_error = curr_error

        # Converting errors into the lin_vel, ang_vel

        twist = np.zeros(2)  # lin, ang velocity
        twist[0] = -ctrl[0]
        twist[1] = ctrl[1]

        print("twist", twist)

        """For real robot"""
        ctrl = diff_drive_model(
            twist
        )  # computes wheel velocities from lin, ang vel (twist)

        self._effort_msg.left_drive = constrain(ctrl[0])
        self._effort_msg.right_drive = constrain(ctrl[1])

        print("left: ", constrain(ctrl[0]))
        print("right: ", constrain(ctrl[1]))

        """ For simulation (lin and ang vel swapped)
        self._effort_msg.linear.x = -ctrl[1]
        self._effort_msg.angular.z = -ctrl[0]
        """

    def loop(self):
        self._effort_pub.publish(self._effort_msg)  # Sends ctrl to robot
        pass

    def stop(self):
        self._effort_msg.left_drive = 0
        self._effort_msg.right_drive = 0
        self._effort_pub.publish(self._effort_msg)  # Sends ctrl to robot


if __name__ == "__main__":
    rospy.init_node("homing_controller_node")

    ctrl = HomingController()
    r = rospy.Rate(20)

    def shutdown_hook():
        ctrl.stop()
        print("stopping homing control")

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        ctrl.loop()
        r.sleep()
