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


# clips input is within limits bounds
def constrain(unconstr_input):
    return np.int8(min(unconstr_input * 128, 127))


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
            AprilTagDetectionArray,
            self.apritag_cb,
        )

        self._effort_pub_ = rospy.Publisher("effort_homing", RobotEffort, queue_size=1)
        # self._effort_pub_ = rospy.Publisher('linear', Float32, queue_size=1);
        # self._effort_pub_ = rospy.Publisher('Angle', Float32, queue_size=1)

        self._effort_msg = RobotEffort()
        # self._effort_msg = Float32()
        self._prev_error = np.zeros(2)
        self._error_total = np.zeros(2)

    def apritag_cb(self, msg):

        T_camera_april_msg = msg.detections[
            0
        ].pose.pose.pose  # transformation from apriltag to camera frame (check header frame_id to be sure)
        T_camera_april = ros_numpy.numpify(
            T_camera_april_msg
        )  # Converting the transformation matrix to a numpy array

        # Assigning the unit vector values for the different vectors
        v_1A = np.array([0, 0, -1, 1])
        v_2C = np.array([1, 0, 0, 1])

        # Computing the location of point V that is in frame A to transformed to its location in frame C
        v_1C = T_camera_april @ v_1A
        v_1C[2] = 0

        # Finding the error angle between the two points, v_1C and v_2C, using dot product (in radians)
        cos_of_angle = v_2C.dot(v_1C) / (np.linalg.norm(v_2C) * np.linalg.norm(v_1C))
        ang_error = np.arccos(cos_of_angle)

        # determining the linear translational error (in x and y) of the robot considered

        # distance = m.dist(F_2C, F_1C_final)

        # lin_error = np.array([distance * m.cos(ang_error)], [distance * m.sin(ang_error)])

        # write PD controller here with the output being lin, ang velocity
        # Define the K constants for P, I, and D
        KP = np.array([0.1, 0.1])
        KI = np.array([0, 0])
        KD = np.array([0.01, 0.01])

        # Define the errors
        error_lin = 0
        curr_error = np.array([error_lin, ang_error])
        self._error_total += curr_error

        # Computing PID errors
        ctrl = curr_error * KP
        ctrl += self._error_total * KI
        ctrl += (curr_error - self._prev_error) * KD

        # Set current error to previous error
        self._prev_error = curr_error

        # Converting errors into the lin_vel, ang_vel

        twist = np.zeros(2)  # lin, ang velocity
        twist[0] = ctrl[0]
        twist[1] = ctrl[1]
        ctrl = diff_drive_model(
            ctrl
        )  # computes wheel velocities from lin, ang vel (twist)

        self._effort_msg.left_drive = constrain(ctrl[0])
        self._effort_msg.right_drive = constrain(ctrl[1])

    def loop(self):
        self._effort_pub_.publish(self._effort_msg)  # Sends ctrl to robot

    def stop(self):
        # self._effort_msg.left_drive = 0
        # self._effort_msg.right_drive = 0

        self._exc_latch_val = 0
        self._exc_latch = False


if __name__ == "__main__":
    rospy.init_node("homing_controller_node")

    ctrl = HomingController()
    r = rospy.Rate(20)

    def shutdown_hook():
        print("stopping manual control")
        ctrl.stop()

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        ctrl.loop()
        r.sleep()

    rospy.spin()
