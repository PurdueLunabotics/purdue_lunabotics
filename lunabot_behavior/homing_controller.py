#!/usr/bin/env python3

import numpy as np

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

import interrupts

class HomingController:
    """
    This class aligns the robot to the apriltag for deposition
    """

    # How far should the robot be from the apriltag pos / angle
    linear_setpoint = 0.8
    angular_setpoint = 0

    alignment_threshold = 0.05 # in rad, how close to align before stopping

    # Linear, Angular
    KP = np.array([0.0, 1.0])
    KI = np.array([0.0, 0.0])
    KD = np.array([0.0, 0.0])

    linear_limits = np.array([-0.5, 0.5]) #m/s
    angular_limits = np.array([-1.0, 1.0]) #rad/s

    def __init__(self, cmd_vel_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        is_sim = rospy.get_param("/is_sim")
        if is_sim:
            cam_topic = "/d455_front/camera/color/tag_detections"
        else:
            cam_topic = "/d435_backward/color/tag_detections"

        self.apriltag_subscriber = rospy.Subscriber(cam_topic,  AprilTagDetectionArray, self.apritag_callback)

        if cmd_vel_publisher is None:
            self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node("homing_controller_node")
        else:
            self.cmd_vel_publisher = cmd_vel_publisher

        self.cmd_vel = Twist()

        self.berm_apriltag_position: Pose = None
        self.berm_apriltag_header: Header = None

        odom_topic = rospy.get_param("/odom_topic")

        self.odom: Odometry = None
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.prev_error = np.zeros(2)
        self.curr_error = np.zeros(2)
        self.error_total = np.zeros(2)

        self.rate = rospy.Rate(20)

    def apritag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) != 0:

            self.berm_apriltag_position = msg.detections[0].pose.pose.pose
            self.berm_apriltag_header = msg.detections[0].pose.header
        else:
            self.berm_apriltag_position = None

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def spin_until_apriltag(self):

        # TODO look for the right apriltag bundle

        self.cmd_vel.angular.z = 0.785398 # around 45 degrees per second

        while self.berm_apriltag_position is None:
            self.cmd_vel_publisher.publish(self.cmd_vel)
            rospy.sleep(0.1)

        self.stop()

    def home(self):

        self.spin_until_apriltag()

        while (True):

            if interrupts.check_for_interrupts() != interrupts.Errors.FINE:
                return False

            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            target_frame = "odom"

            pose = tf2_geometry_msgs.PoseStamped()
            pose.header = self.berm_apriltag_header
            pose.pose = self.berm_apriltag_position

            # Set the time to 0 to get the latest available transform
            pose.header.stamp = rospy.Time(0)

            pose_in_odom = tf_buffer.transform(pose, target_frame, rospy.Duration(2.0))

            euler_angles = euler_from_quaternion([pose_in_odom.pose.orientation.x, pose_in_odom.pose.orientation.y, pose_in_odom.pose.orientation.z, pose_in_odom.pose.orientation.w])
            apriltag_yaw = euler_angles[2] + np.pi / 2  # The yaw 'out of' the face of the apriltag (adjusted by 90 degrees)

            robot_yaw = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2]

            # Define the errors

            angular_error = apriltag_yaw - robot_yaw
            angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

            # Stopping point
            if abs(angular_error) < self.alignment_threshold:
                self.stop()
                break

            # TODO right now this linear error is not right
            linear_dist = np.sqrt(pose_in_odom.pose.position.x - self.odom.pose.pose.orientation.x) ** 2 + (pose_in_odom.pose.position.y - self.odom.pose.pose.orientation.y) ** 2

            self.curr_error = np.array([self.linear_setpoint - linear_dist, angular_error])

            self.error_total += self.curr_error # add for I

            # Computing PID control from error
            control = self.curr_error * self.KP
            control += self.error_total * self.KI
            control += (self.curr_error - self.prev_error) * self.KD

            # Set current error to previous error (for D)
            self.prev_error = self.curr_error

            # Publish the control (and constrain it)
            cmd_vel_message = Twist()
            cmd_vel_message.linear.x = np.clip(control[0], self.linear_limits[0], self.linear_limits[1])
            cmd_vel_message.angular.z = np.clip(control[1], self.angular_limits[0], self.angular_limits[1])

            self.cmd_vel_publisher.publish(cmd_vel_message)

            self.rate.sleep()

        return True


    def stop(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_publisher.publish(self.cmd_vel)

if __name__ == "__main__":
    homing_controller = HomingController()
    homing_controller.home()