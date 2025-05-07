#!/usr/bin/env python3

import numpy as np
import math

import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from lunabot_behavior.zones import Zone

class AlignmentController:
    """
    This class aligns the robot angles based on our known position in the field
    """

    alignment_threshold = 0.1 # in rad, how close to align before stopping

    # Linear, Angular constants for PID
    KP = np.array(5.0)
    KI = np.array(0.0)
    KD = np.array(0.0)

    angular_limits = np.array([-1.0, 1.0]) #rad/s

    def __init__(self, cmd_vel_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if cmd_vel_publisher is None:
            self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node("homing_node")
        else:
            self.cmd_vel_publisher = cmd_vel_publisher


        self.is_sim = rospy.get_param("/is_sim")

        apriltag_topic = rospy.get_param("/apriltag_topic")

        self.apriltag_subscriber = rospy.Subscriber(apriltag_topic,  PoseStamped, self.apriltag_callback)

        self.apriltag_pose_in_odom: PoseStamped = None

        odom_topic = rospy.get_param("/odom_topic")

        self.odom: Odometry = None
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.rate = rospy.Rate(30)

    def apriltag_callback(self, msg: PoseStamped):
        self.apriltag_pose_in_odom = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def align_to_angle(self, angle: float):
        """
        Align to an angle in the field. Based on the start apriltag, angle in radians
        """

        rospy.loginfo("Homing: Aligning to angle")

        while self.odom is None or self.apriltag_pose_in_odom is None:
            self.rate.sleep()

        euler_angles = euler_from_quaternion([self.apriltag_pose_in_odom.pose.orientation.x, self.apriltag_pose_in_odom.pose.orientation.y, self.apriltag_pose_in_odom.pose.orientation.z, self.apriltag_pose_in_odom.pose.orientation.w])
        if (self.is_sim):
            apriltag_yaw = euler_angles[2] + np.pi / 2
        else:
            apriltag_yaw = euler_angles[2] + np.pi / 2

        # apply angle
        apriltag_yaw += angle

        error_total = 0
        curr_error = 0
        prev_error = 0

        while True:

            robot_yaw = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2]

            angular_error = apriltag_yaw - robot_yaw
            angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

            if abs(angular_error) < self.alignment_threshold:
                self.stop()
                rospy.loginfo("Homing: Done aligning")
                break


            curr_error = angular_error

            error_total += curr_error # add for I

            # Computing PID control from error
            control = curr_error * self.KP
            control += error_total * self.KI
            control += (curr_error - prev_error) * self.KD

            # Set current error to previous error (for D)
            prev_error = curr_error

            # Publish the control (and constrain it)
            cmd_vel_message = Twist()
            cmd_vel_message.linear.x = 0
            cmd_vel_message.angular.z = np.clip(control, self.angular_limits[0], self.angular_limits[1]) 
 
            self.cmd_vel_publisher.publish(cmd_vel_message)

            self.rate.sleep()

    def back_to_berm(self, berm_zone: Zone):
     
        middle_left = [
            (berm_zone.top_left[0] + berm_zone.bottom_left[0])/2,
            (berm_zone.top_left[1] + berm_zone.bottom_left[1])/2,
        ]

        middle_right = [
            (berm_zone.top_right[0] + berm_zone.bottom_right[0])/2,
            (berm_zone.top_right[1] + berm_zone.bottom_right[1])/2,
        ]

        # pt to line distance formulat (given line defined by two points)
        dist = (abs(((middle_right[0]-middle_left[0]) * (middle_left[1] - self.odom.pose.pose.position.y)) - ((middle_left[0]-self.odom.pose.pose.position.x) * (middle_right[1]-middle_left[1]))) /
                (math.sqrt((middle_right[0]-middle_left[0])**2 + (middle_right[1]-middle_right[1])**2)))

        cmd_vel = Twist()

        start_time = rospy.get_time()
        while (dist > 0.35):
            cmd_vel.linear.x = -0.2
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

            dist = (abs(((middle_right[0]-middle_left[0]) * (middle_left[1] - self.odom.pose.pose.position.y)) - ((middle_left[0]-self.odom.pose.pose.position.x) * (middle_right[1]-middle_left[1]))) /
                    (math.sqrt((middle_right[0]-middle_left[0])**2 + (middle_right[1]-middle_right[1])**2)))
            print(dist)

            if (rospy.get_time() - start_time > 5.0):
                break

        cmd_vel.linear.x = 0
        self.cmd_vel_publisher.publish(cmd_vel)

            

    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.cmd_vel_publisher.publish(cmd_vel)

if __name__ == "__main__":
    pass