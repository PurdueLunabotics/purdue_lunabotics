#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from lunabot_msgs.msg import RobotEffort


class DifferentialDriveController:
    def __init__(self):

        # ROS Publishers and subsribers to get / send data

        self._vel_sub = rospy.Subscriber("cmd_vel", Twist, self._vel_cb)
        self._effort_pub = rospy.Publisher("effort", RobotEffort, queue_size=1)
        rospy.Subscriber("left_wheel_enc", Float32, self._left_enc_cb)
        rospy.Subscriber("right_wheel_enc", Float32, self._right_enc_cb)

        # Variables for PIDF Velocity Control

        self.prev_left_vel_reading = 0
        self.prev_right_vel_reading = 0
        self.prev_left_reading = 0
        self.left_reading = 0
        self.prev_right_reading = 0
        self.right_reading = 0

    def _vel_cb(self, vel_msg):
        w = 20  # Robot width, TODO find
        lin = vel_msg.linear.x
        ang = vel_msg.angular.z
        effort_msg = RobotEffort()

        left = lin - ang * w / 2
        right = lin + ang * w / 2

        # left, right = self._vel_pidf(left, right) #Using encoder velocity pid

        effort_msg.left_drive = self.constrain(left)
        effort_msg.right_drive = self.constrain(right)

        self._effort_pub.publish(effort_msg)

    def _left_enc_cb(self, enc):
        self.left_prev_reading = self.left_reading
        self.left_reading = enc.data

    def _right_enc_cb(self, enc):
        self.right_prev_reading = self.right_reading
        self.right_reading = enc.data

    def _vel_pidf(self, left_vel, right_vel):
        max_speed = 1  # TODO find max speed of motors in meters / second
        p = 0  # P gain for PD controller
        d = 0  # D gain for PD controller

        # Feed forward
        left_percent_estimate = np.clip(left_vel / max_speed, -1, 1)
        right_percent_estimate = np.clip(right_vel / max_speed, -1, 1)

        meters_per_tick = 1  # TODO find
        encoder_dt = 0.01  # Amount of time between readings
        loop_dt = 0.1  # Amount of time between calls of this function

        # Measured Velocity in m / s
        left_vel_reading = (
            (self.left_reading - self.prev_left_reading) / encoder_dt * meters_per_tick
        )
        right_vel_reading = (
            (self.right_reading - self.prev_right_reading)
            / encoder_dt
            * meters_per_tick
        )

        # Calculating error
        left_error = left_vel - left_vel_reading
        right_error = right_vel - right_vel_reading

        # Calculating previous error
        left_prev_error = left_vel - self.prev_left_vel_reading
        right_prev_error = right_vel - self.prev_right_vel_reading

        # Calculating motor velocities
        left = (
            left_percent_estimate
            + left_error * p
            + (left_error - left_prev_error) / loop_dt * d
        )
        right = (
            right_percent_estimate
            + right_error * p
            + (right_error - right_prev_error) / loop_dt * d
        )

        # Updating previous readings
        self.prev_left_vel_reading = left_vel_reading
        self.prev_right_vel_reading = right_vel_reading

        return left, right

    def constrain(self, val):
        val = np.clip(-1, val, 1)  # Clipping speed to not go over 100%
        max_speed_percentage = 1  # Maximum speed we're allowing drive motors to spin
        return np.int8(val * 127 * max_speed_percentage)


if __name__ == "__main__":
    rospy.init_node("differential_drive_controller")

    controller = DifferentialDriveController()
    rospy.spin()
