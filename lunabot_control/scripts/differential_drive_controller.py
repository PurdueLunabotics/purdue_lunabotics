#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Twist

from lunabot_msgs.msg import RobotEffort, RobotState


def ang_delta(deg, prev_deg):
    raw = deg - prev_deg
    if raw == 0:
        return 0
    turn = min(raw % 360.0, -raw % 360.0)
    dir_ = abs(raw) / raw if turn == raw else -abs(raw) / raw
    return turn * dir_


class DifferentialDriveController:
    width = 0.5588  # Robot width, TODO find
    max_speed_percentage = 0.25  # Maximum speed we're allowing drive motors to spin
    HZ = 20

    def __init__(self):

        # ROS Publishers and subsribers to get / send data

        self._vel_sub = rospy.Subscriber("/cmd_vel", Twist, self._vel_cb)
        self._effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
        self._state_sub = rospy.Subscriber("state", RobotState, self._robot_state_cb)

        self.lin = 0
        self.ang = 0

        # Variables for PIDF Velocity Control

        self.prev_left_vel_reading = 0
        self.prev_right_vel_reading = 0
        self.prev_left_reading = 0
        self.left_reading = 0
        self.prev_right_reading = 0
        self.right_reading = 0

        rate = rospy.Rate(self.HZ)
        rospy.on_shutdown(self.shutdown_hook)

        while not rospy.is_shutdown():
            self._loop()
            rate.sleep()

    def _vel_cb(self, vel_msg):
        self.lin = vel_msg.linear.x
        self.ang = vel_msg.angular.z

    def _robot_state_cb(self, msg):
        self.right_prev_reading = self.right_reading
        self.right_reading = msg.drive_right_ang

        self.left_prev_reading = self.left_reading
        self.left_reading = msg.drive_left_ang

    def _loop(self):
        effort_msg = RobotEffort()

        left = self.lin - self.ang * self.width / 2
        right = self.lin + self.ang * self.width / 2

        max_speed = 0.62  # TODO find max speed of motors in meters / second
        p = 0.00001  # P gain for PD controller
        d = 0.000004  # D gain for PD controller

        # Feed forward
        left_percent_estimate = np.clip(left / max_speed, -1, 1)
        right_percent_estimate = np.clip(right / max_speed, -1, 1)

        meters_per_tick = 0.1397  # TODO find
        encoder_dt = 0.01  # Amount of time between readings
        loop_dt = 1 / self.HZ  # Amount of time between calls of this function

        # Measured Velocity in m / s
        left_vel_reading = (
            ang_delta(self.left_reading, self.prev_left_reading)
            / encoder_dt
            * meters_per_tick
        ) * 3
        right_vel_reading = (
            ang_delta(self.right_reading, self.prev_right_reading)
            / encoder_dt
            * meters_per_tick
        )

        # Calculating error
        left_error = left - left_vel_reading
        right_error = right - right_vel_reading

        # Calculating previous error
        left_prev_error = left - self.prev_left_vel_reading
        right_prev_error = right - self.prev_right_vel_reading

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
        effort_msg.left_drive = self.constrain(left)
        effort_msg.right_drive = self.constrain(right)
        self._effort_pub.publish(effort_msg)

    def constrain(self, val):
        val = np.clip(-1, val, 1)  # Clipping speed to not go over 100%
        return np.int8(val * 127 * self.max_speed_percentage)

    def shutdown_hook(self):
        effort_msg = RobotEffort()
        self._effort_pub.publish(effort_msg)
        print("stopping diff_drive control")


if __name__ == "__main__":
    rospy.init_node("differential_drive_controller")
    controller = DifferentialDriveController()
