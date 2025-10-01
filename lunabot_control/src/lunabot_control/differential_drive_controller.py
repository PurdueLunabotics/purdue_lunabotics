#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import threading

from lunabot_msgs.msg import RobotSensors



class DifferentialDriveController(Node):
    def __init__(self, **kwargs):
        super().__init__('differential_drive_controller_node', **kwargs)
        rclpy.get_global_executor().add_node(self)

        self.declare_parameter("~width", 0.5588)
        self.declare_parameter("~max_speed_percentage", 0.8)
        self.declare_parameter("~hz", 20.0)
        self.declare_parameter("~max_speed", 1000.0)
        self.declare_parameter("autonomy", True)

        # ROS Publishers and subsribers to get / send data

        self._vel_sub = self.create_subscription(Twist, "/cmd_vel", self._vel_cb, 1)
        self._right_drive_pub = self.create_publisher(Int32, "/right_drive", 10)
        self._left_drive_pub = self.create_publisher(Int32, "/left_drive", 10)
        self._state_sub = self.create_subscription(RobotSensors, "sensors", self._robot_state_cb, 1)

        self.width = self.get_parameter("~width").get_parameter_value().double_value
        self.max_speed_percentage = self.get_parameter("~max_speed_percentage").get_parameter_value().double_value
        self.hz = self.get_parameter("~hz").get_parameter_value().double_value

        self._max_speed = self.get_parameter("~max_speed").get_parameter_value().double_value # In rad/s converted to RPM
        
        self.lin = 0
        self.ang = 0

        self._left_vel = 0
        self._right_vel = 0
        self._wheel_diameter = 0.3429 # converts rad /s to m / s
        self._gearbox_ratio = 50
        self._left_prev_error = 0
        self._right_prev_error = 0

        self.left_error_sum = 0
        self.right_error_sum = 0

        # Variables for PIDF Velocity Control

        self.loop_dt = 1 / self.hz  # Amount of time between calls of this function
        rate = self.create_rate(self.hz, self.get_clock())
        # rospy.on_shutdown(self.shutdown_hook)

        while rclpy.ok():
            if (self.get_parameter("autonomy").get_parameter_value().bool_value):
                self._loop()
            rate.sleep()

        self.shutdown_hook()

    def _vel_cb(self, vel_msg):
        self.lin = vel_msg.linear.x
        self.ang = vel_msg.angular.z

    def _robot_state_cb(self, msg):
        self._right_vel = msg.drive_right_vel
        self._left_vel = msg.drive_left_vel

    def _loop(self):
        left_drive_msg = Int32()
        right_drive_msg = Int32()

        WEIGHT = 1

        self.lin *= WEIGHT
        self.ang *= 1/WEIGHT

        left_set = self.lin - self.ang * self.width / 2
        right_set = self.lin + self.ang * self.width / 2

        left_drive_msg.data = int(self.constrain(self.meters_per_sec_to_rpm(left_set)))
        right_drive_msg.data = int(self.constrain(self.meters_per_sec_to_rpm(right_set)))

        if self.lin == 0 and self.ang == 0:
            left_drive_msg.data = 0
            right_drive_msg.data = 0
        self._left_drive_pub.publish(left_drive_msg)
        self._right_drive_pub.publish(right_drive_msg)

    def constrain(self, val):
        val = np.clip(-self._max_speed, val, self._max_speed)  # Clipping speed to not go over 100%
        return np.int32(val)

    def shutdown_hook(self):
        left_drive_msg = Int32()
        right_drive_msg = Int32()

        left_drive_msg.data = 0
        right_drive_msg.data = 0

        self._left_drive_pub.publish(left_drive_msg)
        self._right_drive_pub.publish(right_drive_msg)
        self.get_logger().info("Differential Drive Controller: Stopping")
        
    def meters_per_sec_to_rpm(self, vel):
        return vel * 60 / np.pi / self._wheel_diameter * self._gearbox_ratio


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException: # Not defined
        pass

def main():
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    controller = DifferentialDriveController()
    controller.run_node()
