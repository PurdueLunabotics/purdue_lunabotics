#!/usr/bin/python3

from rclpy.node import Node
import rclpy
import math

from lunabot_msgs.msg import RobotEffort
from std_msgs.msg import Float64MultiArray


class DrivetrainController(Node):
    def __init__(self, **kwargs):
        super().__init__('drivetrain_controller_node', **kwargs)

        self._gearbox_ratio = 50.0

        self.effort_subscriber = self.create_subscription(RobotEffort, "/effort", self.effort_callback, 1)
        self.velocity_commands_publisher = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)

    def scale_rpm(self, rpm):
        return rpm / 60.0 * 2.0 * math.pi / self._gearbox_ratio

    def effort_callback(self, effort: RobotEffort):
        output = Float64MultiArray()
        
        left = self.scale_rpm(float(effort.left_drive))
        right = self.scale_rpm(float(effort.right_drive))

        output.data = [left, left,
                       right, right]

        self.velocity_commands_publisher.publish(output)


def main():
    rclpy.init()
    controller = DrivetrainController()
    rclpy.spin(controller)
