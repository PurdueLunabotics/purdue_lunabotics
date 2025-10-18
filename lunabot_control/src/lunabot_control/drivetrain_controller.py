#!/usr/bin/python3

from rclpy.node import Node
import rclpy

from lunabot_msgs.msg import RobotEffort
from std_msgs.msg import Float64MultiArray


class DrivetrainController(Node):
    def __init__(self, **kwargs):
        super().__init__('drivetrain_controller_node', **kwargs)

        self.effort_subscriber = self.create_subscription(RobotEffort, "/effort", self.effort_callback, 1)
        self.velocity_commands_publisher = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)

    def effort_callback(self, effort: RobotEffort):
        output = Float64MultiArray()

        output.data = [float(effort.left_drive) / 500.0, float(effort.left_drive) / 500.0,
                       float(effort.right_drive) / 500.0, float(effort.right_drive) / 500.0]

        self.velocity_commands_publisher.publish(output)


def main():
    rclpy.init()
    controller = DrivetrainController()
    rclpy.spin(controller)
