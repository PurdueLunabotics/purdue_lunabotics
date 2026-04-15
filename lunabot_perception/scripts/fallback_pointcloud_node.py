#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import rclpy

class FallbackPointCloudNode(Node):
    def __init__(self, **kwargs):
        super().__init__("apriltag_combine_node", **kwargs)

        self.primary_sub = self.create_subscription(PointCloud2, "primary", self.primary_cb, 10)
        self.secondary_sub = self.create_subscription(PointCloud2, "secondary", self.secondary_cb, 10)

        self.timeout_length = self.declare_parameter("timeout", 5.0).get_parameter_value().double_value

        self.output_pub = self.create_publisher(PointCloud2, "output", 10)
        self.timeout = self.create_timer(self.timeout_length, self.timeout_cb)

        self.on_secondary = False

    def timeout_cb(self):
        self.on_secondary = True

    def primary_cb(self, points: PointCloud2):
        self.output_pub.publish(points)
        self.timeout.reset()
        self.on_secondary = False

    def secondary_cb(self, points: PointCloud2):
        if self.on_secondary:
            self.output_pub.publish(points)

def main():
    rclpy.init()
    node = FallbackPointCloudNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
