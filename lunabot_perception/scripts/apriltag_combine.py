#!/usr/bin/env python3

from rclpy.node import Node, ParameterDescriptor, ParameterType
from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy

class ApriltagCombine(Node):
    def __init__(self, **kwargs):
        super().__init__("apriltag_combine_node", **kwargs)

        self.declare_parameter("input_topics", ["example"], ParameterDescriptor(type = ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter("input_tags", [11, 7], ParameterDescriptor(type = ParameterType.PARAMETER_INTEGER_ARRAY))

        self.subs = [self.create_subscription(AprilTagDetectionArray, topic, self.detection_cb, 10)
                     for topic in self.get_parameter("input_topics").get_parameter_value().string_array_value]
        self.pub = self.create_publisher(AprilTagDetectionArray, "detections", 1)
        self.input_tags = self.get_parameter("input_tags").get_parameter_value().integer_array_value

    def detection_cb(self, detections: AprilTagDetectionArray):
        detections.detections = list(filter(lambda detection: detection.id in self.input_tags, detections.detections))
        self.pub.publish(detections)

def main():
    rclpy.init()
    node = ApriltagCombine()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
