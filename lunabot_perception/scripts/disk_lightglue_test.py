#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lightglue import DISK
import rclpy

class Matcher(Node):
    def __init__(self, **kwargs):
        super().__init__("apriltag_combine_node", **kwargs)
        self.image_sub = self.create_subscription(Image, "/mini/d455_front/color/image_raw", self.image_cb, 10)
        self.bridge = CvBridge()
        self.detector = DISK(max_num_keypoints=512).eval()

    def image_cb(self, msg: Image):
        mat = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')


def main():
    rclpy.init()
    node = Matcher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
