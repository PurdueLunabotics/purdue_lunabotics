import cv2

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)

        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)

        if not self.camera.isOpened():
            self.get_logger().error("Could not open USB camera")
            return

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

        self.get_logger().info("Camera publisher started")
    


