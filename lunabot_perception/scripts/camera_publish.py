import cv2

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter("camera_port", 0)
        self.camera_port = self.get_parameter("camera_port").get_parameter_value()

        self.get_logger().info(f'Received argument: {self.camera_port}')
        
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)

        self.bridge = CvBridge()

        # Initialize with camera port
        self.camera = cv2.VideoCapture(self.camera_port)

        if not self.camera.isOpened():
            self.get_logger().error("Could not open USB camera")
            return

        # Create a timer which publishes 30 fps
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

        self.get_logger().info("Camera publisher started")

    def publish_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error("Failed to capture image from camera")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def main():
    rclpy.init()

    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()