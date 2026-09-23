import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import os
import yaml

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter("camera_port", 0)
        self.camera_port = self.get_parameter("camera_port").get_parameter_value()
        self.get_logger().info(f'Received argument: {self.camera_port}')

        self.image_publisher = self.create_publisher(Image, 'camera/image', 10)
        self.info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # TODO put in path for camera calibration
        self.yaml_path = os.path.expanduser('camera_calibration.yaml')
        self.camera_info_msg = self.load_camera_info(self.yaml_path)

        self.bridge = CvBridge()

        # Initialize with camera port
        self.camera = cv2.VideoCapture(self.camera_port)

        if not self.camera.isOpened():
            self.get_logger().error("Could not open USB camera")
            return

        # Create a timer which publishes 30 fps
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info("Camera publisher started")

    def load_camera_info(self, yaml_path):
        msg = CameraInfo()

        # check for the yaml file
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Calibration file not found at {yaml_path}! Low-key gonna publish blank camera info now.")

        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        # process yaml calibration data into camera info msg
        msg.width = data['image.width']
        msg.height = data['image.height']
        msg.distortion_model = data['distortion_model']

        msg.d = data['distortion_coefficients']['data']
        msg.k = data['camera_matrix']['data']
        msg.r = data['rectification_matrix']['data']
        msg.p = data['projection_matrix']['data']

        return msg

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error("Failed to capture image from camera")
            return

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        time_stamp = self.get_clock().now().to_msg()

        self.image_msg.header.stamp = time_stamp
        self.image_msg.header.frame_id = 'camera_optical_frame'   

        self.camera_info_msg.header.stamp = time_stamp
        self.camera_info_msg.header.frame_id = 'camera_optical_frame'
        
        # Publish the messages
        self.image_publisher.publish(image_msg)
        self.info_publisher.publish(self.camera_info_msg)
        self.get_logger().info('Publishing CameraInfo parameters...', once=True)

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