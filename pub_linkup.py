import rclpy
from rclpy.node import Node

from lunabot_msgs.msg import Linkup

class LinkupPub(Node):
    def __init__(self):
        super().__init__("linkup_pub")

        self.linkup_pub = self.create_publisher(Linkup, "/linkup_pos", 10)

        self.create_timer(0.1, self.publish)

    def publish(self):
        linkup = Linkup()
        linkup.main_target.x = 1.0
        linkup.main_target.y = -0.5

        self.linkup_pub.publish(linkup)

if __name__ == "__main__":
    rclpy.init()
    node = LinkupPub()

    rclpy.spin(node)

    rclpy.shutdown()