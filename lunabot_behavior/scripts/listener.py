#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from lunabot_behavior.utils import add_two_nums

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        print(add_two_nums(1,3))

        # Declares a parameter and a default value for it
        self.declare_parameter("my_name", "default_name")

    def listener_callback(self, msg):
        name = self.get_parameter("my_name").value
        self.get_logger().info(f"I heard: {msg.data} and name {name}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()