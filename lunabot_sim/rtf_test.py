#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time


class SimTimeMonitor(Node):
    def __init__(self):
        super().__init__('sim_time_monitor')
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.prev_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        self.get_logger().info(f"time factor: {1 / (current_time - self.prev_time)}")
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SimTimeMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
