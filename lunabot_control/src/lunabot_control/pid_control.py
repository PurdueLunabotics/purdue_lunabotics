#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class PIDController(Node):
    def __init__(self, **kwargs):
        super().__init__("pid_controller", **kwargs)

       # Subscriptions
        self.create_subscription(PoseStamped, "/goal", self.__goal_callback, 1)
        self.create_subscription(Odometry, "/odom", self.__odom_callback, 1)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # PID constants
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.01
        
        # time between each refresh, so 1/self.dt is the frequency in Hz
        self.dt = 0.05

        # PID state
        self.goal_x = None
        self.current_x = None
        self.integral = 0.0
        self.prev_error = 0.0

        # Run main loop
        self.timer = self.create_timer(self.dt, self.control_loop)

    def __goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x

    def __odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x

    def control_loop(self):
        if self.goal_x is None or self.current_x is None:
            return

        # Compute error
        error = self.goal_x - self.current_x
        if (error <= .2):

            self.integral += error * self.dt  # assuming dt = 0.1s
        else:
            self.integral = 0
        
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error


        # PID formula
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Make and publish Twist
        msg = Twist()
        msg.linear.x = output
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PIDController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

