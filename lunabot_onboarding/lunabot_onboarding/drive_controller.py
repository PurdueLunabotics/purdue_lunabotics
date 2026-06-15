from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class DriveController(Node):
    def __init__(self):
        super().__init__("drivetrain_controller_node")

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.cmd_vel = Twist()

        self.left_drive_pub = self.create_publisher(Int32, "left_drive", 10)
        self.right_drive_pub = self.create_publisher(Int32, "right_drive", 10)

        self.gear_ratio = self.declare_parameter("gear_ratio", 50.0).get_parameter_value().double_value
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.2).get_parameter_value().double_value
        self.base_width = self.declare_parameter("base_width", 0.64).get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.loop)

    def cmd_vel_cb(self, cmd_vel: Twist):
        self.cmd_vel = cmd_vel

    def speed_to_rpm(self, speed: float):
        return speed / self.wheel_radius / 2.0 / 3.14159 * self.gear_ratio * 60.0

    def loop(self):
        left_drive = self.cmd_vel.linear.x - self.cmd_vel.angular.z * self.base_width / 2
        right_drive = self.cmd_vel.linear.x + self.cmd_vel.angular.z * self.base_width / 2

        self.left_drive_pub.publish(Int32(data = int(self.speed_to_rpm(left_drive))))
        self.right_drive_pub.publish(Int32(data = int(self.speed_to_rpm(right_drive))))

def main():
    rclpy.init()
    drive_controller = DriveController()
    rclpy.spin(drive_controller)
    rclpy.shutdown()
