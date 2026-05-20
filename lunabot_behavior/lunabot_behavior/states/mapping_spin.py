from rclpy.node import Node
from lunabot_behavior.state import State, Events

from geometry_msgs.msg import Twist

SPIN_TIME = 20
SPIN_SPEED = 0.2 # rad/s

class MappingSpin(State):
    def setup(self, manager: Node):
        self.manager = manager

        self.cmd_vel_pub = manager.create_publisher(Twist, "cmd_vel", 10)

    def start(self):
        self.start_time = self.manager.get_clock().now().nanoseconds / float(1e9)

    def periodic(self):
        curr_time = self.manager.get_clock().now().nanoseconds / float(1e9)
        if curr_time - self.start_time > SPIN_TIME:
            return Events.SUCCESS
        
        cmd_vel = Twist()
        cmd_vel.angular.z = SPIN_SPEED
        self.cmd_vel_pub.publish(cmd_vel)

        return None
    
    def exit(self, event):
        self.cmd_vel_pub.publish(Twist())