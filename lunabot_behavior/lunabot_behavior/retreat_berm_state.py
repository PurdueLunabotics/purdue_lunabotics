from rclpy.node import Node

from state import State, Events

from geometry_msgs.msg import Twist

class RetreatBerm(State):
  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.manager = manager
    self.move_time = 10 #seconds
    self.linear_speed = -0.2 #m/s
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    output = Twist()
    output.linear.x = self.linear_speed
    self.cmd_vel_publisher.publish()
    if (self.start_time.seconds_nanoseconds()[0] + self.deposit_time < self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())
