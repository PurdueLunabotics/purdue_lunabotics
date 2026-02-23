from rclpy.node import Node

from lunabot_behavior.state import State, Events

from geometry_msgs.msg import Twist
from lunabot_msgs.msg import Event


class SeparateMain(State):
  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.main_event_pub = manager.create_publisher(Event, "/events", 10)

    self.manager = manager
    self.move_time = 5 #seconds
    self.linear_speed = 0.1 #m/s
    
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    output = Twist()
    output.linear.x = -self.linear_speed # TODO: determine the direction of the mini bot
    self.cmd_vel_publisher.publish(output)
    if (self.start_time.seconds_nanoseconds()[0] + self.move_time < self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())
    self.main_event_pub.publish(Event.SUCCESS)
