from rclpy.node import Node

from lunabot_behavior.state import State, Events

from geometry_msgs.msg import Twist

class RetreatBerm(State):
  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.manager = manager
    self.linear_speed = 0.1 #m/s
    self.stalled = False
    self.elapsed = 0
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
    self.move_time = 7
    if self.stalled:
      self.move_time -= self.elapsed
      self.stalled = False
  
  def periodic(self) -> None | Events:
    output = Twist()
    output.linear.x = -self.linear_speed
    self.cmd_vel_publisher.publish(output)
    if (self.start_time.seconds_nanoseconds()[0] + self.move_time < self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
  
  def exit(self, event):
    self.cmd_vel_publisher.publish(Twist())
    if event is Events.STALL:
      self.stalled = True
      self.elapsed += self.manager.get_clock().now().seconds_nanoseconds()[0] - self.start_time.seconds_nanoseconds()[0]
    else:
      self.elapsed = 0
