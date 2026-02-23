from rclpy.node import Node

from lunabot_behavior.state import State, Events

from geometry_msgs.msg import Twist

class ApproachTrench(State):
  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.manager = manager
    self.move_time = 0 #seconds
    self.num_cycles = 7 # cycles before stepping forward
    self.time_step = 10 #seconds
    self.linear_speed = 0.2 #m/s
    self.counter = 0
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
    self.counter += 1
    if self.counter %self.num_cycles == 0:
      self.move_time += self.time_step
    
  
  def periodic(self) -> None | Events:
    output = Twist()
    output.linear.x = self.linear_speed
    self.cmd_vel_publisher.publish(output)
    if (self.start_time.seconds_nanoseconds()[0] + self.move_time <= self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())
