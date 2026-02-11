from rclpy.node import Node

from state import State
from state import Events

from std_msgs.msg import Int32

class Deposit_State(State):
  def setup(self, manager: Node):
    self.dep_pub = manager.create_publisher(Int32, "deposition", 10)
    self.manager = manager
    self.deposit_time = 10 #seconds
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    self.dep_pub.publish(1000)
    if (self.start_time.seconds_nanoseconds()[0] + self.deposit_time < self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
      