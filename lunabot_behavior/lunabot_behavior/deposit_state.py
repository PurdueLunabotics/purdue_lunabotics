from rclpy.node import Node

from state import State, Events

from std_msgs.msg import Int32, Bool
from lunabot_msgs.msg import Event

class Deposit(State):
  def __init__(self, transfer, **kwargs):
    super().__init__()
    self.transfer = transfer
  def setup(self, manager: Node):
    self.dep_pub = manager.create_publisher(Int32, "deposition", 10)
    self.dep_gate_pub = manager.create_publisher(Bool, "gate", 10)
    self.mini_event_pub = manager.create_publisher(Event, "/mini/events", 10)
    self.manager = manager
    self.deposit_time = 10 #seconds
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    self.dep_pub.publish(1000)
    self.dep_gate_pub.publish(True)
    if (self.start_time.seconds_nanoseconds()[0] + self.deposit_time < self.manager.get_clock().now().seconds_nanoseconds()[0]):
      return Events.SUCCESS
    return None  
  
  def exit(self):
    self.dep_pub.publish(0)
    self.dep_gate_pub.publish(False)
    if self.transfer:
      self.mini_event_pub.publish(Event.SUCCESS)
      