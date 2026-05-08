from rclpy.node import Node

from lunabot_behavior.state import State, Events

from std_msgs.msg import Int32, Bool
from lunabot_msgs.msg import Event

class Deposit(State):
  def __init__(self, transfer=False):
    super().__init__()
    self.transfer = transfer


  def setup(self, manager: Node):
    self.dep_pub = manager.create_publisher(Int32, "deposition", 10)
    self.dep_gate_pub = manager.create_publisher(Bool, "gate", 10)
    self.empty_msg_publisher = manager.create_publisher(Bool, "/behavior/main_empty", 10)
    self.manager = manager
    self.GATE_TIME = 2 # seconds, how long gate takes to open
    self.DEPOSIT_TIME = 20 # seconds, inclusive of gate + deposit
    self.REVERSE_TIME = 20.5 # seconds, inclusive of gate + deposit + reverse
    self.DEPOSIT_SPEED = 3000
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    elapsed_time = self.manager.get_clock().now() - self.start_time
    elapsed_time = elapsed_time.nanoseconds / 1_000_000_000  # convert to seconds

    self.dep_gate_pub.publish(Bool(data = True))

    dep_speed = 0
    if (elapsed_time > self.GATE_TIME and elapsed_time < self.DEPOSIT_TIME): # start pushing material out
      dep_speed = self.DEPOSIT_SPEED

    if (elapsed_time > self.DEPOSIT_TIME): # reverse dep just slightly to pull flap in
      dep_speed = -self.DEPOSIT_SPEED

    self.dep_pub.publish(Int32(data = dep_speed))

    if (elapsed_time > self.REVERSE_TIME):
      return Events.SUCCESS
    
    return None  
  
  def exit(self, event):
    self.dep_pub.publish(Int32(data = 0))
    self.dep_gate_pub.publish(Bool(data = False))

    # if this is a transfer from mainbot to minibot, publish the 'empty' message when done
    if self.transfer:
      msg = Bool()
      msg.data = True
      for i in range(10):
        self.empty_msg_publisher.publish(msg)
      
