from rclpy.node import Node

from lunabot_behavior.state import State, Events

from std_msgs.msg import Bool

class CollectRegolithState(State):
  """
  Mini waiting for regolith from main
  """
  
  def __init__(self):
    self.received_empty_msg = False

  def setup(self, manager: Node):
    manager.create_subscription(Bool, "/behavior/main_empty", self.empty_msg_callback, 10)

  def empty_msg_callback(self, msg: Bool):
    self.received_empty_msg = msg

  def start(self):
    self.received_empty_msg = False
  
  def periodic(self) -> None | Events:
    if (self.received_empty_msg):
      return Events.SUCCESS
    
    return None  
  
  def exit(self):
    pass