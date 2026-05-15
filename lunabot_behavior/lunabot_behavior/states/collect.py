from rclpy.node import Node

from lunabot_behavior.state import State, Events

from std_msgs.msg import Bool, Int32

class CollectRegolithState(State):
  """
  Mini waiting for regolith from main
  """
  
  def __init__(self):
    self.received_empty_msg = False
    self.DEPOSIT_SPEED = 300

  def setup(self, manager: Node):
    self.dep_publisher = manager.create_publisher(Int32, "deposition", 10)
    manager.create_subscription(Bool, "/behavior/main_empty", self.empty_msg_callback, 10)

  def empty_msg_callback(self, msg: Bool):
    self.received_empty_msg = msg

  def start(self):
    self.received_empty_msg = False
  
  def periodic(self) -> None | Events:

    # move deposition slowly
    self.dep_publisher.publish(Int32(data = self.DEPOSIT_SPEED))

    if (self.received_empty_msg):
      return Events.SUCCESS
    
    return None  
  
  def exit(self, event):
    self.dep_publisher.publish(Int32(data = 0))
    pass
