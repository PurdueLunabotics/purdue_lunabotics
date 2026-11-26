from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

class MiniWaitForFirstAlignState(State):
    def __init__(self):
        self.received_aligned_msg = False

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(Bool, "/behavior/main_first_aligned", self.align_msg_callback, 10)
    
    def align_msg_callback(self, msg: Bool):
        if (msg.data == True):
            self.received_aligned_msg = True

    def start(self):
        self.received_aligned_msg = False

    def periodic(self) -> None | Events:

      # when the main bot is done aligning, it will send msg, continue to next state.
      if (self.received_aligned_msg):
        return Events.SUCCESS

    def exit(self, event):
      pass
