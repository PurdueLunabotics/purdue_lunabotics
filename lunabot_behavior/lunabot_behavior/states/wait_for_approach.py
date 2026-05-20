from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool

class WaitForApproachState(State):
    """
    Main waiting for mini to approach 
    """
    
    def __init__(self):
        self.received_aligned_msg = False

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(Bool, "/behavior/mini_aligned", self.aligned_msg_callback, 10)

    def aligned_msg_callback(self, msg: Bool):
        self.received_aligned_msg = msg
    
    def start(self):
        self.received_aligned_msg = False

    def periodic(self) -> None | Events:

        # when we have the offset, continue to next state
        if (self.received_aligned_msg):
            return Events.SUCCESS
    

        return None

    def exit(self, event):
        pass
