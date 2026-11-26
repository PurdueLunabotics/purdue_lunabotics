from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool

class WaitForDivergeState(State):
    """
    Main waiting for mini to leave safely
    maybe TODO: combine this and other plain wait states with one wait state, parameterized by a topic to wait?
    """
    
    def __init__(self):
        self.received_diverge_msg = False

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(Bool, "/behavior/mini_diverged", self.aligned_msg_callback, 10)

    def aligned_msg_callback(self, msg: Bool):
        self.received_diverge_msg = msg
    
    def start(self):
        self.received_diverge_msg = False

    def periodic(self) -> None | Events:

        # when we have the offset, continue to next state
        if (self.received_diverge_msg):
            return Events.SUCCESS
    

        return None

    def exit(self, event):
        pass
