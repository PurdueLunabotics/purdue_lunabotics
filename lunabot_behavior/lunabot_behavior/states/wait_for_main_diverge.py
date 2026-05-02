from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool

class WaitForMainDivergeState(State):
    """
    Mini waiting for main to leave safely
    """
    
    def __init__(self):
        self.received_diverge_msg = False

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(Bool, "/behavior/main_diverged", self.aligned_msg_callback, 10)

    def aligned_msg_callback(self, msg: Bool):
        self.received_diverge_msg = msg
    
    def start(self):
        self.received_diverge_msg = False

    def periodic(self) -> None | Events:

        # when we have the offset, continue to next state
        if (self.received_diverge_msg):
            return Events.SUCCESS
    

        return None

    def exit(self):
        pass
