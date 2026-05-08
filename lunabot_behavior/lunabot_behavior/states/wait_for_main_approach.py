from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool

class WaitForMainApproachState(State):
    """
    Mini waiting for main to approach 
    """
    
    def __init__(self):
        self.received_aligned_msg = False
        self.realign_msg = False

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(Bool, "/behavior/main_approached", self.aligned_msg_callback, 10)
        manager.create_subscription(Bool, "/behavior/main_wants_realign", self.realign_msg_callback, 10)

    def aligned_msg_callback(self, msg: Bool):
        self.received_aligned_msg = msg

    def realign_msg_callback(self, msg: Bool):
        self.realign_msg = msg
    
    def start(self):
        self.received_aligned_msg = False

    def periodic(self) -> None | Events:

        # when we have the offset, continue to next state
        if (self.received_aligned_msg):
            return Events.SUCCESS
    
        if (self.realign_msg):
            return Events.NEED_REALIGN

        return None

    def exit(self, event):
        pass
