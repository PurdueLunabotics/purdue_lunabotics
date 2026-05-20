from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

class MainWaitForAlignState(State):
    def __init__(self):
        self.mini_offset: TransformStamped = None

    def setup(self, manager: Node) -> Future | None:
        self.aligned_msg_publisher = manager.create_publisher(Bool, "/behavior/main_first_aligned", 10)
        manager.create_subscription(TransformStamped, "/behavior/mini_apriltag_offset", self.transform_callback, 10)

    def transform_callback(self, msg: TransformStamped):
        self.mini_offset = msg
    
    def start(self):
        self.mini_offset = None

    def periodic(self) -> None | Events:

        # Tell the mini that you are ready, continuously
        msg = Bool()
        msg.data = True
        self.aligned_msg_publisher.publish(msg)

        # when we have the offset, continue to next state
        if (self.mini_offset is not None):
            return Events.SUCCESS
        
        return None

    def exit(self, event):
        pass
