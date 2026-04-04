from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from geometry_msgs.msg import TransformStamped

class MainWaitForAlignState(State):
    def __init__(self):
        self.mini_offset: TransformStamped = None

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(TransformStamped, "/behavior/mini_apriltag_offset", self.transform_callback, 10)

    def transform_callback(self, msg: TransformStamped):
        self.mini_offset = msg
    
    def start(self):
        self.mini_offset = None

    def periodic(self) -> None | Events:

        # when we have the offset, continue to next state
        if (self.mini_offset is not None):
            return Events.SUCCESS
        
        return None

    def exit(self):
        pass
