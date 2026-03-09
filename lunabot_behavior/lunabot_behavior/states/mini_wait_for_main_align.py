from lunabot_behavior.state import Events, State
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from geometry_msgs.msg import TransformStamped

class MiniWaitForAlignState(State):
    def __init__(self):
        self.transform: TransformStamped = None

        self.received_aligned_msg = False


    def setup(self, manager: Node) -> Future | None:
        self.offset_publisher = manager.create_publisher(TransformStamped, "/behavior/mini_apriltag_offset", 10)
        manager.create_subscription(TransformStamped, "/behavior/mini_apriltag_offset", self.transform_callback, 10)
        self.node = manager
    
    def transform_callback(self, msg: TransformStamped):
        self.transform = msg

    def start(self):
        self.received_aligned_msg = False

    def periodic(self) -> None | Events:

        # when the main bot is done aligning, it will send msg, continue to next state.
        if (self.received_aligned_msg):
            return Events.SUCCESS

        # This state should have received a transform/offset from the previous state, it just republishes it here.
        if (self.transform is None):
            self.node.get_logger().warn("Behavior: Mini bot is waiting but has no offset to transmit")
            return None
        
        self.offset_publisher.publish(self.transform)

    def exit(self):
        # delete the transform so it's not reused
        self.transform = None
