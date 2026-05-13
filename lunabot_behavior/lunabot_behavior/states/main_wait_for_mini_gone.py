from lunabot_behavior.state import Events, State
import math
from rclpy.node import Node
from rclpy.task import Future
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool

class MainWaitForMiniGoneState(State):
    def __init__(self):
        self.mini_position: PoseStamped = None
        self.position: PoseStamped = None
        self.node: Node = None

        self.DIST_THRESHOLD = 3 # in meters, how far the main/mini separation has to be

        self.TIMEOUT = 30 # seconds

    def setup(self, manager: Node) -> Future | None:
        manager.create_subscription(PoseStamped, "/mini/position", self.mini_position_callback, 10)
        manager.create_subscription(PoseStamped, "/position", self.position_callback, 10)

        self.node = manager

    def mini_position_callback(self, msg: PoseStamped):
        self.mini_position = msg

    def position_callback(self, msg: PoseStamped):
        self.position = msg
    
    def start(self):
        self.mini_position = None
        self.position = None

        self.start_time = self.node.get_clock().now()

    def periodic(self) -> None | Events:
        elapsed_time = self.node.get_clock().now() - self.start_time
        elapsed_time = elapsed_time.nanoseconds / 1_000_000_000  # convert to seconds

        if (elapsed_time > self.TIMEOUT):
            return Events.SUCCESS

        if (self.mini_position is None or self.position is None):
            return None

        # NOTE: We can't get the MINI's current position in our own frame (map), only /mini/map
        # Therefore this relies on the fact that INIT_MAP set up each robot to be consistent
        # in the map frame, and as such, is no longer a valid assumption if the mini loses its own 
        # map frame relative to main,

        # Therefore- this (SOMETIMES) wrong comparison here
        dist = math.sqrt((self.position.pose.position.x - self.mini_position.pose.position.x)**2 +
                         (self.position.pose.position.y - self.mini_position.pose.position.y)**2 + 
                         (self.position.pose.position.x - self.mini_position.pose.position.z)**2)
        
        self.node.get_logger().info(f"distance between bots:{dist}/{self.DIST_THRESHOLD}")
        
        if (dist >= self.DIST_THRESHOLD):
            return Events.SUCCESS
        
        return None

    def exit(self, event):
        pass
