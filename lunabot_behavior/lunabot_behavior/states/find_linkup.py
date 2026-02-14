from state import State, Events
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose

from geometry_msgs.msg import PoseStamped

from lunabot_behavior.zones import ZoneMeasurements

class FindLinkup(State):
    def setup(self, manager: Node):
        self.manager = manager

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "map"
        self.start_pose.pose.position.x = ZoneMeasurements.START_OFFSET_X
        self.start_pose.pose.position.y = ZoneMeasurements.START_OFFSET_Y
        self.start_pose.pose.position.z = 0.0
        self.start_pose.pose.orientation.x = 0.0
        self.start_pose.pose.orientation.y = 0.0
        self.start_pose.pose.orientation.z = 0.0
        self.start_pose.pose.orientation.w = 1.0

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = ZoneMeasurements.BERM_OFFSET_X
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self.linkup_found = False

        self.action_client = ActionClient(manager, ComputePathToPose, "compute_path_to_pose")

    def start(self):
        pass

    def get_path(self):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start_pose
        goal_msg.goal = self.goal_pose

        self.action_client.wait_for_server()

        return self.action_client.send_goal()
    
    def periodic(self):
        self.start_pose.header.stamp = self.manager.get_clock().now()
        self.goal_pose.header.stamp = self.manager.get_clock().now()

        path = self.get_path()
    
    def exit(self):
        pass