import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
import threading

class nav2_localclient(Node):
    def __init__(self):
        super().__init__("nav2_localclient_node")
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        
    def send_goal(self):
        goal_msg = FollowPath.Goal()
        goal_msg.path.header.frame_id = 'map'
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'simple_goal_checker' 
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        return future
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')
        

def main():
    rclpy.init()
    action_client = nav2_localclient()
    executor = rclpy.get_global_executor()
    try:
        executor.spin(action_client)
    except ExternalShutdownException:
        pass