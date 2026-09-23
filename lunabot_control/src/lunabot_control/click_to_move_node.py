import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class ClickToMoveNode(Node):

    def __init__(self):
        super().__init__('click_to_move')

        self.action_client = ActionClient(self,NavigateToPose,'/navigate_to_pose')

        self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)

    def goal_callback(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    node = ClickToMoveNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()