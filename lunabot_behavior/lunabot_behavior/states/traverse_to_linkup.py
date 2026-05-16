from math import atan2

from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.traverse import Traverse
from rclpy.node import Node
from lunabot_msgs.msg import Linkup
from tf_transformations import quaternion_from_euler

class TraverseToLinkup(Traverse):
  def __init__(self, is_main: bool, is_backwards: bool):
    super().__init__(PoseStamped(), is_backwards)
    self.is_main = is_main

  def setup(self, manager: Node):
    self.linkup_sub = manager.create_subscription(Linkup, "/linkup_pos", self.linkup_cb, 10)
    self.manager = manager
    super().setup(manager)

  def linkup_cb(self, linkup: Linkup):
    self.x = linkup.main_target.x - linkup.mini_target.x
    self.y = linkup.main_target.y - linkup.mini_target.y
    if self.is_main:
      self.goal.pose.position.x = linkup.main_target.x
      self.goal.pose.position.y = linkup.main_target.y
      self.goal.pose.position.z = linkup.main_target.z
    if not self.is_main:
      self.x *= -1
      self.y *= -1
      self.goal.pose.position.x = linkup.mini_target.x
      self.goal.pose.position.y = linkup.mini_target.y
      self.goal.pose.position.z = linkup.mini_target.z

    self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w = quaternion_from_euler(0, 0, atan2(self.y, self.x))
    self.goal.header.frame_id = "map" if self.is_main else "mini/map"
    self.goal.header.stamp = self.manager.get_clock().now().to_msg()

  def start(self):
    return super().start()

  def periodic(self):
    return super().periodic()

  def exit(self, event):
    super().exit(event)
