from math import atan2
from lunabot_behavior.states.align_to_angle import AlignToAngle
from rclpy.node import Node
from lunabot_msgs.msg import Linkup

class AlignToLinkup(AlignToAngle):
  def __init__(self):
    super().__init__(0)

  def setup(self, manager: Node):
    manager.create_subscription(Linkup, "linkup_pos", self.linkup_cb, 10)
    super().setup(manager)

  def linkup_cb(self, linkup: Linkup):
    x = linkup.main_target.x - linkup.mini_target.x
    y = linkup.main_target.y - linkup.mini_target.y
    self.target_angle = atan2(y, x)

  def start(self):
    return super().start()

  def periodic(self):
    return super().periodic()

  def exit(self, event):
    super().exit(event)
