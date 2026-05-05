from math import atan2
from lunabot_behavior.states.align_to_angle import AlignToAngle
from rclpy.node import Node
from lunabot_msgs.msg import Linkup
from std_msgs.msg import Bool

class AlignToLinkup(AlignToAngle):
  def __init__(self):
    super().__init__(0)

  def setup(self, manager: Node):
    manager.create_subscription(Linkup, "linkup_pos", self.linkup_cb, 10)
    super().setup(manager)

    self.aligned_msg_publisher = manager.create_publisher(Bool, "/behavior/main_first_aligned", 10)

  def linkup_cb(self, linkup: Linkup):
    x = linkup.main_target.x - linkup.mini_target.x
    y = linkup.main_target.y - linkup.mini_target.y
    self.target_angle = atan2(y, x)

  def start(self):
    return super().start()

  def periodic(self):
    return super().periodic()

  def publish_aligned_msg(self):
    msg = Bool()
    msg.data = True

    for i in range(10):
      self.aligned_msg_publisher.publish(msg)

  def exit(self, event):
    self.publish_aligned_msg()
    super().exit(event)
