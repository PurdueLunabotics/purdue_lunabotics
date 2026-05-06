from lunabot_behavior.state import State, Events
from rclpy.node import Node
from lunabot_msgs.msg import Linkup

class WaitForLinkup(State):
    def setup(self, manager):
        self.manager = manager

        self.linkup_sub = manager.create_subscription(Linkup, "/linkup_pos", self.linkup_cb, 10)
        self.linkup_published = False

    def linkup_cb(self, msg: Linkup):
        self.linkup_published = True

    def periodic(self):
        if self.linkup_published:
            return Events.SUCCESS
