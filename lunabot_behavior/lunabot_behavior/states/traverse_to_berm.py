from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.traverse import Traverse
from lunabot_behavior import zones

TARGET_DIST_TO_BERM = 1.5

class TraverseToBerm(Traverse):
    def __init__(self, backwards: bool):
        berm_center = zones.zone_to_poly(zones.berm_zone).centroid
        pose = PoseStamped()
        pose.header.stamp = Time()
        pose.header.frame_id = "map"
        pose.pose.position.x = berm_center.x
        pose.pose.position.y = berm_center.y
        pose.pose.position.y += -TARGET_DIST_TO_BERM if pose.pose.position.y > 0 else TARGET_DIST_TO_BERM
        super().__init__(pose, backwards)

    def setup(self, manager):
        ns = manager.get_namespace().lstrip('/')
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"
        self.goal.header.frame_id = self.frame
        super().setup(manager)
