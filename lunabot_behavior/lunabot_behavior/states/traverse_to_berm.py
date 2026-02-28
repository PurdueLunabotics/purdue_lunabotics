import math
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.traverse import Traverse
from tf_transformations import quaternion_from_euler
from lunabot_behavior import zones

class TraverseToBerm(Traverse):
    def __init__(self, backwards: bool):
        pose = PoseStamped()
        pose.header.stamp = Time()
        pose.header.frame_id = "map"
        pose.pose.position.x = (zones.berm_zone.v2.x + zones.berm_zone.v3.x) / 2
        pose.pose.position.y = (zones.berm_zone.v2.y + zones.berm_zone.v3.y) / 2 - 0.5
        pose.pose.position.z = (zones.berm_zone.v2.z + zones.berm_zone.v3.z) / 2
        x, y, z, w = quaternion_from_euler(0, 0, -math.pi / 2)
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        super().__init__(pose, backwards)
