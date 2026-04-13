import math
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.traverse import Traverse
from tf_transformations import quaternion_from_euler
from lunabot_behavior import zones
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

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


    def freeze_map(self):
        while not self._freeze_map_client.wait_for_service(timeout_sec=1.0):
            self.manager.get_logger().info('The RTAB pause service is NOT AVAILABLE =_=. We will have to make do without...')
            return
            
        self.req = Empty.Request()
        out = self._freeze_map_client.call(self.req)
        self.future = self.__freeze_map_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        self.manager.get_logger().info(self.future.result()
        #if self.future.result() != None:
        #    self.manager.get_logger().info('RTAB Mapping is paused. Hooray!')
        #else:
        #    self.manager.get_logger().error('RTAB Mapping failed. :(')
        
    def setup(self, manager):
        self._freeze_map_client = manager.create_client(Empty, '/rtabmap/rtabmap/pause')
        ns = manager.get_namespace().lstrip('/')
        self.manager = manager
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"
        self.goal.header.frame_id = self.frame
        super().setup(manager)
    def exit(self):
        self.freeze_map()
        super().exit()
