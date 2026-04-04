import math
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.traverse import Traverse
from tf_transformations import quaternion_from_euler
from lunabot_behavior import zones
from std_srvs/srv import Empty

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

        // RTAB-Map Pausing
        self._freeze_map_client = self.create_client(Empty, '/rtabmap/rtabmap/pause')
        freeze_map()
    def freeze_map(self):
        if not self._freeze_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('The RTAB pause service is NOT AVAILABLE =_=. We will have to make do without...')
            return
            
        self.req = Empty.Request()
        future = self._freeze_map_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() != None:
            self.get_logger().info('RTAB Mapping is paused. Hooray!')
        else:
            self.get_logger().error('RTAB Mapping failed. :(')
        
