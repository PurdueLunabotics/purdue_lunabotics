import rclpy
from rclpy.node import Node
from rclpy.task import Future

from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

from lunabot_behavior.zones import ZoneMeasurements

import numpy as np
from shapely.geometry import LineString

class FindLinkup(Node):
    # STATE CORE FUNCTIONS ==========================================================

    def __init__(self):
        super().__init__("find_linkup_node")

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "map"
        self.start_pose.pose.position.x = ZoneMeasurements.START_OFFSET_X
        self.start_pose.pose.position.y = ZoneMeasurements.START_OFFSET_Y
        self.start_pose.pose.position.z = 0.0
        self.start_pose.pose.orientation.x = 0.0
        self.start_pose.pose.orientation.y = 0.0
        self.start_pose.pose.orientation.z = 0.0
        self.start_pose.pose.orientation.w = 1.0

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = ZoneMeasurements.BERM_OFFSET_X
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self.linkup_found = False
        self.path: Path = None

        self.path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.MIN_SEGMENT_LENGTH = 1.325

        # excavation edge for linkup
        self.exc_p1 = np.array([ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                ZoneMeasurements.EXC_OFFSET_Y + (ZoneMeasurements.EXC_LENGTH_Y / 2)])
        self.exc_p2 = np.array([ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                ZoneMeasurements.EXC_OFFSET_Y - (ZoneMeasurements.EXC_LENGTH_Y / 2)])
        
        self.excavation_edge = LineString([self.exc_p1, self.exc_p2])

        # publishers
        self.path_publisher = self.create_publisher(Path, "linkup_path", 10)
        self.linkup_line_publisher = self.create_publisher(Marker, "linkup_segment", 10)
        self.exc_edge_pub = self.create_publisher(Marker, "exc_edge", 10)

        self.start_pub = self.create_publisher(PoseStamped, "linkup_start", 10)
        self.end_pub = self.create_publisher(PoseStamped, "linkup_end", 10)

        self.create_timer(1 / 2, self.periodic)

    def start(self):
        pass
    
    def periodic(self):
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_path(self.start_pose, self.goal_pose)

        linkup_segment = self.find_linkup(self.path)
        if linkup_segment is not None:
            self.visualize_line_segment(linkup_segment, self.linkup_line_publisher)
    
    def exit(self):
        pass

    def is_finished(self):
        pass

    # HELPER FUNCTIONS ===============================================================

    def get_path(self, start: PoseStamped, end: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = end
        goal_msg.goal = start

        self.start_pub.publish(start)
        self.end_pub.publish(end)

        self.path_client.wait_for_server()

        self.future = self.path_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.get_path_cb)

    def get_path_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.path = future.result().result.path
        self.path.poses = list(self.path.poses)[::-1]
        self.path_publisher.publish(self.path)

    def crosses_exc_edge(self, segment: list[np.array]):
        segment_shape = LineString(segment)

        intersection_geom = self.excavation_edge.intersection(segment_shape)
        is_valid_intersection = not intersection_geom.is_empty # assumes the cross is a point

        self.visualize_line_segment([self.exc_p1, self.exc_p2], self.exc_edge_pub, g=1.0)
        # self.visualize_line_segment(segment, self.linkup_line_publisher, g=1.0)

        return is_valid_intersection
    
    def find_linkup(self, path=Path) -> list[np.array]:
        if path is not None:
            self.get_logger().info("finding linkup")

            visited = []
            prev_waypoint: np.array = None
            linkup_segment = None

            poses: list[PoseStamped] = path.poses

            for i, waypoint in enumerate(poses): # determine which segments have enough length
                self.get_logger().info(f"waypoint {i}")
                p1 = self.point_from_pose(waypoint)

                if prev_waypoint is not None:
                    segment = [p1, prev_waypoint]
                    is_crossing_edge = self.crosses_exc_edge(segment)

                    if is_crossing_edge:
                        self.get_logger().info("calculating things")
                        # determine whether current segment is what we're looking for
                        dist = np.linalg.norm(p1 - prev_waypoint)
                        if dist >= self.MIN_SEGMENT_LENGTH:
                            self.get_logger().info("crossing edge")
                            linkup_segment = [prev_waypoint, p1]
                            break
                        
                        # otherwise check next segment in path (closer to berm zone)
                        if i < len(poses) - 1:
                            next_segment = [p1, self.point_from_pose(poses[i+1])]
                            if self.is_viable_segment(next_segment, self.MIN_SEGMENT_LENGTH):
                                linkup_segment = next_segment
                                self.get_logger().info("berm edge")
                                break

                        # if neither option successful, check previous segment (inside exc zone)
                        if i > 0:
                            prev_segment = [self.point_from_pose(poses[i-1]), prev_waypoint]
                            if self.is_viable_segment(prev_segment, self.MIN_SEGMENT_LENGTH):
                                linkup_segment = next_segment
                                self.get_logger().info("exc edge")
                                break

                        # no linkup option found
                        break

                visited.append(p1)
                prev_waypoint = p1

            return linkup_segment

        # can't do anything if path doesn't exist
        return None
    
    def point_from_pose(self, pose: PoseStamped):
        return np.array([pose.pose.position.x, pose.pose.position.y])
    
    def is_viable_segment(self, segment: list[np.array], min_segment_len):
        dist = np.linalg.norm(segment[0] - segment[1])
        return dist >= min_segment_len
    
    def visualize_line_segment(self, segment: list[np.array], publisher, r=1.0, g=0.0, b=0.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "line_segment"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Line width
        marker.scale.x = 0.1

        # Color (RGBA)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Identity pose
        marker.pose.orientation.w = 1.0

        # Convert numpy points → geometry_msgs/Point
        for p in segment:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            marker.points.append(point)

        publisher.publish(marker)

def main():
    rclpy.init()
    node = FindLinkup()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()