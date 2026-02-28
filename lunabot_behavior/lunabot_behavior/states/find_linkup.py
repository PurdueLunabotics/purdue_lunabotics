#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Point
from lunabot_msgs.msg import Linkup
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

from lunabot_behavior.state import Events
from lunabot_behavior.states.traverse import Traverse
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool
from lunabot_behavior.zones import ZoneMeasurements, get_distance_from_exc, get_distance_from_berm
from lunabot_behavior.util import point_from_pose_2d

from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

import numpy as np
from shapely.geometry import LineString

class FindLinkup(Traverse):
    def __init__(self):
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = ZoneMeasurements.BERM_OFFSET_X
        self.goal.pose.position.y = 0.0

        self.goal_vec = point_from_pose_2d(self.goal)

        super().__init__(self.goal, False)

        # planning target for action server call
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "map"
        self.start_pose.pose.position.x = ZoneMeasurements.START_OFFSET_X
        self.start_pose.pose.position.y = ZoneMeasurements.START_OFFSET_Y
        self.start_vec = point_from_pose_2d(self.start_pose)

        self.MIN_SEGMENT_LENGTH = 1.325

        # excavation edge for linkup
        self.exc_p1 = np.array([ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                ZoneMeasurements.EXC_OFFSET_Y + (ZoneMeasurements.EXC_LENGTH_Y / 2)])
        self.exc_p2 = np.array([ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                ZoneMeasurements.EXC_OFFSET_Y - (ZoneMeasurements.EXC_LENGTH_Y / 2)])
        
        self.excavation_edge = LineString([self.exc_p1, self.exc_p2])
        
        self.linkup_found = False

    def setup(self, manager: Node):
        self.goal_pub = manager.create_publisher(PoseStamped, "goal", 10)
        self.backwards_pub = manager.create_publisher(Bool, "traversal/backwards", 10)
        self.enabled_pub = manager.create_publisher(Bool, "traversal/enabled", 10)
        self.linkup_pub = manager.create_publisher(Linkup, "linkup_pos", 10)
        self.path_publisher = manager.create_publisher(Path, "linkup_path", 10)
        self.linkup_line_pub = manager.create_publisher(Marker, "linkup_segment", 10)
        self.exc_edge_pub = manager.create_publisher(Marker, "exc_edge", 10)

        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 10)

        self.odom = None
        self.tolerance = 0.3 # wider tolerance is ok - finding linkup isn't an exact science
        self.logger = manager.get_logger()

        self.path_client = ActionClient(manager, ComputePathToPose, "compute_path_to_pose")

        self.manager = manager

    def odom_cb(self, pose: PoseStamped):
        self.odom = point_from_pose_2d(pose)

    def periodic(self) -> None | Events:
        self.publish_everything()
        if self.linkup_found:
            return Events.SUCCESS

        if self.odom is not None and np.linalg.norm(self.odom - self.goal_vec) <= self.tolerance:
            self.find_linkup() # find the linkup thingamabob

        return None
    
    def exit(self):
        pass
        # destroy all publishers created
        # self.manager.destroy_publisher(self.goal_pub)
        # self.manager.destroy_publisher(self.backwards_pub)
        # self.manager.destroy_publisher(self.enabled_pub)
        # self.manager.destroy_publisher(self.linkup_pub)
        # self.manager.destroy_publisher(self.path_publisher)
        # self.manager.destroy_publisher(self.linkup_line_pub)
        # self.manager.destroy_publisher(self.exc_edge_pub)

        # # destroy all subscribers created
        # self.manager.destroy_subscription(self.odom_sub)

        # # destroy action client
        # self.manager.destroy_client(self.path_client)


    # helper functions ===========================================================================

    def find_linkup(self):
        print("finding linkup")
        self.goal.header.stamp = self.manager.get_clock().now().to_msg()

        self.find_segment(self.start_pose)

    def find_segment(self, end: PoseStamped):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start_pose
        goal_msg.goal = end

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
        path = future.result().result.path
        self.path_publisher.publish(path)

        # actually determine linkup segment
        segment = self.linkup_seg_from_path(path)
        self.visualize_line_segment(segment, self.linkup_line_pub)
        self.visualize_line_segment([self.exc_p1, self.exc_p2], self.exc_edge_pub, g=1.0)
        self.linkup_found = True

        # publish result
        linkup_msg = self.build_linkup_msg(segment)
        self.linkup_pub.publish(linkup_msg)

    def build_linkup_msg(self, linkup_segment: list[np.array]):
        linkup = Linkup()
        if linkup_segment is not None:
            # main target should be closer to berm (low dist), mini target closer to berm (high dist)
            p1_dist = get_distance_from_exc(linkup_segment[0])
            p2_dist = get_distance_from_exc(linkup_segment[1])
            if (p1_dist == p2_dist): # tiebreaker - subtract distance to berm
                p1_dist -= get_distance_from_berm(linkup_segment[0])
                p2_dist -= get_distance_from_berm(linkup_segment[1])

            main_target = linkup_segment[0] if p1_dist < p2_dist else linkup_segment[1]
            mini_target = linkup_segment[0] if p1_dist > p2_dist else linkup_segment[1]

            linkup.main_target = Point()
            linkup.main_target.x = main_target[0]
            linkup.main_target.y = main_target[1]

            linkup.mini_target = Point()
            linkup.mini_target.x = mini_target[0]
            linkup.mini_target.y = mini_target[1]

            self.visualize_line_segment(linkup_segment, self.linkup_line_pub)
        else:
            linkup.main_target = Point()
            linkup.mini_target = Point()

        return linkup

    def crosses_exc_edge(self, segment: list[np.array]):
        segment_shape = LineString(segment)

        intersection_geom = self.excavation_edge.intersection(segment_shape)
        is_valid_intersection = not intersection_geom.is_empty # assumes the cross is a point
        # self.visualize_line_segment(segment, self.linkup_line_publisher, g=1.0)

        return is_valid_intersection
    
    def linkup_seg_from_path(self, path=Path) -> list[np.array]:
        if path is not None:
            # self.get_logger().info("finding linkup")

            prev_waypoint: np.array = None
            linkup_segment = None

            poses: list[PoseStamped] = path.poses

            for i, waypoint in enumerate(poses): # determine which segments have enough length
                p1 = point_from_pose_2d(waypoint)

                if prev_waypoint is not None:
                    segment = [p1, prev_waypoint]
                    is_crossing_edge = self.crosses_exc_edge(segment)

                    if is_crossing_edge:
                        # determine whether current segment is what we're looking for
                        dist = np.linalg.norm(p1 - prev_waypoint)
                        if dist >= self.MIN_SEGMENT_LENGTH:
                            # self.get_logger().info("crossing edge")
                            linkup_segment = [prev_waypoint, p1]
                            break
                        
                        # if exc crossing edge is not viable, check neighbor on berm side
                        if i > 0:
                            prev_segment = [point_from_pose_2d(poses[i-1]), prev_waypoint]
                            if self.is_viable_segment(prev_segment, self.MIN_SEGMENT_LENGTH):
                                linkup_segment = prev_segment
                                break

                        # otherwise check neighbor on excavation side
                        if i < len(poses) - 1:
                            next_segment = [p1, point_from_pose_2d(poses[i+1])]
                            if self.is_viable_segment(next_segment, self.MIN_SEGMENT_LENGTH):
                                linkup_segment = next_segment
                                break

                        # no linkup option found
                        break

                prev_waypoint = p1

            return linkup_segment

        # can't do anything if path doesn't exist
        return None
    
    def is_viable_segment(self, segment: list[np.array], min_segment_len):
        dist = np.linalg.norm(segment[0] - segment[1])
        return dist >= min_segment_len
    
    # VISUALIZATION HELPERS ==========================================================================
    
    def visualize_line_segment(self, segment: list[np.array], publisher, r=1.0, g=0.0, b=0.0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.manager.get_clock().now().to_msg()

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
