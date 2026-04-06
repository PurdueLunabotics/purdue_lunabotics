#!/usr/bin/env python3

from math import inf
import math
from geometry_msgs.msg import PoseStamped, Point
from lunabot_msgs.msg import Linkup
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
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
from nav2_msgs.srv import GetCostmap

import numpy as np
from shapely.geometry import LineString
import shapely.geometry as shp

import rclpy
import sys
import random

LETHAL_COST = 252

def world_to_map(costmap: Costmap, x: float, y: float):
    resolution = costmap.metadata.resolution
    origin_pos = costmap.metadata.origin.position
    return (int((x - origin_pos.x) / resolution), int((y - origin_pos.y) / resolution))

def map_to_world(costmap: Costmap, x: float, y: float) -> tuple[float, float]:
    resolution = costmap.metadata.resolution
    origin_pos = costmap.metadata.origin.position
    return (x * resolution + origin_pos.x, y * resolution + origin_pos.y)

def get_cost(costmap: Costmap, x: int, y: int) -> float:
    return 26.0 + 0.9 * int(costmap.data[x + y * costmap.metadata.size_x])

def is_blocked(costmap: Costmap, x: int, y: int) -> bool:
    return costmap.data[x + y * costmap.metadata.size_x] >= LETHAL_COST

def get_traversal_cost(costmap: Costmap, x: int, y: int):
  curr_cost = get_cost(costmap, x, y)
  return (curr_cost / LETHAL_COST) ** 2

def line_cost(costmap: Costmap, a: shp.Point, b: shp.Point):
    initial = world_to_map(costmap, a.x, a.y)
    end = world_to_map(costmap, b.x, b.y)
    current = initial
    distance_x = abs(initial[0] - end[0])
    distance_y = abs(initial[1] - end[1])

    divisor = np.gcd(distance_x, distance_y)
    dx = (end[0] - initial[0]) / divisor
    dy = (end[1] - initial[1]) / divisor

    x_increment = 0;
    if distance_x != 0:
        x_increment = int((end[0] - current[0]) / distance_x);

    y_increment = 0;
    if distance_y != 0:
        y_increment = int((end[1] - current[1]) / distance_y);

    cost = 0.0

    if distance_x > distance_y:
        while current[0] != end[0]:
            current = (current[0] + x_increment, current[1])
            target_y = dy * (current[0] - initial[0]) / dx + initial[1];
            if int(target_y) != current[1]:
                current = (current[0], current[1] + y_increment)

            # if is_blocked(costmap, current[0], current[1]):
            #     return inf

            cost += get_traversal_cost(costmap, current[0], current[1])
    else:
        while current[1] != end[1]:
            current = (current[0], current[1] + y_increment)
            target_x = dx * (current[1] - initial[1]) / dy + initial[0];
            if int(target_x) != current[0]:
                current = (current[0] + x_increment, current[1])

            # if is_blocked(costmap, current[0], current[1]):
            #     return inf

            cost += get_traversal_cost(costmap, current[0], current[1])

    return cost

class FindLinkup(Traverse):
    def __init__(self):
        self.goal = PoseStamped()
        self.goal.pose.position.x = ZoneMeasurements.BERM_OFFSET_X
        self.goal.pose.position.y = 0.0

        self.goal_vec = point_from_pose_2d(self.goal)

        super().__init__(self.goal, False)

        # planning target for action server call
        self.start_pose = PoseStamped()
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
        ns = manager.get_namespace().lstrip('/')
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"
        self.start_pose.header.frame_id = self.frame
        self.goal.header.frame_id = self.frame

        self.goal_pub = manager.create_publisher(PoseStamped, "goal", 10)
        self.backwards_pub = manager.create_publisher(Bool, "traversal/backwards", 10)
        self.enabled_pub = manager.create_publisher(Bool, "traversal/enabled", 10)
        self.linkup_pub = manager.create_publisher(Linkup, "/linkup_pos", QoSProfile(durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, depth = 10))
        self.marker_pub = manager.create_publisher(Marker, "/linkup_marker", 10)

        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 10)

        self.odom = None
        self.tolerance = 2.0 # wider tolerance is ok - finding linkup isn't an exact science
        self.logger = manager.get_logger()
        self.finding = False

        self.path_client = ActionClient(manager, ComputePathToPose, "compute_path_to_pose")
        self.costmap_client = manager.create_client(GetCostmap, "global_costmap/get_costmap")

        self.manager = manager

    def odom_cb(self, pose: PoseStamped):
        self.odom = point_from_pose_2d(pose)

    def periodic(self) -> None | Events:
        self.publish_everything()
        if self.linkup_found:
            return Events.SUCCESS

        if self.odom is not None and np.linalg.norm(self.odom - self.goal_vec) <= self.tolerance and not self.finding:
            self.finding = True
            self.find_linkup() # find the linkup thingamabob

        return None
    
    def exit(self):
        pass


    # helper functions ===========================================================================

    def find_linkup(self):
        # print("finding linkup")
        self.goal.header.stamp = self.manager.get_clock().now().to_msg()

        self.costmap_client.wait_for_service()
        self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(self.costmap_cb)

        # self.find_segment(self.start_pose)

    def costmap_cb(self, future: Future):
        result = future.result()
        if result == None:
            return
        costmap: Costmap = result.map

        padding = 0.4
        num_points = 20
        num_iterations = 100
        length = self.excavation_edge.length - padding * 2
        offset = length / num_points
        points = (self.excavation_edge.interpolate(offset * i + padding) for i in range(0, num_points))
        iterated_points = (self.iterate_point(costmap, point, 0, num_iterations, id) for id, point in enumerate(points))

        pos, angle = min(iterated_points, key=lambda alt: self.evaluate_point(costmap, alt[0], alt[1]))
        self.show_line(pos, angle, num_points, "final", 1.0, 1.0, 1.0, 0.1)

        linkup = Linkup()
        linkup.main_target.x = pos.x + np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2
        linkup.main_target.y = pos.y + np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2
        linkup.mini_target.x = pos.x - np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2
        linkup.mini_target.y = pos.y - np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2

        self.linkup_pub.publish(linkup)
        self.linkup_found = True

    def evaluate_point(self, costmap: Costmap, pos: shp.Point, angle: float):
        if pos.distance(self.excavation_edge) > self.MIN_SEGMENT_LENGTH / 4:
            return inf

        a = shp.Point(pos.x + np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2, pos.y + np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2)
        b = shp.Point(pos.x - np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2, pos.y - np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2)

        return line_cost(costmap, a, b)

    def iterate_point_once(self, costmap: Costmap, pos: shp.Point, angle: float):
        alternatives = [(shp.Point(pos.x + costmap.metadata.resolution, pos.y), angle),
                        (shp.Point(pos.x - costmap.metadata.resolution, pos.y), angle),
                        (shp.Point(pos.x, pos.y + costmap.metadata.resolution), angle),
                        (shp.Point(pos.x, pos.y - costmap.metadata.resolution), angle),
                        (shp.Point(pos.x, pos.y), angle + 0.1),
                        (shp.Point(pos.x, pos.y), angle - 0.1),
                        (shp.Point(pos.x, pos.y), angle)]

        min = None
        min_alts = []

        for pos, angle in alternatives:
            cost = self.evaluate_point(costmap, pos, angle)
            if min == None or cost < min:
                min = cost
                min_alts = [(pos, angle)]
            elif min == cost:
                min_alts.append((pos, angle))

        return random.choice(min_alts)

    def iterate_point(self, costmap: Costmap, pos: shp.Point, angle: float, num_iterations: int, id):
        self.show_line(pos, angle, id, "intermediate")
        for _ in range(0, num_iterations):
            pos, angle = self.iterate_point_once(costmap, pos, angle)
            cost = self.evaluate_point(costmap, pos, angle)/5.0
            self.show_line(pos, angle, id, "intermediate", action=Marker.MODIFY, r=0.0 if cost > 1.0 else cost, g=1.0 if not math.isinf(cost) and cost > 1.0 else 0.0, b=1.0 if math.isinf(cost) else 0.0)

        return pos, angle

    # VISUALIZATION HELPERS ==========================================================================

    def show_line(self, pos: shp.Point, angle: float, id, ns, r=1.0, g=0.0, b=0.0, width=0.05, action=Marker.ADD):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.manager.get_clock().now().to_msg()

        marker.ns = ns
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = action

        # Line width
        marker.scale.x = width

        # Color (RGBA)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Identity pose
        marker.pose.orientation.w = 1.0

        a = Point(x = pos.x + np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2, y = pos.y + np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2)
        b = Point(x = pos.x - np.cos(angle) * self.MIN_SEGMENT_LENGTH / 2, y = pos.y - np.sin(angle) * self.MIN_SEGMENT_LENGTH / 2)

        marker.points = [a, b]

        self.marker_pub.publish(marker)

def main():
    rclpy.init(args=sys.argv)

    node = Node("linkup_test_node")

    find_linkup = FindLinkup()
    find_linkup.setup(node)
    find_linkup.find_linkup()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
