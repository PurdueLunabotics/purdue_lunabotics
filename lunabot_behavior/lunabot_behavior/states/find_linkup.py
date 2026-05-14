#!/usr/bin/env python3

from math import inf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from lunabot_msgs.msg import Linkup
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from rtabmap_msgs.msg import OdomInfo

from lunabot_behavior import zones
from lunabot_behavior.state import Events, State
from lunabot_behavior.states.traverse import Traverse
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Duration
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

def map_to_world(costmap: Costmap, x: int, y: int) -> tuple[float, float]:
    resolution = costmap.metadata.resolution
    origin_pos = costmap.metadata.origin.position
    return (x * resolution + origin_pos.x, y * resolution + origin_pos.y)

def is_in_costmap(costmap: Costmap, x: int, y: int) -> bool:
    return x >= 0 and x < costmap.metadata.size_x and y >= 0 and y < costmap.metadata.size_y

def get_cost(costmap: Costmap, x: int, y: int) -> float:
    return 26.0 + 0.9 * int(costmap.data[x + y * costmap.metadata.size_x])

def is_blocked(costmap: Costmap, x: int, y: int) -> bool:
    return costmap.data[x + y * costmap.metadata.size_x] >= LETHAL_COST

def get_traversal_cost(costmap: Costmap, x: int, y: int):
  curr_cost = get_cost(costmap, x, y)
  return (curr_cost / LETHAL_COST) ** 2

def line_cost(costmap: Costmap, a: shp.Point, b: shp.Point) -> tuple[float, bool]:
    initial = world_to_map(costmap, a.x, a.y)
    end = world_to_map(costmap, b.x, b.y)
    current = initial
    distance_x = abs(initial[0] - end[0])
    distance_y = abs(initial[1] - end[1])

    if not is_in_costmap(costmap, initial[0], initial[1]) or not is_in_costmap(costmap, end[0], end[1]):
        return inf, True

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
    blocked = False

    if distance_x > distance_y:
        while current[0] != end[0]:
            current = (current[0] + x_increment, current[1])
            target_y = dy * (current[0] - initial[0]) / dx + initial[1];
            if int(target_y) != current[1]:
                current = (current[0], current[1] + y_increment)

            blocked = blocked or is_blocked(costmap, current[0], current[1])

            cost += get_traversal_cost(costmap, current[0], current[1])
    else:
        while current[1] != end[1]:
            current = (current[0], current[1] + y_increment)
            target_x = dx * (current[1] - initial[1]) / dy + initial[0];
            if int(target_x) != current[0]:
                current = (current[0] + x_increment, current[1])

            blocked = blocked or is_blocked(costmap, current[0], current[1])

            cost += get_traversal_cost(costmap, current[0], current[1])

    return cost, blocked

WAITING = 0
FINDING_LINKUP = 1
FOUND_LINKUP = 2
FAILED = 3

class TraverseToMiddle(Traverse):
    def __init__(self):
        self.goal = PoseStamped()
        self.goal.pose.position.x = ZoneMeasurements.BERM_OFFSET_X
        self.goal.pose.position.y = 0.0

        super().__init__(self.goal, False)

    def setup(self, manager):
        ns = manager.get_namespace().lstrip('/')
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"
        self.goal.header.frame_id = self.frame
        super().setup(manager)


class FindLinkup(State):
    def __init__(self):
        self.MIN_SEGMENT_LENGTH = 1.325

        # excavation edge for linkup
        self.is_mirrored = ZoneMeasurements.BERM_OFFSET_X > ZoneMeasurements.EXC_OFFSET_X
        if not self.is_mirrored:
            self.exc_p1 = [ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                    ZoneMeasurements.EXC_OFFSET_Y + (ZoneMeasurements.EXC_LENGTH_Y / 2)]
            self.exc_p2 = [ZoneMeasurements.EXC_OFFSET_X - (ZoneMeasurements.EXC_LENGTH_X / 2),
                                    ZoneMeasurements.EXC_OFFSET_Y - (ZoneMeasurements.EXC_LENGTH_Y / 2)]
        else:
            self.exc_p1 = [ZoneMeasurements.EXC_OFFSET_X + (ZoneMeasurements.EXC_LENGTH_X / 2),
                                    ZoneMeasurements.EXC_OFFSET_Y + (ZoneMeasurements.EXC_LENGTH_Y / 2)]
            self.exc_p2 = [ZoneMeasurements.EXC_OFFSET_X + (ZoneMeasurements.EXC_LENGTH_X / 2),
                                    ZoneMeasurements.EXC_OFFSET_Y - (ZoneMeasurements.EXC_LENGTH_Y / 2)]
        
        self.excavation_edge = LineString([self.exc_p1, self.exc_p2])
        self.padding = 0.4

    def setup(self, manager: Node):
        ns = manager.get_namespace().lstrip('/')
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"

        super().setup(manager)

        self.linkup_pub = manager.create_publisher(Linkup, "/linkup_pos", QoSProfile(durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, depth = 10))
        self.marker_pub = manager.create_publisher(Marker, "/linkup_marker", 10)
        self.marker_pub.publish(Marker(action=Marker.DELETEALL, header=Header(frame_id=self.frame)))

        self.logger = manager.get_logger()

        self.costmap_client = manager.create_client(GetCostmap, "global_costmap/get_costmap")

        self.manager = manager

    def start(self):
        self.state = WAITING
        self.find_linkup()

    def periodic(self) -> None | Events:
        if self.state == FOUND_LINKUP:
            return Events.SUCCESS
        elif self.state == FAILED:
            return Events.FAIL
        return None


    # helper functions ===========================================================================

    def line_to_ends(self, pos: shp.Point, angle: float, length: float | None = None) -> tuple[shp.Point, shp.Point]:
        if length is None:
            length = self.MIN_SEGMENT_LENGTH

        if not self.is_mirrored:
            a = shp.Point(pos.x + np.cos(angle) * length / 2, pos.y + np.sin(angle) * length / 2)
            b = shp.Point(pos.x - np.cos(angle) * length / 2, pos.y - np.sin(angle) * length / 2)
        else:
            a = shp.Point(pos.x - np.cos(angle) * length / 2, pos.y + np.sin(angle) * length / 2)
            b = shp.Point(pos.x + np.cos(angle) * length / 2, pos.y - np.sin(angle) * length / 2)

        return (a, b)

    def find_linkup(self):
        self.costmap_client.wait_for_service()
        self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(self.costmap_cb)

    def costmap_cb(self, future: Future):
        result = future.result()
        if result == None:
            return
        costmap: Costmap = result.map

        num_points = 20
        num_iterations = 100
        length = self.excavation_edge.length - self.padding * 2
        offset = length / num_points
        points = (self.excavation_edge.interpolate(offset * i + self.padding) for i in range(0, num_points))
        iterated_points = (self.iterate_point(costmap, point, 0, num_iterations, id) for id, point in enumerate(points))

        pos, angle = min(iterated_points, key=lambda alt: self.evaluate_point(costmap, alt[0], alt[1]))
        self.show_line(pos, angle, num_points, "final", 1.0, 1.0, 1.0, 0.1)

        linkup = Linkup()
        a, b = self.line_to_ends(pos, angle)
        linkup.main_target.x = a.x
        linkup.main_target.y = a.y
        linkup.mini_target.x = b.x
        linkup.mini_target.y = b.y

        linkup.exc_target.x = linkup.main_target.x
        linkup.exc_target.y = linkup.main_target.y

        for i in range(10):
            self.linkup_pub.publish(linkup)
        self.state = FOUND_LINKUP

    def evaluate_point(self, costmap: Costmap, pos: shp.Point, angle: float) -> tuple[float, bool]:
        if pos.distance(self.excavation_edge) > self.MIN_SEGMENT_LENGTH / 4 or pos.y > self.exc_p1[1] - self.padding or pos.y < self.exc_p2[1] + self.padding:
            return (inf, True)

        eval_length = self.MIN_SEGMENT_LENGTH * 2.0

        a, b = self.line_to_ends(pos, angle, eval_length)

        if not a.within(zones.zone_to_poly(zones.exc_zone)):
            return (inf, True)

        cost, blocked = line_cost(costmap, a, b)

        return cost + np.abs(angle) * 5, blocked

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
            cost = self.evaluate_point(costmap, pos, angle)[0]/5.0
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

        start, end = self.line_to_ends(pos, angle, self.MIN_SEGMENT_LENGTH)

        marker.points = [Point(x = start.x, y = start.y), Point(x = end.x, y = end.y)]

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
