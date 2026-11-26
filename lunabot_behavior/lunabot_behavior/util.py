#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.msg import Costmap

import numpy as np
import shapely.geometry as shp

def point_from_pose_2d(pose: PoseStamped | Pose):
    """
    Converts a Pose or PoseStamped into a 2D np point.
    """
    if isinstance(pose, Pose):
        return np.array([pose.position.x, pose.position.y])
    else:
        return np.array([pose.pose.position.x, pose.pose.position.y])
    
# =======================
# Costmap processing
# =======================

class CostmapUtil:
    @staticmethod
    def world_to_map(costmap: Costmap, x: float, y: float):
        resolution = costmap.metadata.resolution
        origin_pos = costmap.metadata.origin.position
        return (int((x - origin_pos.x) / resolution), int((y - origin_pos.y) / resolution))

    @staticmethod
    def map_to_world(costmap: Costmap, x: int, y: int) -> tuple[float, float]:
        resolution = costmap.metadata.resolution
        origin_pos = costmap.metadata.origin.position
        return (x * resolution + origin_pos.x, y * resolution + origin_pos.y)

    @staticmethod
    def is_in_costmap(costmap: Costmap, x: int, y: int) -> bool:
        return x >= 0 and x < costmap.metadata.size_x and y >= 0 and y < costmap.metadata.size_y

    @staticmethod
    def get_cost(costmap: Costmap, x: int, y: int) -> float:
        return 26.0 + 0.9 * int(costmap.data[x + y * costmap.metadata.size_x])

    @staticmethod
    def is_blocked(costmap: Costmap, x: int, y: int, lethal_cost: float) -> bool:
        return costmap.data[x + y * costmap.metadata.size_x] >= lethal_cost

    @staticmethod
    def get_traversal_cost(costmap: Costmap, x: int, y: int, lethal_cost: float):
        curr_cost = CostmapUtil.get_cost(costmap, x, y)
        return (curr_cost / lethal_cost) ** 2

    @staticmethod
    def line_cost(costmap: Costmap, a: shp.Point, b: shp.Point, lethal_cost: float) -> tuple[float, bool]:
        """
        Returns cost and whether the path is over lethal cost.

        :return: cost, whether the cost is greater than lethal
        """
        initial = CostmapUtil.world_to_map(costmap, a.x, a.y)
        end = CostmapUtil.world_to_map(costmap, b.x, b.y)
        current = initial
        distance_x = abs(initial[0] - end[0])
        distance_y = abs(initial[1] - end[1])

        if not CostmapUtil.is_in_costmap(costmap, initial[0], initial[1]) or not CostmapUtil.is_in_costmap(costmap, end[0], end[1]):
            return np.inf, True

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

                blocked = blocked or CostmapUtil.is_blocked(costmap, current[0], current[1], lethal_cost)

                cost += CostmapUtil.get_traversal_cost(costmap, current[0], current[1], lethal_cost)
        else:
            while current[1] != end[1]:
                current = (current[0], current[1] + y_increment)
                target_x = dx * (current[1] - initial[1]) / dy + initial[0];
                if int(target_x) != current[0]:
                    current = (current[0] + x_increment, current[1])

                blocked = blocked or CostmapUtil.is_blocked(costmap, current[0], current[1], lethal_cost)

                cost += CostmapUtil.get_traversal_cost(costmap, current[0], current[1], lethal_cost)

        return cost, blocked