#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GridNode:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent


class PathGenerator:
    def __init__(self, x, y, width, height, cell_size, filled_prob):
        self.width = int(width/cell_size)
        self.height = int(height/cell_size)
        self.cell_size = cell_size
        self.filled_prob = filled_prob
        coords = self.__get_grid_location_from_coordinates(x, y)
        self.x = coords[1]
        self.y = coords[0]
        self.odometry = Odometry()
        self.grid = [[0] * self.width] * self.height

        rospy.Subscriber(
            "/map", OccupancyGrid, self.__generate_path
        )

        rospy.Subscriber("/odometry/filtered", Odometry, self.__update_position)

        self.publisher = rospy.Publisher("/path", Path,queue_size=10)

        rospy.spin()

    def __generate_path(self, occupancy_grid):
        # Generating Grid
        row = 0
        col = 0
        for val in occupancy_grid.data:
            if val > self.filled_prob:
                self.grid[row][col] = 1
            else:
                self.grid[row][col] = 0
            col += 1
            if col == self.width:
                col = 0
                row += 1

        # Getting Starting Location
        pos_x = self.odometry.pose.pose.position.x
        pos_y = self.odometry.pose.pose.position.y
        start_pos = self.__get_grid_location_from_coordinates(pos_x, pos_y)

        # A Star Setup
        open_list = []
        closed_list = []
        current_node = GridNode(start_pos[0], start_pos[1])
        current_node.g = 0
        current_node.h = (self.x - current_node.x) * (self.x - current_node.x) + (
            self.y - current_node.y
        ) * (self.y - current_node.y)
        current_node.f = current_node.g + current_node.h
        open_list.append(current_node)

        # A Star Loop
        while len(open_list) > 0:

            # Finding node w/ smallest F
            current_node = open_list[0]
            current_idx = 0
            for i in range(1, len(open_list)):
                node = open_list[i]
                if node.f < current_node.f:
                    current_node = node
                    current_idx = i
            

            open_list.pop(current_idx)
            closed_list.append(current_node)

            # Stopping if end is found
            if current_node.x == self.x and current_node.y == self.y:
                break

            # Going Through all move options
            move_options = [[1, 0], [-1, 0], [0, 1], [0, -1]]
            for move_option in move_options:
                node_x = current_node.x + move_option[0]
                node_y = current_node.y + move_option[1]

                # Stopping if position out of bounds or space is occupied
                if (
                    node_x < 0
                    or node_x >= self.width
                    or node_y < 0
                    or node_y >= self.height
                    or self.grid[node_x][node_y] == 1
                ):
                    continue

                # Stopping if position was already visited
                visited = False
                for visited_node in closed_list:
                    if node_x == visited_node.x and node_y == visited_node.y:
                        visited = True
                        break
                if visited:
                    continue

                # Changing Node if already in Open List
                checking_visited = False
                for to_visit_node in open_list:
                    if to_visit_node.x == node_x and to_visit_node.y == node_y:
                        checking_visited = True
                        g = np.abs(to_visit_node.x - start_pos[0]) + np.abs(
                            to_visit_node.y - start_pos[1]
                        )
                        h = (self.x - to_visit_node.x) * (self.x - to_visit_node.x) + (
                            self.y - to_visit_node.y
                        ) * (self.y - to_visit_node.y)
                        f = g + h
                        if f < to_visit_node.f:
                            to_visit_node.g = g
                            to_visit_node.h = h
                            to_visit_node.f = f
                            to_visit_node.parent = current_node
                        break

                # Adding new Node to open list if currently not there
                if not checking_visited:
                    new_node = GridNode(node_x, node_y, parent=current_node)
                    new_node.g = np.abs(new_node.x - start_pos[0]) + np.abs(
                        new_node.y - start_pos[1]
                    )
                    new_node.h = (self.x - new_node.x) * (self.x - new_node.x) + (
                        self.y - new_node.y
                    ) * (self.y - new_node.y)
                    new_node.f = new_node.g + new_node.h
                    open_list.append(new_node)
        print('here')
        # Creating and Publishing Path
        poses = []
        if len(closed_list) > 0:
            print('here 147')
            while current_node != None:
                pose = PoseStamped()
                pose.header.frame_id = "path"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = (
                    current_node.x * self.cell_size
                )  # TODO convert to real life coordinates, not grid coordinates
                pose.pose.position.y = current_node.y * self.cell_size
                poses.append(pose)
                current_node = current_node.parent
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "base_link"
        path.poses = poses
        self.publisher.publish(path)

    def __update_position(self, odometry):
        self.odometry = odometry

    def __get_grid_location_from_coordinates(
        self, x_coordinate, y_coordinate, x_0=-100, y_0=-100
    ):
        x = int((x_coordinate - x_0) / self.cell_size)
        y = int((y_coordinate - y_0) / self.cell_size)
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return [-1, -1]
        return [x, y]
