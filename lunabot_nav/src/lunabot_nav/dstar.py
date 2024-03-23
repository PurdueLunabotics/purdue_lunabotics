import rospy

from queue import PriorityQueue, Queue
from sys import maxsize 

import numpy as np


class Dstar:
    """
    Initializes the Dstar algorithm by creating a priority queue, initializing accumulation (km),
    creating a list of node values (g and rhs), all initialized to infinity, setting the rhs of the goal node to 0, and inserting
    the goal node into the priority queue

    http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
    """

    def __init__(self, goal, start, init_map, radius, resolution, x_offset, y_offset):
        self.node_queue: PriorityQueue = PriorityQueue()
        self.km: float = 0.0

        self.OCCUPANCY_THRESHOLD = 50

        self.root2 = np.sqrt(2) # faster than computing sqrt 2 over and over

        self.node_values_list: np.ndarray[float] = maxsize * np.ones((init_map.shape[0], init_map.shape[1], 2))  # 2d Array of nodes: each value is [Distance (g), Estimate (rhs)]

        self.x_offset: float = x_offset
        self.y_offset: float = y_offset

        self.current_map: np.ndarray[int] = init_map # 2d array occupancy map: Occupancy probabilities from [0 to 100].  Unknown is -1.

        self.resolution: float = resolution # meters per grid cell

        self.goal: list[int] = self.convert_to_grid(goal) # Convert the goal to grid coordinates

        self.node_values_list[self.goal[0], self.goal[1], 1] = 0 # Set the estimate (rhs) value of the goal to 0

        start = self.convert_to_grid(start) # Convert the start to grid coordinates

        self.current_node: list[int] = start
        self.prev_node: list[int] = start

        self.radius: int = radius # radius of the robot (grid units)

        self.needs_new_path: bool = True  # updates when map changes to know when to create new path

        # Insert the goal node into the priority queue
        self.insert(self.goal, self.calculate_key(self.goal))

    def update_position(self, coords):
        """
        When receiving a new position from the node, change it to grid coords and update the current node
        """

        position = self.convert_to_grid(coords)
        self.current_node = position

    def convert_to_grid(self, position: 'list[float]') -> 'list[int]':
        """
        Convert a real world (x y) position to grid coordinates. Grid offset should be in the same frame as position, the grid is row-major.
        """

        shifted_pos = [position[0] - self.x_offset, position[1] - self.y_offset]
        coord = [
            int(shifted_pos[1] / self.resolution + 0.5),
            int(shifted_pos[0] / self.resolution + 0.5),
        ]
        return coord

    def convert_to_real(self, coord: 'list[int]') -> 'list[float]':
        """
        Convert a grid position to real world (x y) coordinates. Grid offset should be in the same frame as position, the grid is row-major.
        """

        position = [(coord[1] + 0.5) * self.resolution, (coord[0] + 0.5) * self.resolution]
        position = [position[0] + self.x_offset, position[1] + self.y_offset]
        return position

    # returns the lowest priority in the queue
    def get_top_key(self):
        if self.node_queue.empty():
            return (maxsize, maxsize)
        else:
            return self.node_queue.queue[0][0]

    # inserts a node into the queue
    def insert(self, node: list, key: tuple):
        self.node_queue.put((key, node))

    # removes a given node from the queue
    # TODO- this probably is not efficient
    def remove(self, node: list):
        nodelist = self.node_queue.queue
        newlist = nodelist.copy()

        for node_tuple in nodelist:
            if node_tuple[1] == node:
                newlist.remove(node_tuple)
                break

        self.node_queue = PriorityQueue()

        for node_tuple in newlist:
            self.node_queue.put(node_tuple)

    def hueristic(self, node: list) -> float:
        """
        Calculates the hueristic used for the priority of a node based on its distance to the goal
        """
        return np.sqrt((node[0] - self.current_node[0]) ** 2
                       + (node[1] - self.current_node[1]) ** 2)

    def bfs_non_occupied(self, current_node: list) -> 'list[int]':
        """
        Searches for the nearest non-occupied node closest to the given node. Used for finding a path if the robot is stuck on top of an obstacle.
        """

        nodequeue = Queue()
        visited_nodes = set()

        nodequeue.put(current_node)

        while not nodequeue.empty():

            node = nodequeue.get()

            if tuple(node) in visited_nodes:
                continue

            if self.current_map[node[0]][node[1]] < self.OCCUPANCY_THRESHOLD:
                return node

            directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]  # above, below, left, right

            for direction in directions:
                new_node = [node[0] + direction[0], node[1] + direction[1]]
                # Add the node if in bounds
                if 0 <= new_node[0] < len(self.node_values_list) and 0 <= new_node[1] < len(self.node_values_list[0]):
                    nodequeue.put(new_node)

            visited_nodes.add(tuple(node))

        rospy.loginfo("Dstar: Error in search for non-occupied node")
        return [current_node]

    def calculate_key(self, node: list):
        """
        Calculates the priority of a node based on its g and rhs values
        """
        g_value = self.node_values_list[node[0]][node[1]][0]
        rhs_value = self.node_values_list[node[0]][node[1]][1]
        min_val = min(g_value, rhs_value)

        heuristic = self.hueristic(node)

        key1 = min_val + heuristic + self.km
        key2 = min_val

        return (key1, key2)

    def calculate_RHS(self, node: list):
        """
        Calculates the RHS (estimate value) of a given node by: first checking if it's an obstacle, then
        checking each surrounding node, calculating what the distance value should be based on those nodes,
        and taking the lowest value.
        """

        current_map = self.current_map  # copy map avoid async problems
        node_values_list = self.node_values_list

        if current_map[node[0]][node[1]] > self.OCCUPANCY_THRESHOLD or current_map[node[0]][node[1]] == 2: # Check for obstacle
            return maxsize

        surrounding_values = []  # a list of distance values (floats)

        # For each node (all 4 directions)

        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
        for direction in directions:
            new_node = [node[0] + direction[0], node[1] + direction[1]]
            # If the node is in bounds, and not an obstacle, add the distance value to the list
            if (
                0 <= new_node[0] < len(current_map) and 0 <= new_node[1] < len(current_map[0])
                and (current_map[new_node[0]][new_node[1]] <= self.OCCUPANCY_THRESHOLD or current_map[new_node[0]][new_node[1]] == 2)
            ):
                g_val = node_values_list[new_node[0]][new_node[1]][0]
                surrounding_values.append(g_val + self.root2 if direction[0] != 0 and direction[1] != 0 else g_val + 1)

                
        return min(surrounding_values)

    def update_node(self, node: list):
        """
        Updates a node's values by calculating its RHS (estimate value). It removes the node from the queue (based on old value) and
        replaces it on the queue if its values are locally inconsistent (if g != RHS)
        """

        node_values_list = self.node_values_list

        if node != self.goal:
            node_values_list[node[0]][node[1]][1] = self.calculate_RHS(node)  # Calculate RHS

        # Calculate if it's in the queue and remove
        queue_contains = False
        for node_tuple in self.node_queue.queue:
            if node_tuple[1] == node:
                queue_contains = True
                break

        if queue_contains:
            self.remove(node)

        g_val = self.node_values_list[node[0]][node[1]][0]
        rhs_val = self.node_values_list[node[0]][node[1]][1]

        # Place it on the queue if not consistent
        if g_val != rhs_val:
            self.insert(node, self.calculate_key(node))

    def find_path(self):
        """
        Find_Path calculates the path for DStar by looping until the current node is locally consistent (g value = rhs) and the priority of the current node is the lowest in the queue.
        It picks the lowest priority node, checks whether its priority is correct, then updates its g value (distance). If the g value is higher then the estimate, it lowers the g value to the estimate.
        If the g value is lower then the estimate, it sets it for correction by setting the g value to infinity. It then marks all surrounding nodes to be checked.

        Through this process, all nodes on the grid have their g value calculated correctly so the path can be found.
        """

        node_values_list = (
            self.node_values_list
        )  # copy values list to avoid async problems

        if self.current_map[self.current_node[0]][self.current_node[1]] > 50:
            self.current_node = self.bfs_non_occupied(self.current_node)

        while (
            self.get_top_key() < self.calculate_key(self.current_node)
            or node_values_list[self.current_node[0]][self.current_node[1]][0]
            != node_values_list[self.current_node[0]][self.current_node[1]][1]
        ):
            # Loop until current node is locally consistent and priority is lowest in the queue
            old_key = self.get_top_key()
            chosen_node = self.node_queue.get()[1]  # Chosen node to check

            # If the priority of the node was incorrect, add back to the queue with the correct priority.
            if old_key < self.calculate_key(chosen_node):
                self.insert(chosen_node, self.calculate_key(chosen_node))

            # If g value is greater then rhs
            elif (
                node_values_list[chosen_node[0]][chosen_node[1]][0]
                > node_values_list[chosen_node[0]][chosen_node[1]][1]
            ):
                node_values_list[chosen_node[0]][chosen_node[1]][0] = node_values_list[chosen_node[0]][chosen_node[1]][1]  # Lower the g value

                # update all surrounding nodes
                directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
                for direction in directions:
                    new_node = [chosen_node[0] + direction[0], chosen_node[1] + direction[1]]
                    if 0 <= new_node[0] < len(node_values_list) and 0 <= new_node[1] < len(node_values_list[0]):
                        self.update_node(new_node)

            # G is lower then rhs
            else:
                # Set g to infinity
                node_values_list[chosen_node[0]][chosen_node[1]][0] = maxsize

                # Update this node
                self.update_node(chosen_node)

                # update all the surrounding nodes
                directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
                for direction in directions:
                    new_node = [chosen_node[0] + direction[0], chosen_node[1] + direction[1]]
                    if 0 <= new_node[0] < len(node_values_list) and 0 <= new_node[1] < len(node_values_list[0]):
                        self.update_node(new_node)


            if self.node_queue.qsize() == 0:
                rospy.loginfo("Dstar: No path found (map processing failed)")
                break

    def create_path_list(self):
        """
        Create path: This creates a temporary node at the start, and picks the lowest g value (lowest distance to goal) as the next step on the path, adds it to the list, and
        repeats until the goal is reached. All of these points in order are the path.
        """

        node_values_list = self.node_values_list  # copy to avoid async problems

        # if start = goal or map/goal are obstacle
        if (
            self.current_node[0] == self.goal[0]
            and self.current_node[1] == self.goal[1]
            or self.current_map[self.current_node[0]][self.current_node[1]] > self.OCCUPANCY_THRESHOLD
        ):

            rospy.loginfo("Dstar: No path (start = goal or map/goal are obstacle)")
            return []

        path_node = self.current_node.copy()

        path_list = []

        while (
            path_node[0] != self.goal[0] or path_node[1] != self.goal[1]
        ):  # Until robot reaches self.goal

            if node_values_list[path_node[0]][path_node[1]][0] >= maxsize:  # If g value of current node is inf
                rospy.loginfo("Dstar: No path (couldn't create a complete path list)")
                break

            # Check all surrounding nodes for the lowest g value

            gvals = []  # find smallest g value (closest to self.goal)

            directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
            for direction in directions:
                new_x = path_node[0] + direction[0]
                new_y = path_node[1] + direction[1]
                if 0 <= new_x < len(node_values_list) and 0 <= new_y < len(node_values_list[0]):
                    if self.current_map[new_x][new_y] < self.OCCUPANCY_THRESHOLD:
                        heuristic = np.sqrt(
                            (self.goal[0] - new_x) ** 2 + (self.goal[1] - new_y) ** 2
                        )
                        gvals.append(
                            (
                                node_values_list[new_x][new_y][0] + 1,
                                heuristic,
                                [new_x, new_y],
                            )
                        )

            if len(gvals) == 0:  # Nowhere to go
                rospy.loginfo("Dstar: No path (couldn't create a complete path list)")
                return []

            min_val = min(gvals)  # pick lowest g value
            path_node = min_val[2]

            if path_node in path_list:  # Doubling back- no more path
                rospy.loginfo("Dstar: No path (path list incomplete (would double back))")
                return []

            path_list.append(path_node)
            gvals.clear()

        self.needs_new_path = False

        for i in range(len(path_list)):
            path_list[i] = self.convert_to_real(path_list[i])

        return path_list

    def update_replan(self, prev_map):
        """
        Update and replanning: Should trigger whenever there is a new map.
        Sets affected nodes to update, and calculates new g values (finds new path)
        This function runs when the map changes
        """

        self.km += ((self.prev_node[0] - self.current_node[0]) ** 2
                   + (self.prev_node[1] - self.current_node[1]) ** 2) ** 0.5
        
        # Add to the accumulation value the distance from the last point (of changed map) to the current point
        self.prev_node = self.current_node.copy()  # update the prev_node

        for i in range(len(prev_map)):
            for j in range(len(prev_map[i])):
                if (
                    self.current_map[i][j] != prev_map[i][j]
                ):  # for all differing values, update it and its surrounding nodes

                    self.update_node([i, j])

                    # The below block also updates every surrounding node, seemingly is not needed.

                    # if (i > 0): #above
                    #     self.update_node([i-1,j])
                    # if (i < len(self.node_values_list)-1): #below
                    #     self.update_node([i+1,j])
                    # if (j > 0): #left
                    #     self.update_node([i,j-1])
                    # if (j < len(self.node_values_list[0])-1): #right
                    #     self.update_node([i,j+1])


        self.find_path()  # recalculate g values

        self.needs_new_path = True

    def update_map(self, new_map, x_offset=0, y_offset=0):
        """    
        Updates the map with new grid whenever map is changed
        """

        prev_x_offset = self.x_offset
        prev_y_offset = self.y_offset

        # set prev_map
        prev_map = self.current_map.copy()

        new_map = np.array(new_map)

        if np.array_equal(prev_map, new_map):
            return

        new_node_values = self.node_values_list

        if (
            x_offset != prev_x_offset
        ):  # Adjust size for new offset (only grows negative)
            self.x_offset = x_offset

            xdiff = int((prev_x_offset - x_offset) / self.resolution)

            columns = maxsize * np.ones((len(self.node_values_list), xdiff, 2))
            new_node_values = np.concatenate((columns, new_node_values), axis=1)

        if y_offset != y_offset:
            self.y_offset = y_offset

            ydiff = int((prev_y_offset - y_offset) / self.resolution)

            rows = maxsize * np.ones((ydiff, len(self.node_values_list[0]), 2))
            new_node_values = np.concatenate((rows, new_node_values), axis=0)

        if len(new_map[0]) != len(new_node_values[0]):  # adjust x length (if map grows)
            xdiff = len(new_map[0]) - len(new_node_values[0])
            columns = maxsize * np.ones((len(new_node_values), xdiff, 2))
            new_node_values = np.concatenate((new_node_values, columns), axis=1)

        if len(new_map) != len(new_node_values):
            ydiff = len(new_map) - len(new_node_values)
            rows = maxsize * np.ones((ydiff, len(new_node_values[0]), 2))
            new_node_values = np.concatenate((new_node_values, rows), axis=0)

        self.node_values_list = new_node_values

        self.current_map = new_map

        self.update_replan(prev_map)
