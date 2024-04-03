import rospy

from queue import PriorityQueue, Queue
from sys import maxsize 

import numpy as np


class Dstar:
    """
    A class that implements the dstar lite path planning algorithm
    http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

    Finds the shortest path between the current position and goal position. Allows 'quick' replanning
    when the map changes by saving the processed data and updating only what is necessary.
    """

    def __init__(self, goal: 'list[float]', start: 'list[float]', init_map: np.ndarray, resolution: float, x_offset: float, y_offset: float):
        """
        Initializes the Dstar algorithm by setting up the map, node values, start, and the goal. The map/node values will be buffered to include the goal if necessary.
        Creates the priority queue, adds the first node to check, and marks the node's estimate value as 0.

        The goal/start should be given in real-world coordinates.
        The map, resolution, and offsets should come from the occupancy map.
        """

        self.node_queue: PriorityQueue = PriorityQueue()
        self.km: float = 0.0

        # What number on the map corresponds to occupied
        self.OCCUPANCY_THRESHOLD = 50  

         # faster than computing sqrt 2 over and over
        self.root2 = np.sqrt(2)

        # 2d Array of nodes that corresponds to the map: each value is [Distance (g), Estimate (rhs)]
        self.node_values_list: np.ndarray[float] = maxsize * np.ones((init_map.shape[0], init_map.shape[1], 2))  

        # Real-world location of the 0,0 in the grid (not including buffer in the map)
        self.x_offset: float = x_offset 
        self.y_offset: float = y_offset

        # The amount of buffer on any side of the map
        self.buffer_offset_left: int = 0
        self.buffer_offset_up: int = 0
        self.buffer_offset_right: int = 0
        self.buffer_offset_down: int = 0

        # 2d array occupancy map: Occupancy probabilities from [0 to 100].  Unknown is -1.
        self.current_map: np.ndarray[int] = init_map

        self.resolution: float = resolution # meters per grid cell

        # Save the real-world position of the goal
        self.real_goal = goal 

        self.goal: list[int] = self.convert_to_grid(goal) # Convert the goal to grid coordinates initially
        self.buffer_map_for_goal()                        # Add a buffer to the map so we can plan to the goal when it is outside the map
        self.goal = self.convert_to_grid(goal)            # Reconvert given the new buffer

        self.node_values_list[self.goal[0], self.goal[1], 1] = 0 # Set the estimate (rhs) value of the goal to 0

        start = self.convert_to_grid(start) # Convert the start to grid coordinates

        self.current_node: list[int] = start
        self.prev_node: list[int] = start

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
            int(shifted_pos[1] / self.resolution + 0.5) + self.buffer_offset_up,
            int(shifted_pos[0] / self.resolution + 0.5) + self.buffer_offset_left,
        ]

        return coord

    def convert_to_real(self, coord: 'list[int]') -> 'list[float]':
        """
        Convert a grid position to real world (x y) coordinates. Grid offset should be in the same frame as position, the grid is row-major.
        """

        position = [(coord[1] + 0.5 - self.buffer_offset_left) * self.resolution, (coord[0] + 0.5 - self.buffer_offset_up) * self.resolution]
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

            if  (0 <= node[0] < len(self.current_map) and 0 <= node[1] < len(self.current_map[0]) and
                self.current_map[node[0]][node[1]] < self.OCCUPANCY_THRESHOLD):
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

        current_map = self.current_map  
        node_values_list = self.node_values_list

        if current_map[node[0]][node[1]] > self.OCCUPANCY_THRESHOLD: # Check for obstacle
            return maxsize

        surrounding_values = []  # a list of distance values (floats)

        # For each node (all 4 directions)

        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
        for direction in directions:
            new_node = [node[0] + direction[0], node[1] + direction[1]]
            # If the node is in bounds, and not an obstacle, add the distance value to the list
            if (0 <= new_node[0] < len(current_map) and 0 <= new_node[1] < len(current_map[0])):

                if (current_map[new_node[0]][new_node[1]] > self.OCCUPANCY_THRESHOLD):
                    surrounding_values.append(maxsize)
                else:
                    g_val = node_values_list[new_node[0]][new_node[1]][0]
                    surrounding_values.append(g_val + self.root2 if (direction[0] != 0 and direction[1] != 0) else g_val + 1)

                
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

    def find_path(self) -> 'list[list[float]]':
        """
        Find_Path calculates the path for DStar by looping until the current node is locally consistent (g value = rhs) and the priority of the current node is the lowest in the queue.
        It picks the lowest priority node, checks whether its priority is correct, then updates its g value (distance). If the g value is higher then the estimate, it lowers the g value to the estimate.
        If the g value is lower then the estimate, it sets it for correction by setting the g value to infinity. It then marks all surrounding nodes to be checked.

        Through this process, all nodes on the grid have their g value calculated correctly so the path can be found.

        Once finished, returns the calculated path from the start to the goal.
        """

        rospy.loginfo("Dstar: Finding path")

        # If the start is out of the map, or is an obstacle, search for the closest non-occupied node
        if (self.current_node[0] < 0 or self.current_node[0] >= len(self.current_map) or 
            self.current_node[1] < 0 or self.current_node[1] >= len(self.current_map[0]) or
            self.current_map[self.current_node[0]][self.current_node[1]] > self.OCCUPANCY_THRESHOLD):

            self.current_node = self.bfs_non_occupied(self.current_node)


        # Loop until current node (start) is locally consistent (g == rhs) and its priority is lowest in the queue
        while (
            self.get_top_key() < self.calculate_key(self.current_node)
            or self.node_values_list[self.current_node[0]][self.current_node[1]][0]
            != self.node_values_list[self.current_node[0]][self.current_node[1]][1]
        ):

            old_key = self.get_top_key()
            chosen_node = self.node_queue.get()[1]  # Chosen node to check

            # If the priority of the node was incorrect, add back to the queue with the correct priority.
            if old_key < self.calculate_key(chosen_node):
                self.insert(chosen_node, self.calculate_key(chosen_node))

            # If g value is greater then rhs
            elif (
                self.node_values_list[chosen_node[0]][chosen_node[1]][0]
                > self.node_values_list[chosen_node[0]][chosen_node[1]][1]
            ):
                # Lower the g value (the estimate is the more recent data)
                self.node_values_list[chosen_node[0]][chosen_node[1]][0] = self.node_values_list[chosen_node[0]][chosen_node[1]][1]

                # update all surrounding nodes
                directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
                for direction in directions:
                    new_node = [chosen_node[0] + direction[0], chosen_node[1] + direction[1]]
                    if 0 <= new_node[0] < len(self.node_values_list) and 0 <= new_node[1] < len(self.node_values_list[0]):
                        self.update_node(new_node)

            # G is lower then rhs
            else:
                # Set g to infinity (mark it for replanning)
                self.node_values_list[chosen_node[0]][chosen_node[1]][0] = maxsize

                # Update this node
                self.update_node(chosen_node)

                # update all the surrounding nodes
                directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
                for direction in directions:
                    new_node = [chosen_node[0] + direction[0], chosen_node[1] + direction[1]]
                    if 0 <= new_node[0] < len(self.node_values_list) and 0 <= new_node[1] < len(self.node_values_list[0]):
                        self.update_node(new_node)


            if self.node_queue.qsize() == 0:
                rospy.loginfo("Dstar: No path found (map processing failed)")
                break


        return self.create_path_list()

    def create_path_list(self) -> 'list[list[float]]':
        """
        Create path: This creates a temporary node at the start, and picks the lowest g value (lowest distance to goal) as the next step on the path, adds it to the list, and
        repeats until the goal is reached. All of these points in order are the path.
        """

        rospy.loginfo("Dstar: Generating path")

        node_values_list = self.node_values_list 

        # if start = goal, there is no path
        if (
            self.current_node[0] == self.goal[0]
            and self.current_node[1] == self.goal[1]
        ):

            rospy.loginfo("Dstar: No path (start = goal)")
            self.needs_new_path = False
            return []
        


        path_node = self.current_node.copy()

        path_list = []

        # Until robot reaches self.goal
        while (path_node[0] != self.goal[0] or path_node[1] != self.goal[1]):

            # Check all surrounding nodes for the lowest g value

            gvals = [] 

            directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # left, right, above, below
            for direction in directions:
                new_y = path_node[0] + direction[0]
                new_x = path_node[1] + direction[1]

                # if in bounds, and not an obstacle, add the g value to the list
                if 0 <= new_y < len(node_values_list) and 0 <= new_x < len(node_values_list[0]):
                    if self.current_map[new_y][new_x] < self.OCCUPANCY_THRESHOLD:

                        heuristic = np.sqrt((self.goal[0] - new_y) ** 2 + (self.goal[1] - new_x) ** 2)

                        gvals.append(
                            (
                                # We sort the path to take by two values- first the g value, then the euclidean distance to the goal.
                                # This should pick, in event of a tie, the closer node to the goal.

                                # If we encounter the goal in the surrounding nodes, make its priority the smallest
                                (node_values_list[new_y][new_x][0] + 1) if (self.goal[0] != new_y or self.goal[1] != new_x) else (-1),

                                heuristic,
                                
                                [new_y, new_x],
                            )
                        )

            if len(gvals) == 0:  # Nowhere to go
                rospy.loginfo("Dstar: No path (couldn't create a complete path list)")

                self.needs_new_path = False
                return []

            min_val = min(gvals)  # pick lowest g value
            path_node = min_val[2]

            if path_node in path_list:  # Doubling back- no more path
                rospy.loginfo("Dstar: No path (path list incomplete (would double back))")

                self.needs_new_path = False
                return []

            path_list.append(path_node)
            gvals.clear()

        # Convert the path to real-world coordinates
        for i in range(len(path_list)):
            path_list[i] = self.convert_to_real(path_list[i])

        return path_list

    def update_replan(self, prev_map: np.ndarray, left_offset: int, up_offset: int):
        """
        Update and replanning: Should trigger whenever there is a new map.
        Sets affected nodes to update, and calculates new g values (finds new path)
        This function runs when the map changes.

        Left offset and up offset are the amount of buffer added to the left and up sides of the map
        and are used to compare the correct parts of the map.

        Finally, it calculates the new path and returns the result
        """

        rospy.loginfo("Dstar: Updating values for new map")

        # Add to the accumulation value the distance from the last point (of changed map) to the current point
        self.km += ((self.prev_node[0] - self.current_node[0]) ** 2
                   + (self.prev_node[1] - self.current_node[1]) ** 2) ** 0.5
        
        
        self.prev_node = self.current_node.copy()  # update the prev_node

        for i in range(len(prev_map)):
            for j in range(len(prev_map[i])):
                # for all differing values, update it
                if (self.current_map[i + up_offset][j + left_offset] != prev_map[i][j]):

                    self.update_node([i + up_offset, j + left_offset])

                    # The below block can update every surrounding node, seemingly is not needed.

                    # if (i > 0): #above
                    #     self.update_node([i-1,j])
                    # if (i < len(self.node_values_list)-1): #below
                    #     self.update_node([i+1,j])
                    # if (j > 0): #left
                    #     self.update_node([i,j-1])
                    # if (j < len(self.node_values_list[0])-1): #right
                    #     self.update_node([i,j+1])

        # After all done updating, calculate the new path
        return self.find_path()

    def buffer_map_for_goal(self):
        """
        Add a buffer to the map so we can plan to the goal when it is outside the map.
        Saves the amount of buffer to allow for correct translations between real-world coords and grid coords.
        """

        EXTRA_BUFFER = 3  # extra buffer to add to the map, needed because the real-world coordinates can be slightly off

        if self.goal[0] < 0: # expand map up
            numRows = abs(self.goal[0]) + EXTRA_BUFFER

            mapRows = -1 * np.ones((numRows, len(self.current_map[0])))
            nodeValueRows = maxsize * np.ones((numRows, len(self.node_values_list[0]), 2))

            self.buffer_offset_up = numRows

            self.current_map = np.concatenate((mapRows, self.current_map), axis=0)
            self.node_values_list = np.concatenate((nodeValueRows, self.node_values_list), axis=0)

        if self.goal[0] >= len(self.current_map): # expand map down
            numRows = self.goal[0] - len(self.current_map) + 1 + EXTRA_BUFFER

            mapRows = -1 * np.ones((numRows, len(self.current_map[0])))
            nodeValueRows = maxsize * np.ones((numRows, len(self.node_values_list[0]), 2))

            self.buffer_offset_down = numRows

            self.current_map = np.concatenate((self.current_map, mapRows), axis=0)
            self.node_values_list = np.concatenate((self.node_values_list, nodeValueRows), axis=0)

        if self.goal[1] < 0: # expand map left
            numCols = abs(self.goal[1]) + EXTRA_BUFFER

            mapCols = -1 * np.ones((len(self.current_map), numCols))
            nodeValueCols = maxsize * np.ones((len(self.node_values_list), numCols, 2))

            self.buffer_offset_left = numCols

            self.current_map = np.concatenate((mapCols, self.current_map), axis=1)
            self.node_values_list = np.concatenate((nodeValueCols, self.node_values_list), axis=1)

        if self.goal[1] >= len(self.current_map[0]): # expand map right

            numCols = self.goal[1] - len(self.current_map[0]) + 1 + EXTRA_BUFFER

            mapCols = -1 * np.ones((len(self.current_map), numCols))
            nodeValueCols = maxsize * np.ones((len(self.node_values_list), numCols, 2))

            self.buffer_offset_right = numCols

            self.current_map = np.concatenate((self.current_map, mapCols), axis=1)
            self.node_values_list = np.concatenate((self.node_values_list, nodeValueCols), axis=1)


    def update_map(self, new_map: np.ndarray, x_offset=0, y_offset=0):
        """    
        Updates the map with new grid whenever map is changed. The node values list is expanded if needed to match the new size of the map.
        The map is updated with the new given map, and buffer is added so that we never have to shrink the map/node values.
        
        After the new map is built, we call update/replan, updating the needed node values, which later calculates the path and returns the new path
        """

        prev_x_offset = self.x_offset
        prev_y_offset = self.y_offset

        # set prev_map- keeps track of the old map
        prev_map = self.current_map.copy()

        new_map = np.array(new_map)

        # compare the prev map with the new one (remove the buffer off of the old map first)
        true_prev_map = prev_map[self.buffer_offset_up:len(prev_map) - self.buffer_offset_down, self.buffer_offset_left:len(prev_map[0]) - self.buffer_offset_right]
        if np.array_equal(true_prev_map, new_map):
            return "same" # This case is handled specifically in dstar-node- basically, don't calculate a new path if nothing changed
        
        rospy.loginfo("Dstar: Building map")
        
        new_node_values = self.node_values_list.copy()

        # Find how many columns and rows are present in the new given map (how much the real map expanded by)
        columnsLeft = int((prev_x_offset - x_offset) / self.resolution)
        columnsRight = len(new_map[0]) - len(true_prev_map[0]) - columnsLeft 

        rowsUp = int((prev_y_offset - y_offset) / self.resolution) 
        rowsDown = len(new_map) - len(true_prev_map) - rowsUp 

        # Find how many columns and rows we'll have to append to node values (the number of new columns/rows, unless already present in the buffer)
        columns_left_node_values = columnsLeft - self.buffer_offset_left if columnsLeft > self.buffer_offset_left else 0  
        columns_right_node_values = columnsRight - self.buffer_offset_right if columnsRight > self.buffer_offset_right else 0  

        rows_up_node_values = rowsUp - self.buffer_offset_up if rowsUp > self.buffer_offset_up else 0  
        rows_down_node_values = rowsDown - self.buffer_offset_down if rowsDown > self.buffer_offset_down else 0  

        # Find how much buffer we'll have to add to the map (to keep it the same size as earlier, the earlier buffer length minus anything new)
        buffer_cols_left = self.buffer_offset_left - columnsLeft if columnsLeft < self.buffer_offset_left else 0
        buffer_cols_right = self.buffer_offset_right - columnsRight if columnsRight < self.buffer_offset_right else 0

        buffer_rows_up = self.buffer_offset_up - rowsUp if rowsUp < self.buffer_offset_up else 0
        buffer_rows_down = self.buffer_offset_down - rowsDown if rowsDown < self.buffer_offset_down else 0

        # Create the new map by taking the new data, and adding any needed buffer to the sides
        buffer_left = -1 * np.ones((len(new_map), buffer_cols_left))
        buffer_right = -1 * np.ones((len(new_map), buffer_cols_right))

        new_map = np.concatenate((buffer_left, new_map, buffer_right), axis=1)

        buffer_up = -1 * np.ones((buffer_rows_up, len(new_map[0])))
        buffer_down = -1 * np.ones((buffer_rows_down, len(new_map[0])))

        new_map = np.concatenate((buffer_up, new_map, buffer_down), axis=0)

        # create new node values by adding any needed new rows/columns to the sides
        new_cols_left = maxsize * np.ones((len(new_node_values), columns_left_node_values, 2))
        new_cols_right = maxsize * np.ones((len(new_node_values), columns_right_node_values, 2))

        new_node_values = np.concatenate((new_cols_left, new_node_values, new_cols_right), axis=1)

        new_rows_up = maxsize * np.ones((rows_up_node_values, len(new_node_values[0]), 2))
        new_rows_down = maxsize * np.ones((rows_down_node_values, len(new_node_values[0]), 2))

        new_node_values = np.concatenate((new_rows_up, new_node_values, new_rows_down), axis=0)

        # update the offsets
        self.buffer_offset_left = buffer_cols_left
        self.buffer_offset_right = buffer_cols_right
        self.buffer_offset_up = buffer_rows_up
        self.buffer_offset_down = buffer_rows_down

        self.x_offset = x_offset
        self.y_offset = y_offset

        # change goal- as the size/buffer of the map has changed, the goal should be reconverted
        self.goal = self.convert_to_grid(self.real_goal)

        # Change the value of the current node- these values (new rows/cols of node values) represent how much the physical size of the map has changed
        self.current_node[0] += rows_up_node_values
        self.current_node[1] += columns_left_node_values

        self.node_values_list = new_node_values

        self.current_map = new_map

        # Call update-replan, which compares the prev map and new map to mark any differences.
        # This eventually calcualtes the new path and returns it.
        return self.update_replan(true_prev_map, columnsLeft, rowsUp)
