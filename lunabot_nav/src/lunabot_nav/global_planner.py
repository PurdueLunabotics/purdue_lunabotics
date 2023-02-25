#!/usr/bin/env python
"""
Global path planner using RRTstar algorithm
author: Raghava Uppuluri, code adapted from Ahmed Qureshi and AtsushiSakai(@Atsushi_twi)
"""
import copy
import logging
import math
import os
import random
import time

import numpy as np

from lunabot_nav.utils import pose_to_array

if os.environ.get("MPL_VISUALIZE") == "1":
    from lunabot_nav.utils import visualize


DOF = 2

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class Map:
    """Map class"""

    def __init__(self, occ_threshold=0.5):
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.grid = None
        self.occ_threshold = occ_threshold

    def from_msg(self, grid_msg):
        self.resolution = grid_msg.info.resolution  # m/cell
        self.origin, _ = np.array(
            pose_to_array(grid_msg.info.origin)
        )  # (pose, ori = usually zero, so same frame as 'map')
        self.origin = self.origin[:2]  # only x,y
        self.width = grid_msg.info.width  # m/cell
        self.height = grid_msg.info.height  # m/cell
        self.grid = np.array(grid_msg.data).reshape(
            (self.width, self.height), order="F"
        )

    def from_data(
        self, grid, resolution, height, width, origin=np.zeros(2), occ_threshold=0.5
    ):
        """Sets the map grid and internal parameters related to the grid

        Args:
            grid (1D np.array height * width): Occupancy Grid
            resolution (float): resolution m/cell
            height (int): the number of rows of the grid in cells (rows or x-axis)
            width (int): the width of the grid in cells (cols or y-axis)
            origin (np.array): the (x,y) offset of the grid in  in the robot cspace
        """
        assert grid is not None
        assert resolution is not None
        assert height is not None
        assert width is not None

        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = grid.reshape(width, height)
        self.origin = origin
        self.occ_threshold = occ_threshold

    def cspace_to_grid(self, state):
        """Converts an np.array in c-space to a discretized index in the occ grid

        Args:
            state (_type_): np.array of size DOF

        Returns:
            np.array 2x1: index of state in uv format of the occ grid
        """
        assert self.initialized, "Not initalized with set_data or set_msg"
        pos = state.copy() - self.origin
        pos = pos / self.resolution
        uv_pos = np.round(pos).astype("uint32")
        return uv_pos

    @property
    def cspace_spec(self):

        assert self.initialized, "Not initalized with set_data or set_msg"
        x_spec = np.array([0, self.width * self.resolution]) + self.origin[0]
        y_spec = np.array([0, self.height * self.resolution]) + self.origin[1]
        return x_spec, y_spec

    @property
    def initialized(self):
        return (
            self.resolution is not None
            and self.origin is not None
            and self.width is not None
            and self.height is not None
            and self.grid is not None
            and self.grid is not None
            and self.occ_threshold is not None
        )

    def in_collision(self, node):
        """Checks if node corresponds to an occupied state in the occupancy grid

        Args:
            node (Node): node to check in collision

        Returns:
            bool: Returns True if in Collision and False otherwise
        """
        assert self.initialized, "Not initalized with set_data or set_msg"

        w, h = self.cspace_to_grid(node.state)
        if w < 0 or w >= self.width:
            return True
        if h < 0 or h >= self.height:
            return True
        logger.debug("uv_ind: (%d,%d)", w, h)
        return self.grid[w, h] > self.occ_threshold


class Node:
    """Node class"""

    def __init__(self, state):
        """
        Args:
            state (np.array): array of size 1xDOF, where DOF is the dimension of the robot c-space
        """
        self.state = state
        self.cost = 0.0
        self.parent = None
        self.children = set()

    def __lt__(self, other):
        return self.cost < other.cost

    def dist(self, other):
        return np.sqrt(np.sum((other.state - self.state) ** 2))


class Planner:
    def __init__(self):
        self.grid = None
        self.goal = None
        self.start = None
        self.node_list = []
        self.visualize = os.environ.get("MPL_VISUALIZE") == "1"
        self.min_dist_to_goal = 0.1

    def plan(self, start, goal):
        raise NotImplementedError

    def get_path_to_goal(self):
        raise NotImplementedError

    def is_near_goal(self, node):
        """Checks whether node is within self.min_dist_to_goal of goal

        Args:
            node (Node): location to check

        Returns:
            bool: true or false
        """
        d = np.linalg.norm(node.state - self.goal.state)
        if d < self.min_dist_to_goal:
            return True
        return False


# class AStarNode(Node):
#     def __init__(self,state):
#         super().__init__(state)

# class AStar(Planner):
#     def __init__(self):
#         super().__init__()
#         self.came_from = {}
#         self.g_score = defaultdict(np.inf)
#         self.f_score = defaultdict(np.inf)

#     def h(self, node):
#         collision_penalty = np.inf if self.in_collision(node) else 0
#         return self.goal.dist(node) + collision_penalty

#     def plan(self, start, goal):
#         self.start = Node(start)
#         self.goal = Node(goal)
#         open_set = [Node(start)]

#         heapq.heappush()


class RRTStarPlanner(Planner):
    def __init__(
        self, goal_sample_rate=15, max_iter=100, GAMMA=10, disc_step=0.05, **kwargs
    ):
        """Implements RRT*, a sampling-based planner

        Args:
            goal_sample_rate (int, optional): How often the sampler will sample the goal state. Defaults to 10.
            max_iter (int, optional): Max iterations of the planning loop to run. Defaults to 100.
            GAMMA (int, optional): Hyperparameter that determines nearest nodes to randomly sample node that will be rewired. Defaults to 10.
            discretization_step (float, optional): step size when determining if two points have a collision-free straight-line path between them. Defaults to 0.05.
        """
        super().__init__(**kwargs)

        self.GAMMA = GAMMA
        self.disc_step = disc_step

        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.goalfound = False
        self.solution_set = set()

    def plan(self, start, goal):
        """Plan path

        Args:
            start (np.array): start configuration of size DOF in GRID frame
            goal (np.array): goal configuration of size DOF in GRID frame
        """
        assert start is not None
        assert goal is not None
        self.start = Node(start)
        self.goal = Node(goal)
        if not self.grid.initialized:
            logger.info("Occupancy grid not defined yet...")
            return

        logger.info("planning")
        plan_start = time.perf_counter()
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.generate_sample()
            nind = self.nearest_list_index(self.node_list, rnd)
            rnd_valid, rnd_cost = self.steer_to(rnd, self.node_list[nind])
            if rnd_valid:
                new_node = copy.deepcopy(rnd)
                new_node.parent = nind
                new_node.cost = rnd_cost + self.node_list[nind].cost

                near_inds = self.find_near_nodes(new_node)
                new_parent = self.choose_parent(new_node, near_inds)

                # insert newNode into the tree
                if new_parent is not None:
                    new_node.parent = new_parent
                    new_node.cost = (
                        np.linalg.norm(
                            new_node.state - self.node_list[new_parent].state
                        )
                        + self.node_list[new_parent].cost
                    )
                else:
                    pass  # nind is already set as newNode's parent
                self.node_list.append(new_node)
                newNodeIndex = len(self.node_list) - 1
                self.node_list[new_node.parent].children.add(newNodeIndex)

                self.rewire(
                    new_node, newNodeIndex, near_inds
                )  # you'll implement this method

                if self.is_near_goal(new_node):
                    self.solution_set.add(newNodeIndex)
                    self.goalfound = True
                logger.info("goalfound: %s", self.goalfound)
                if self.visualize:
                    visualize(self, rnd.state)

        path = self.get_path_to_goal()
        if path is not None:
            plan_time = time.perf_counter() - plan_start
            logger.info("plan time: %.3f", plan_time)
        logger.info("path not found")
        self.cleanup()
        return path

    def cleanup(self):
        """Resets the solution_set, goalfound class variables, called after every call to self.plan"""
        self.goalfound = False
        self.solution_set = set()

    def choose_parent(self, newNode, nearinds):
        """
        Selects the best parent for newNode. This should be the one that results in newNode having the lowest possible cost.

        newNode: the node to be inserted
        nearinds: a list of indices. Contains nodes that are close enough to newNode to be considered as a possible parent.

        Returns: index of the new parent selected
        """
        min_cost = np.inf
        best_parent = None
        for i in nearinds:
            valid, cost = self.steer_to(newNode, self.node_list[i])
            if valid:
                assert cost is not None
                cost += self.node_list[i].cost
                if cost < min_cost:
                    best_parent = i
                    min_cost = cost
        return best_parent

    def steer_to(self, dest, source):
        """
        Charts a route from source to dest, and checks whether the route is collision-free.
        Discretizes the route into small steps, and checks for a collision at each step.

        This function is used in planning() to filter out invalid random samples. You may also find it useful
        for implementing the functions in question 1.

        dest: destination node
        source: source node

        returns: (success, cost) tuple
            - success is True if the route is collision free; False otherwise.
            - cost is the distance from source to dest, if the route is collision free; or None otherwise.
            7,1 1,1 6,0
        """

        newNode = copy.deepcopy(source)
        dists = dest.state - source.state
        distTotal = np.linalg.norm(dists)

        if distTotal > 0:
            incrementTotal = distTotal / self.disc_step
            dists = dists / incrementTotal
            numSegments = int(math.floor(incrementTotal)) + 1
            logger.debug("state_check: %s", newNode.state)
            logger.debug("dists: %s", dists)
            logger.debug("numSegments: %d", numSegments)
            stateCurr = Node(newNode.state)

            for i in range(0, numSegments):
                if self.grid.in_collision(stateCurr):
                    logger.debug("COLLISION")
                    return (False, None)
                logger.debug("SAFE")
                stateCurr.state = stateCurr.state + dists
                logger.debug("state_check: %s", stateCurr.state)

            if self.grid.in_collision(dest):
                return (False, None)
            return (True, distTotal)
        else:
            return (False, None)

    def generate_sample(self):
        """
        Randomly generates a sample within a sample_window around the current position to minimize the search space, to be used as a new node.
        This sample may be invalid - if so, call generate_sample() again.

        Returns:
            Node(state=np.array(dof)): random sample
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            x_spec, y_spec = self.grid.cspace_spec
            sample = [
                np.random.uniform(*x_spec),
                np.random.uniform(*y_spec),
            ]
            rnd = Node(np.array(sample))
        else:
            rnd = self.goal
        return rnd

    def gen_final_course(self, goalind):
        """Generates list of c-space states through looping by parent

        Args:
            goalind (int): index of the self.goal node in self.node_list

        Returns:
            list(np.array): list of c-space states of dim DOF of the robot planning c-space
        """
        path = [self.goal.state]
        while self.node_list[goalind].parent is not None:
            node = self.node_list[goalind]
            path.append(node.state)
            goalind = node.parent
        path.append(self.start.state)
        return np.array(path)

    def find_near_nodes(self, newNode):
        """Returns indicies of nodes within a certain radius from the newNode calculated by GAMMA

        Args:
            newNode (Node): sampled node to check for near nodes

        Returns:
            list(int): list of indicies of near nodes
        """
        i = len(self.node_list)
        upper_bound = self.GAMMA * (np.log(i) / i) ** (1.0 / DOF)
        near_nodes = []
        for i, node in enumerate(self.node_list):
            if np.linalg.norm(newNode.state - node.state) <= upper_bound:
                near_nodes.append(i)
        return near_nodes

    def rewire(self, newNode, newNodeIndex, nearinds):
        for i in nearinds:
            curr = self.node_list[i]
            currCost = curr.cost
            valid, toCurrCost = self.steer_to(curr, newNode)
            if not valid:
                continue
            withNewNodeCost = toCurrCost + newNode.cost
            if withNewNodeCost < currCost:
                # rewire step
                self.node_list[curr.parent].children.remove(i)
                curr.parent = newNodeIndex
                newNode.children.add(i)
                curr.cost = withNewNodeCost

                # updating cost of descendants of curr with stack
                updateList = list(curr.children)
                costDelta = withNewNodeCost - currCost
                while len(updateList) > 0:
                    updateNodeIdx = updateList.pop()
                    self.node_list[updateNodeIdx].cost += costDelta
                    updateList.extend(self.node_list[updateNodeIdx].children)

    def nearest_list_index(self, nodeList, rnd):
        """
        Searches nodeList for the closest vertex to rnd

        nodeList: list of all nodes currently in the tree
        rnd: node to be added (not currently in the tree)

        Returns: index of nearest node
        """
        dlist = []
        for node in nodeList:
            dlist.append(np.linalg.norm(rnd.state - node.state))

        minind = dlist.index(min(dlist))

        return minind

    def get_path_to_goal(self):
        """
        Traverses the tree to chart a path between the start state and the goal state.
        There may be multiple paths already discovered - if so, this returns the shortest one

        Returns: a list of coordinates, representing the path backwards; if a path has been found; None otherwise
        """
        if self.goalfound:
            goalind = None
            mincost = float("inf")
            for idx in self.solution_set:
                cost = self.node_list[idx].cost + np.linalg.norm(
                    self.node_list[idx].state - self.goal.state
                )
                if goalind is None or cost < mincost:
                    goalind = idx
                    mincost = cost
            return self.gen_final_course(goalind)
        else:
            return None
