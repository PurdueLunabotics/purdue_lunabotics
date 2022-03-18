#!/usr/bin/env python
"""
ROS Node Class for global path planning using RRTstar algorithm 
author: Raghava Uppuluri, code adapted from Ahmed Qureshi and AtsushiSakai(@Atsushi_twi)
"""
import copy
import math
import random
import time
import logging

logging.getLogger("matplotlib").setLevel(logging.WARNING)
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

DOF = 2

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


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


class RRTStarPlanner:
    def __init__(
        self,
        visualize,
        goal_sample_rate=5,
        max_iter=1000,
        GAMMA=2,
        DISCRETIZATION_STEP=0.01,
    ):
        """Args:
        disc_step (float, optional): [description]. Defaults to 0.05.
        goal_sample_rate (int, optional): . Defaults to 5.
        max_iter (int, optional): Max iterations. Defaults to 100.
        """
        self.grid = None
        self.resolution = None
        self.grid_height = None
        self.grid_width = None
        self.goal = None
        self.start = None
        self.node_list = []
        self.visualize_path = visualize

        self.min_dist_to_goal = 0.1
        self.GAMMA = GAMMA
        self.DISCRETIZATION_STEP = DISCRETIZATION_STEP

        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.goalfound = False
        self.solution_set = set()
        self.is_planning = False
        self.plan_start = 0

    def planner_cleanup(self):
        self.goalfound = False
        self.solution_set = set()
        self.is_planning = False
        self.plan_start = 0

    def set_grid_data(self, grid, resolution, height, width):
        """Sets the map grid and internal parameters related to the grid

        Args:
            grid (list height * width): Occupancy Grid
            resolution (float): resolution m/cell
            origin (np.array): [x,y] offset from map frame
            height (_type_): _description_
            width (_type_): _description_
        """
        assert grid is not None
        assert resolution is not None
        assert height is not None
        assert width is not None

        self.grid = grid.flatten(order="C")
        self.resolution = resolution
        self.grid_height = height
        self.grid_width = width

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
        if self.grid is None:
            print("Occupancy grid not defined yet...")
            return

        print("planning")
        self.is_planning = True
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
                if self.visualize_path:
                    self.visualize(rnd.state)

        path = self.get_path_to_goal()
        if path is not None:
            plan_time = time.perf_counter() - plan_start
            print("plan time: {}".format(plan_time))
        print("path not found")
        self.planner_cleanup()
        return path

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
            incrementTotal = distTotal / self.DISCRETIZATION_STEP
            dists = dists / incrementTotal
            numSegments = int(math.floor(incrementTotal)) + 1
            logger.debug("state_check: %s", newNode.state)
            logger.debug("dists: %s", dists)
            logger.debug("numSegments: %d", numSegments)
            stateCurr = Node(newNode.state)

            for i in range(0, numSegments):
                if self.__InCollision(stateCurr):
                    logger.debug("COLLISION")
                    return (False, None)
                logger.debug("SAFE")
                stateCurr.state = stateCurr.state + dists
                logger.debug("state_check: %s", stateCurr.state)

            if self.__InCollision(dest):
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
            x_max = self.grid_width * self.resolution
            # x_min = max(self.curr[0] - self.sample_window_w, 0)
            y_max = self.grid_height * self.resolution
            # y_min = max(self.curr[1] - self.sample_window_h, 0)

            sample = [
                np.random.uniform(0, x_max),
                np.random.uniform(0, y_max),
            ]
            rnd = Node(np.array(sample))
        else:
            rnd = self.goal
        return rnd

    def is_near_goal(self, node):
        """_summary_
        Checks whether node is within self.min_dist_to_goal of goal

        Args:
            node (Node): location to check

        Returns:
            bool: true or false
        """
        d = np.linalg.norm(node.state - self.goal.state)
        if d < self.min_dist_to_goal:
            return True
        return False

    def gen_final_course(self, goalind):
        path = [self.goal.state]
        while self.node_list[goalind].parent is not None:
            node = self.node_list[goalind]
            path.append(node.state)
            goalind = node.parent
        path.append(self.start.state)
        return path

    def find_near_nodes(self, newNode):
        # Use this value of gamma
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

    def __cspace_to_grid(self, state):
        pos = state.copy()
        pos = pos / self.resolution
        uv_pos = np.round(pos).astype("uint32")
        return uv_pos

    def __to_flat(self, uv_ind):
        return self.grid_width * uv_ind[0] + uv_ind[1]

    def __InCollision(self, node):
        uv_ind = self.__cspace_to_grid(node.state)
        flat_ind = self.__to_flat(uv_ind)
        logger.debug("flat_ind: %s", flat_ind)
        if flat_ind < 0 or flat_ind >= len(self.grid):
            return True  # outside of c_space, so it is invalid
        return self.grid[flat_ind] > 0.5

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

    def visualize(self, rnd):

        plt.clf()
        grid = self.grid.copy().reshape(self.grid_height, self.grid_width)
        plt.grid(True)
        obs = np.nonzero(grid > 0.5)
        plt.scatter(obs[0], obs[1], marker="o")

        for node in self.node_list:
            if node.parent is not None:
                if node.state is not None:
                    edge = np.array(
                        [
                            [node.state[0], node.state[1]],
                            [
                                self.node_list[node.parent].state[0],
                                self.node_list[node.parent].state[1],
                            ],
                        ]
                    )
                    edge = self.__cspace_to_grid(edge)
                    plt.plot(edge[:, 0], edge[:, 1], "-g")

        if self.goalfound:
            path = self.get_path_to_goal()
            if path is not None:
                path = np.array(path)
                path = self.__cspace_to_grid(path)
                plt.plot(path[:, 0], path[:, 1], "-r")

        if rnd is not None:
            rnd = self.__cspace_to_grid(rnd)
            plt.plot(rnd[0], rnd[1], "^k")

        start = self.__cspace_to_grid(self.start.state)
        goal = self.__cspace_to_grid(self.goal.state)

        plt.plot(start[0], start[1], "xb")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")
        plt.axis([0, self.grid_height, 0, self.grid_width])
        plt.pause(0.01)
