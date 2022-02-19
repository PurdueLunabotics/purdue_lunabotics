#!/usr/bin/env python3
"""
ROS Node Class for global path planning using RRTstar algorithm 
author: Raghava Uppuluri, code adapted from Ahmed Qureshi and AtsushiSakai(@Atsushi_twi)
"""

import copy
import math
import random
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from conversions import point_from_xyz, pose_to_array, state_to_pose_msg


def rotate(pts, rad, rotate_pt):
    """Applies a 2d rotation to an array of points
    Args:
        points (np.array): array of dim 2xN, where N is the number of points
        rad ([type]): [description]

    Returns:
        [type]: [description]
    """
    pts -= rotate_pt
    c, s = np.cos(rad), np.sin(rad)
    j = np.array([[c, s], [-s, c]])
    pts = j @ pts
    pts += rotate_pt

    return pts


class RRTStarPlanner:
    def __init__(
        self, disc_step=0.05, goal_sample_rate=5, max_iter=30
    ):
        """Args:
        disc_step (float, optional): [description]. Defaults to 0.05.
        goal_sample_rate (int, optional): . Defaults to 5.
        max_iter (int, optional): Max iterations. Defaults to 100.
        """
        (
            self.grid,
            self.resolution,
            self.origin,
            self.maxheight,
            self.maxwidth,
            self.last_map_update,
        ) = (None, None, None, None, None, None)
        self.robot_h, self.robot_w = 1, 0.5
        self.footprint = np.array(
            [
                (self.robot_h * x * 0.5, self.robot_w * y * 0.5)
                for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
            ]
        )
        self.curr = None  # np.array [x,y,theta] in meters and radians
        self.dof = 3
        self.goal = (
            None  # np.array [x,y,theta] in meters and radians, (theta is ignored)
        )
        self.node_list = deque([])

        # samples positions in the window +/- from the robot's current position
        self.sample_window_h = 3.0
        self.sample_window_w = 3.0

        self.min_dist_to_goal = 0.1

        self.disc_step = disc_step
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.goalfound = False
        self.solution_set = set()
        self.is_planning = False
        self.plan_start = 0

        rospy.Subscriber("/projected_map", OccupancyGrid, self.__occ_grid_cb)
        rospy.Subscriber("/odom", Odometry, self.__odom_cb)
        rospy.Subscriber("/goal", PoseStamped, self.__goal_cb)

        self.path_publisher = rospy.Publisher(
            "lunabot_nav/path_generator", Path, queue_size=100
        )
        self.ftprt_publisher = rospy.Publisher(
            "/footprint", PolygonStamped, queue_size=100
        )

    def __occ_grid_cb(self, grid_msg):
        if not self.is_planning:
            self.last_map_update = grid_msg.header.stamp
            self.grid = np.array(grid_msg.data)
            self.resolution = grid_msg.info.resolution  # m/cell
            self.origin, _ = np.array(
                pose_to_array(grid_msg.info.origin)
            )  # (pose, ori = usually zero, so same frame as 'map')
            self.maxwidth = grid_msg.info.width  # m/cell
            self.maxheight = grid_msg.info.height  # m/cell

    def __odom_cb(self, odom_msg):
        pos, ori = pose_to_array(odom_msg.pose.pose)
        self.curr = np.array([pos[0], pos[1], ori[0]])

    def publish_footprint(self):
        footprint = PolygonStamped()
        footprint.header.stamp = rospy.Time.now()
        footprint.header.frame_id = "base_link"
        footprint.polygon.points = [point_from_xyz(x, y, 0) for x, y in self.footprint]
        self.ftprt_publisher.publish(footprint)

    def publish_path(self, poses):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = poses
        self.path_publisher.publish(path)

    def __goal_cb(self, goal_msg):
        if not self.is_planning:
            pos, ori = pose_to_array(goal_msg.pose)
            self.goal = np.array([pos[0], pos[1], ori[0]])
            self.plan()

    def plan(self):
        """
        Implements the RTT (or RTT*) algorithm, following the pseudocode in the handout.
        You should read and understand this function, but you don't have to change any of its code - just implement the 3 helper functions.
        """
        if self.goal is None or self.curr is None or self.grid is None:
            print("Goal, current state, or occupancy grid not defined yet...")
            return

        print("planning")
        self.is_planning = True
        self.plan_start = rospy.Time.now().secs
        # self.node_list.appendleft(Node(self.curr))
        self.node_list = [Node(self.curr)]

        for i in range(self.max_iter):
            rnd = self.generate_sample()
            nind = self.nearest_list_index(self.node_list, rnd)
            rnd_valid, rnd_cost = self.steer_to(rnd, self.node_list[nind])

            if rnd_valid:
                new_node = copy.deepcopy(rnd)
                new_node.parent = nind
                new_node.cost = rnd_cost + self.node_list[nind].cost

                near_inds = self.find_near_nodes(
                    new_node
                )  # you'll implement this method
                new_parent = self.choose_parent(
                    new_node, near_inds
                )  # you'll implement this method

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

        path = self.get_path_to_goal()
        if path is not None:
            print("publishing path")
            plan_time = rospy.Time.now().secs - self.plan_start
            print("plan time: {}".format(plan_time))
            self.publish_path(path)
        else:
            print("path not found")
        self.publish_footprint()
        self.is_planning = False

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
        """

        newNode = copy.deepcopy(source)
        DISCRETIZATION_STEP = self.disc_step
        dists = dest.state - source.state
        distTotal = np.linalg.norm(dists)

        if distTotal > 0:
            incrementTotal = distTotal / DISCRETIZATION_STEP
            dists /= incrementTotal

            numSegments = int(math.floor(incrementTotal)) + 1
            stateCurr = Node(newNode.state)

            for i in range(0, numSegments):
                if not self.__CollisionCheck(stateCurr):
                    return (False, None)
                stateCurr.state += dists

            if not self.__CollisionCheck(dest):
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
            x_max = min(self.curr[0] + self.sample_window_h, self.maxheight)
            x_min = max(self.curr[0] - self.sample_window_h, 0)
            y_max = min(self.curr[1] + self.sample_window_w, self.maxwidth)
            y_min = max(self.curr[1] - self.sample_window_w, 0)

            sample = [
                np.random.uniform(x_min, x_max),
                np.random.uniform(y_min, y_max),
                np.random.uniform(-np.pi, np.pi),
            ]
            rnd = Node(np.array(sample))
        else:
            rnd = Node(self.goal)
        return rnd

    def is_near_goal(self, node):
        """_summary_
        Checks whether node is within self.min_dist_to_goal of goal

        Args:
            node (Node): location to check 
 
        Returns:
            bool: true or false 
        """
        d = np.linalg.norm(node.state - self.goal)
        if d < self.min_dist_to_goal:
            return True
        return False

    def gen_final_course(self, goalind):
        """
        Traverses up the tree to find the path from start to goal

        goalind: index of the goal node

        Returns: a list of coordinates, representing the path backwards. Traverse this list in reverse order to follow the path from start to end
        """
        path = [state_to_pose_msg(self.goal, "base_link")]
        while self.node_list[goalind].parent is not None:
            node = self.node_list[goalind]
            pose = state_to_pose_msg(node.state, "base_link")
            path.append(pose)
            goalind = node.parent
        path.append(state_to_pose_msg(self.curr, "base_link"))
        return path

    def find_near_nodes(self, newNode):
        """
        Finds all nodes in the tree that are "near" newNode.
        See the assignment handout for the equation defining the cutoff point (what it means to be "near" newNode)

        newNode: the node to be inserted.

        Returns: a list of indices of nearby nodes.
        """
        # Use this value of gamma
        GAMMA = 10
        i = len(self.node_list)
        upper_bound = GAMMA * (np.log(i) / i) ** (1.0 / self.dof)
        near_nodes = []
        for i, node in enumerate(self.node_list):
            if np.linalg.norm(newNode.state - node.state) <= upper_bound:
                near_nodes.append(i)
        return near_nodes

    def rewire(self, newNode, newNodeIndex, nearinds):
        """
        Should examine all nodes near newNode, and decide whether to "rewire" them to go through newNode.
        Recall that a node should be rewired if doing so would reduce its cost.

        newNode: the node that was just inserted
        newNodeIndex: the index of newNode
        nearinds: list of indices of nodes near newNode
        """
        for i in nearinds:
            currCost = self.node_list[i].cost
            initIdx = 0
            valid, toNewCost = self.steer_to(newNode, self.node_list[initIdx])
            if not valid:
                continue
            valid, newtoCurrCost = self.steer_to(self.node_list[i], newNode)
            if not valid:
                continue
            withNewNodeCost = newtoCurrCost + toNewCost
            if withNewNodeCost < currCost:
                # rewire step
                newNode.parent = initIdx
                self.node_list[i].parent = newNodeIndex
                newNode.cost = toNewCost
                # updating cost of descendants of newNode with stack
                self.node_list[i].cost = withNewNodeCost
                updateList = list(self.node_list[i].children)
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

    def __CollisionCheck(self, node):
        """
        Checks whether a given configuration is valid. (collides with obstacles)

        You will need to modify this for question 2 (if self.geom == 'circle') and question 3 (if self.geom == 'rectangle')
        """

        pos = node.state[0:2] - self.origin[0:2]
        translated_robot = self.footprint + pos  # maps to origin in 2d
        # robot bounding box = [x_min,y_min, x_max,y_max]
        r_bnds = np.hstack(
            (np.min(translated_robot, axis=0), np.max(translated_robot, axis=0))
        )

        occupied_i = np.array(np.nonzero(self.grid > 0.5))  # flat coords of obstacles
        occupied_i = np.vstack(
            (occupied_i / self.robot_w, occupied_i % self.robot_w)
        )  # xy coords
        occupied_i *= self.resolution  # maps to real-world coordinates
        occupied_i -= self.origin[0:2].reshape(2, 1)  # maps to map frame in 2d
        occupied_i = rotate(
            occupied_i, -node.state[-1], pos.reshape(2, 1)
        )  # rotate points onto robot to check collisions at angle

        is_colliding = np.any(
            (occupied_i >= r_bnds[:2].reshape(2, 1))
            & (occupied_i <= r_bnds[2:].reshape(2, 1))
        )  # colliding
        return not is_colliding

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
                    self.node_list[idx].state - self.goal
                )
                if goalind is None or cost < mincost:
                    goalind = idx
                    mincost = cost
            return self.gen_final_course(goalind)
        else:
            return None


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


def main():
    rospy.init_node("global_path_planner")

    rrt = RRTStarPlanner(max_iter=30, disc_step=0.05, goal_sample_rate=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if rrt.grid is not None:
            """
            print(rrt.grid.shape)
            print(rrt.maxwidth)
            print(rrt.maxheight)
            print(rrt.origin)
            print(rrt.last_map_update)
            print(rrt.curr)
            """
        rate.sleep()


if __name__ == "__main__":
    main()
