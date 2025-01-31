#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from lunabot_nav.global_planner import Map, RRTStarPlanner
from lunabot_nav.smoothing import Bezier, lerp
from lunabot_nav.utils import pose_to_array, state_to_pose_msg


class GlobalPlannerNode:
    def __init__(self):
        self.costmap = None
        self.costmap_info = None
        self.curr_pos = None
        self.goal = None
        self.origin = None

        # Global Params
        odom_topic = rospy.get_param("/odom_topic")
        goal_topic = rospy.get_param("/nav_goal_topic")

        # Nav Params
        map_topic = rospy.get_param("map_topic")
        global_path_topic = rospy.get_param("global_path_topic")
        self.occ_threshold = rospy.get_param("occ_threshold")
        self.bezier_step = rospy.get_param("bezier_step")
        self.lerp_step = rospy.get_param("lerp_step")

        # RRTStar Params
        max_iter = rospy.get_param("~max_iter")
        disc_step = rospy.get_param("~disc_step")
        goal_sample_rate = rospy.get_param("~goal_sample_rate")
        gamma = rospy.get_param("~gamma")
        self.planner = RRTStarPlanner(
            max_iter=max_iter,
            disc_step=disc_step,
            goal_sample_rate=goal_sample_rate,
            GAMMA=gamma,
        )
        self.new_goal = False

        self.planner.grid = Map(self.occ_threshold)
        self.t_curve = np.arange(0, 1, self.bezier_step)

        rospy.Subscriber(map_topic, OccupancyGrid, self.__occ_grid_cb)
        rospy.Subscriber(odom_topic, Odometry, self.__odom_cb)
        rospy.Subscriber(goal_topic, PoseStamped, self.__goal_cb)

        self.path_publisher = rospy.Publisher(global_path_topic, Path, queue_size=10)

    def __occ_grid_cb(self, grid_msg):
        self.planner.grid.from_msg(grid_msg)

    def __odom_cb(self, odom_msg):
        pos, _ = pose_to_array(odom_msg.pose.pose)
        self.curr_pos = np.array([pos[0], pos[1]])

    def __goal_cb(self, goal_msg):
        pos, (r, p, y) = pose_to_array(goal_msg.pose)
        self.goal = np.array([pos[0], pos[1], y])
        self.new_goal = True

    def plan(self):
        if self.curr_pos is not None and self.goal is not None and self.new_goal:
            path = self.planner.plan(self.curr_pos, self.goal[:-1])
            if path is not None:
                path = self.smoothing(path)
                self.publish_path(path)
                self.new_goal = False

    def publish_path(self, poses):
        """Path (in map frame - offset by self.origin from odom)

        Args:
            poses (_type_): _description_
        """
        poses = poses[::-1]
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"
        path.poses = [state_to_pose_msg(pose) for pose in poses]
        self.path_publisher.publish(path)

    def smoothing(self, points):
        """Smoothens path
        Returns:
            np.array: Outputs numpy array of current path waypoints
        """
        try:
            points = lerp(self.lerp_step, points)
            curve = Bezier.Curve(self.t_curve, points)
            print(curve.shape)
        except BaseException as e:
            print(e)
            curve = points
        return curve


def main():
    rospy.init_node("global_planner_node")
    planner = GlobalPlannerNode()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        planner.plan()
        r.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
