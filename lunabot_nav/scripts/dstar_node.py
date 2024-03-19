#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf.transformations import quaternion_from_euler

from lunabot_nav.dstar import Dstar

class DstarNode:

    def __init__(self):
        self.grid: np.ndarray[int] = np.array([]) # 2d array occupancy map: Occupancy probabilities from [0 to 100].  Unknown is -1.
        self.pose: list[float] = [] # [x, y], in meters/odom frame
        self.goal: list[float] = [] 

        self.grid_update_needed: bool = False
        self.goal_update_needed: bool = False

        self.resolution: float = 0  # meters per grid cell

        self.x_offset: float = 0 # real world pose of the point 0,0 in the grid
        self.y_offset: float = 0

        self.dstar: Dstar = None

        self.path_sampling_rate = 5 # Take every <n-th> point from the path


    def grid_callback(self, data: OccupancyGrid):
        """
        Update the grid given a new occupancy grid. Update the flag such that dstar will update the map.
        """

        width = data.info.width
        height = data.info.height
        self.grid = np.reshape(data.data, (height, width))

        self.resolution = data.info.resolution
        self.x_offset = data.info.origin.position.x
        self.y_offset = data.info.origin.position.y
        self.grid_update_needed = True


    def grid_update_callback(self, data: OccupancyGridUpdate):
        """
        Update the grid given the occupancy grid update (applied on top of the current grid). Also update the flag for dstar to update the map.
        """

        temp_map = self.grid.copy()

        index = 0
        for i in range(data.y, data.y + data.height):
            for j in range(data.x, data.x + data.width):
                temp_map[i][j] = data.data[index]
                index += 1

        self.grid = temp_map.copy()
        self.grid_update_needed = True


    def position_callback(self, data: Odometry):
        position = data.pose.pose.position

        coords = [position.x, position.y]
        self.pose = coords


    def goal_callback(self, data: PoseStamped):
        self.goal = [data.pose.position.x, data.pose.position.y]
        self.goal_update_needed = True


    def dstar_loop(self):
        """
        Main loop for dstar. Update / manage dstar and its data, and publish the path whenever a new one becomes available.
        """

        rospy.init_node("dstar_ros_script")

        odom_topic = rospy.get_param("/odom_topic")
        goal_topic = rospy.get_param("/nav_goal_topic")

        radius = 8  # robot rad (grid units)

        frequency = 10  # hz

        path_publisher = rospy.Publisher("/nav/global_path", Path, queue_size=10, latch=True)

        rospy.Subscriber("/maps/costmap_node/global_costmap/costmap", OccupancyGrid, self.grid_callback)
        rospy.Subscriber("/maps/costmap_node/global_costmap/costmap_updates", OccupancyGridUpdate, self.grid_update_callback)
        rospy.Subscriber(odom_topic, Odometry, self.position_callback)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback)

        rate = rospy.Rate(frequency)

        completed_initial_run = False # Keep track of whether a new Dstar object (algorithm) has processed the initial map or not.

        while not rospy.is_shutdown():

            # Startup condition: once all data is available, create a new Dstar object
            if (self.dstar is None) and len(self.grid) > 0 and len(self.pose) > 0 and len(self.goal) > 0:
                self.dstar = Dstar(self.goal, self.pose, self.grid, radius, self.resolution, self.x_offset, self.y_offset)

            if self.dstar is not None:

                self.dstar.update_position(self.pose)

                if self.goal_update_needed:
                    # If we've gotten a new goal, it is more efficient to reset the dstar data (as it is all based on goal location)

                    self.dstar = Dstar(self.goal, self.pose, self.grid, radius, self.resolution, self.x_offset, self.y_offset)
                    completed_initial_run = False
                    self.goal_update_needed = False

                if self.grid_update_needed:
                    # If the map has changed, let dstar make the necessary updates
                    
                    self.dstar.update_map(self.grid, self.x_offset, self.y_offset)
                    self.grid_update_needed = False

                if not completed_initial_run:
                    # Before calculating any new path, ensure the map is processed by the algorithm

                    self.dstar.find_path(True)
                    completed_initial_run = True

                if self.dstar.needs_new_path:
                    # If we have a new path (new data), get the path and publish it

                    path_data = np.array(self.dstar.createPathList())

                    path: Path = Path()
                    path.poses = []
                    path.header.stamp = rospy.Time.now()
                    path.header.frame_id = "odom"

                    for index, point in enumerate(path_data):
                        if index % self.path_sampling_rate == 0 or index == len(path_data) - 1:
                            # Sample every <path sampling rate> points (+ the last one)

                            path_pose = PoseStamped()

                            path_pose.pose.position.x = point[0]
                            path_pose.pose.position.y = point[1]

                            rotation = quaternion_from_euler(0, 0, 0)
                            path_pose.pose.orientation.x = rotation[0]
                            path_pose.pose.orientation.y = rotation[1]
                            path_pose.pose.orientation.z = rotation[2]
                            path_pose.pose.orientation.w = rotation[3]

                            path.poses.append(path_pose)


                    path_publisher.publish(path)

            rate.sleep()


if __name__ == "__main__":
    dstar_node = DstarNode()
    dstar_node.dstar_loop()
