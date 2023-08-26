#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf.transformations import quaternion_from_euler

from lunabot_nav.dstar import Dstar

pose = []
grid = []
goal = []
update_grid = False
need_update_goal = False
res = 0
x_offset = 0
y_offset = 0


def grid_subscriber(data):
    global update_grid, grid, res, x_offset, y_offset
    width = data.info.width
    height = data.info.height
    grid = np.reshape(data.data, (height, width))
    res = data.info.resolution
    x_offset = data.info.origin.position.x
    y_offset = data.info.origin.position.y
    update_grid = True


def grid_update_subscriber(data):
    global update_grid, grid, res, x_offset, y_offset

    # grid = np.array(data.data).reshape(
    #    (data.height, data.width), order="F"
    # )
    temp_map = grid.copy()
    index = 0
    for i in range(data.y, data.y + data.height):
        for j in range(data.x, data.x + data.width):
            temp_map[i][j] = data.data[index]
            index += 1

    grid = temp_map.copy()
    update_grid = True


def position_subscriber(data):
    global pose
    position = data.pose.pose.position

    coords = [position.x, position.y]
    pose = coords


def goal_subscriber(data):
    global goal, need_update_goal
    print("New Goal")
    goal = [data.pose.position.x, data.pose.position.y]
    need_update_goal = True


def main():
    global grid, update_grid, pose, goal, res, need_update_goal

    rospy.init_node("dstar_ros_script")

    odom_topic = rospy.get_param("/odom_topic")
    goal_topic = rospy.get_param("/nav_goal_topic")

    radius = 8  # robot rad (grid units)

    frequency = 10  # hz

    dstar = None

    path_publisher = rospy.Publisher("/nav/global_path", Path, queue_size=10)

    rospy.Subscriber(
        "/maps/costmap_node/global_costmap/costmap", OccupancyGrid, grid_subscriber
    )

    rospy.Subscriber(
        "/maps/costmap_node/global_costmap/costmap_updates",
        OccupancyGridUpdate,
        grid_update_subscriber,
    )

    rospy.Subscriber(odom_topic, Odometry, position_subscriber)

    rospy.Subscriber(goal_topic, PoseStamped, goal_subscriber)

    rate = rospy.Rate(frequency)

    completedInitialRun = False

    path = []
    while not rospy.is_shutdown():
        # print("Loop")
        if (dstar is None) and len(grid) > 0 and len(pose) > 0 and len(goal) > 0:
            dstar = Dstar(goal, pose, grid, radius, res, x_offset, y_offset)
            print("Initialize")

        if dstar is not None:
            # print("Iterate")

            dstar.update_position(pose)

            if need_update_goal:
                print("created new Dstar")
                dstar = Dstar(goal, pose, grid, radius, res, x_offset, y_offset)
                completedInitialRun = False
                # dstar.update_goal(goal)
                need_update_goal = False

            if update_grid:
                dstar.update_map(grid, x_offset, y_offset)
                update_grid = False

            if not completedInitialRun:
                dstar.find_path(True)
                completedInitialRun = True

            if dstar.needs_new_path:
                print("Start create path")
                path = np.array(dstar.createPathList())
                print("End create path")

                # if (len(path) == 0):
                #     print("publish empty path")
                # else:
                #     print("published path")

                ros_struct = Path()
                ros_struct.poses = []
                ros_struct.header.stamp = rospy.Time.now()
                ros_struct.header.frame_id = "odom"

                for index, point in enumerate(path):
                    if index % 5 == 0 or np.all(path[index] == path[-1]):

                        path_pose = PoseStamped()

                        path_pose.pose.position.x = point[0]
                        path_pose.pose.position.y = point[1]

                        rotation = quaternion_from_euler(0, 0, 0)
                        path_pose.pose.orientation.x = rotation[0]
                        path_pose.pose.orientation.y = rotation[1]
                        path_pose.pose.orientation.z = rotation[2]
                        path_pose.pose.orientation.w = rotation[3]

                        ros_struct.poses.append(path_pose)

                # plt.imshow(temp, cmap='hot', interpolation='nearest')
                # plt.show()
                path_publisher.publish(ros_struct)

        rate.sleep()


if __name__ == "__main__":
    main()
