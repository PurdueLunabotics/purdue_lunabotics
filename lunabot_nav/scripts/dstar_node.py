#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from lunabot_nav.dstar import Dstar
from lunabot_nav.smoothing import Bezier, lerp

pose = []
grid = []
goal = []
rot_z = 0
goal_z_rot = 0
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
    global pose, rot_z
    position = data.pose.pose.position
    rpy = euler_from_quaternion(
        [
            data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
        ]
    )
    rot_z = rpy[2]
    coords = [position.x, position.y]
    pose = coords


def goal_subscriber(data):
    global goal, need_update_goal, goal_z_rot
    rospy.loginfo("New Goal")
    goal = [data.pose.position.x, data.pose.position.y]
    goal_quat = [
        data.pose.orientation.w,
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
    ]
    goal_euler = euler_from_quaternion(goal_quat)
    goal_z_rot = goal_euler[2]
    need_update_goal = True


def main():
    global grid, update_grid, pose, goal, res, need_update_goal

    rospy.init_node("dstar_ros_script")

    # global params
    odom_topic = rospy.get_param("/odom_topic")
    goal_topic = rospy.get_param("/nav_goal_topic")

    # /nav params
    enable_smoothing = rospy.get_param("enable_smoothing")
    num_lerp_pts = rospy.get_param("num_lerp_pts")
    num_bezier_pts = rospy.get_param("num_bezier_pts")

    # dstar params
    radius = rospy.get_param("~robot_grid_radius")  # robot radius in grid units
    frequency = rospy.get_param("~hz")
    num_downsample = rospy.get_param("~num_downsample")

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
        if (dstar is None) and len(grid) > 0 and len(pose) > 0 and len(goal) > 0:
            dstar = Dstar(goal, pose, grid, radius, res, x_offset, y_offset)
            rospy.loginfo("Initialize")

        if dstar is not None:
            # rospy.loginfo("Iterate")

            dstar.update_position(pose)

            if need_update_goal:
                rospy.loginfo("created new Dstar")
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
                rospy.loginfo("Start create path")
                path = dstar.createPathList()
                path.reverse()
                path = np.array(path)

                if len(path) != 0 and enable_smoothing:
                    points = lerp(num_lerp_pts, path)
                    t_curve = np.linspace(0, 1, num_bezier_pts)
                    path = Bezier.Curve(t_curve, points)
                    rospy.loginfo("End create path")
                vectors = np.diff(path, axis=0)
                angles = np.zeros(vectors.shape[0])

                for i in range(vectors.shape[0]):
                    # Compute the angles between consecutive vectors
                    v = vectors[i]
                    v_u = v / np.linalg.norm(v)
                    angles[i] = np.arctan(v_u[1] / v_u[0])
                angles[-1] = goal_z_rot

                ros_struct = Path()
                ros_struct.poses = []
                ros_struct.header.stamp = rospy.Time.now()
                ros_struct.header.frame_id = "odom"

                for index, point in enumerate(path):
                    if index % num_downsample == 0 or np.all(path[index] == path[-1]):
                        path_pose = PoseStamped()
                        path_pose.pose.position.x = point[0]
                        path_pose.pose.position.y = point[1]

                        if index == 0:
                            global rot_z
                            z_rot = rot_z
                        else:
                            z_rot = angles[index - 1]

                        rotation = quaternion_from_euler(0, 0, z_rot)
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
