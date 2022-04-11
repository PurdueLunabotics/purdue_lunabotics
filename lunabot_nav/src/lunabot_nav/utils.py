import numpy as np
import rospy
from geometry_msgs.msg import Point32, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import logging
logging.getLogger("matplotlib").setLevel(logging.WARNING)

def grid_to_file(grid, cols, file='map_grid.txt'):
    s = ''
    for i in range(len(grid)):
        s += str(grid[i])
        s += ' '
        if i % cols == 0:
            s += '\n'
    with open(file, "w") as text_file:
        text_file.write(s)

def grid_to_file_rc(grid, file='map_grid_rc.txt'):
    s = ''
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            s += str(grid[i][j])
            s += ' '
        s += '\n'

    with open(file, "w") as text_file:
        text_file.write(s)



def visualize(planner,rnd):
    import matplotlib.pyplot as plt
    """Visualizes obstacles, self.node_list, self.goal, self.start at step in self.plan

    Args:
        rnd (np.array): randomly sampled state at step in self.plan
    """

    plt.clf()
    obs = np.nonzero(planner.grid.grid >= planner.grid.occ_threshold)
    plt.scatter(obs[0], obs[1], marker="o")

    for node in planner.node_list:
        if node.parent is not None:
            if node.state is not None:
                edge = np.array(
                    [
                        [node.state[0], node.state[1]],
                        [
                            planner.node_list[node.parent].state[0],
                            planner.node_list[node.parent].state[1],
                        ],
                    ]
                )
                edge = planner.grid.cspace_to_grid(edge)
                plt.plot(edge[:, 0], edge[:, 1], "-g")

    if planner.goalfound:
        path = planner.get_path_to_goal()
        if path is not None:
            path = np.array(path)
            path = planner.grid.cspace_to_grid(path)
            plt.plot(path[:, 0], path[:, 1], "-r")

    if rnd is not None:
        rnd = planner.grid.cspace_to_grid(rnd)
        plt.plot(rnd[0], rnd[1], "^k")

    start = planner.grid.cspace_to_grid(planner.start.state)
    goal = planner.grid.cspace_to_grid(planner.goal.state)

    plt.plot(start[0], start[1], "xb")
    plt.plot(goal[0], goal[1], "xb")
    plt.axis([0, planner.grid.height, 0, planner.grid.width])

    plt.xlim(0,planner.grid.width)
    plt.ylim(0,planner.grid.height)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(which='both')
    plt.pause(0.01)


def state_to_pose_msg(pos, rot=0.0, frame_id="map"):
    """Generates a PoseStamped ROS msg from a numpy array of the robot c-space

    Args:
        state (numpy.array): [x (m), y (m)] 
        pos (float): theta (deg)

    Returns:
        PoseStamped: Pose of robot with time stamp and relative to 'map' tf frame
    """
    rot = quaternion_from_euler(0, 0, rot)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = 0
    pose.pose.orientation.x = rot[1]
    pose.pose.orientation.y = rot[2]
    pose.pose.orientation.z = rot[3]
    pose.pose.orientation.w = rot[0]
    return pose


def pose_to_array(pose_msg):
    pose = [
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z,
    ]
    ori = [
        pose_msg.orientation.w,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
    ]
    ori = euler_from_quaternion(ori)
    return pose, ori


def point_from_xyz(x, y, z):
    pt = Point32()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt
