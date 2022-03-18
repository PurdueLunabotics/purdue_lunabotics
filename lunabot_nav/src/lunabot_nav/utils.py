import numpy as np
import rospy
from geometry_msgs.msg import Point32, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def state_to_pose_msg(state, frame_id="map"):
    """Generates a PoseStamped ROS msg from a numpy array of the robot c-space

    Args:
        state (numpy.array): [x (m), y (m), theta (deg)]

    Returns:
        PoseStamped: Pose of robot with time stamp and relative to 'map' tf frame
    """
    rot = quaternion_from_euler(0, 0, state[-1])
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = state[0]
    pose.pose.position.y = state[1]
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

