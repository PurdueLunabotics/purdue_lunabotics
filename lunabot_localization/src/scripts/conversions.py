"""Functions to convert between ros msg and numpy data representations 
"""

import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Transform


def np_to_tf(tf_matrix):
    tf = Transform()
    quat = quaternion_from_matrix(tf_matrix)
    trans = tf_matrix[:, -1]
    tf.rotation.x = quat[0]
    tf.rotation.y = quat[1]
    tf.rotation.z = quat[2]
    tf.rotation.w = quat[3]
    tf.translation.x = trans[0]
    tf.translation.y = trans[1]
    tf.translation.z = trans[2]
    return tf


def np_to_pose(tf_matrix):
    pose = Pose()
    quat = quaternion_from_matrix(tf_matrix)
    trans = tf_matrix[:, -1]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    return pose


def pose_to_np_matrix(trans, rot):
    trans = np.array([trans.x, trans.y, trans.z, 1])
    rot = np.array([rot.x, rot.y, rot.z, rot.w])
    matrix = quaternion_matrix(rot)
    matrix[:, -1] = trans
    return matrix