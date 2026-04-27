#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Pose

import numpy as np

def point_from_pose_2d(pose: PoseStamped | Pose):
    """
    Converts a Pose or PoseStamped into a 2D np point.
    """
    if isinstance(pose, Pose):
        return np.array([pose.position.x, pose.position.y])
    else:
        return np.array([pose.pose.position.x, pose.pose.position.y])