#!/usr/bin/env python3
import numpy as np
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

""" Converts a probability value to log odds notation

:param value: The value to convert to log odds from pure probability
"""


def prob_to_log(value):
    return np.log(value / (1 - value))


""" Converts a logg odds value to probability notation

:param value: The value to convert to probability from log odds
"""


def log_to_prob(value):
    return 1 - 1 / (1 + np.exp(value))


""" Converts a quaternion to euler angles

:param quat: The quaternion to convert to euler angles
"""


def quat_to_euler(quat):
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    angles = {}
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    x1 = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) > 1:
        y1 = np.copysign(np.pi / 2, sinp)
    else:
        y1 = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    z1 = math.atan2(siny_cosp, cosy_cosp)
    return np.array([x1, y1, z1])


def euler_to_quat(euler):
    pitch = euler[0]
    roll = euler[1]
    yaw = euler[2]
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])
