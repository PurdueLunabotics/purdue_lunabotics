"""Bezier, a module for creating Bezier curves. SOURCE: https://github.com/torresjrjr/Bezier.py
Version 1.1, from < BezierCurveFunction-v1.ipynb > on 2019-05-02
"""

import math
from enum import Enum

import numpy as np
from scipy.interpolate import splev, splprep

__all__ = ["Bezier"]


def lerp(step, pts):
    def get_traj(start, goal):
        interp = lambda start, end, t: (1 - t) * start + t * end
        return np.array([interp(start, goal, t) for t in np.arange(0, 1, step)])

    path = np.array([get_traj(pts[i], pts[i + 1]) for i in range(len(pts) - 1)])
    path = path.reshape(-1, path.shape[-1])
    return path


def spline(step, pts):
    print(pts.T)
    tck, u = splprep(pts.T, u=None, k=min(3, len(pts) - 1), s=1)
    u_new = np.linspace(u.min(), u.max(), 1000)
    pts = splev(u_new, tck, der=0)
    return pts


class Bezier:
    def TwoPoints(t, P1, P2):
        """
        Returns a point between P1 and P2, parametised by t.
        INPUTS:
            t     float/int; a parameterisation.
            P1    numpy array; a point.
            P2    numpy array; a point.
        OUTPUTS:
            Q1    numpy array; a point.
        """

        if not isinstance(P1, np.ndarray) or not isinstance(P2, np.ndarray):
            raise TypeError("Points must be an instance of the numpy.ndarray!")
        if not isinstance(t, (int, float)):
            raise TypeError("Parameter t must be an int or float!")

        Q1 = (1 - t) * P1 + t * P2
        return Q1

    def Points(t, points):
        """
        Returns a list of points interpolated by the Bezier process
        INPUTS:
            t            float/int; a parameterisation.
            points       list of numpy arrays; points.
        OUTPUTS:
            newpoints    list of numpy arrays; points.
        """
        newpoints = []
        # print("points =", points, "\n")
        for i1 in range(0, len(points) - 1):
            # print("i1 =", i1)
            # print("points[i1] =", points[i1])

            newpoints += [Bezier.TwoPoints(t, points[i1], points[i1 + 1])]
            # print("newpoints  =", newpoints, "\n")
        return newpoints

    def Point(t, points):
        """
        Returns a point interpolated by the Bezier process
        INPUTS:
            t            float/int; a parameterisation.
            points       list of numpy arrays; points.
        OUTPUTS:
            newpoint     numpy array; a point.
        """
        newpoints = points
        # print("newpoints = ", newpoints)
        while len(newpoints) > 1:
            newpoints = Bezier.Points(t, newpoints)
            # print("newpoints in loop = ", newpoints)

        # print("newpoints = ", newpoints)
        # print("newpoints[0] = ", newpoints[0])
        return newpoints[0]

    def Curve(t_values, points):
        """
        Returns a point interpolated by the Bezier process
        INPUTS:
            t_values     list of floats/ints; a parameterisation.
            points       list of numpy arrays; points.
        OUTPUTS:
            curve        list of numpy arrays; points.
        """

        if not hasattr(t_values, "__iter__"):
            raise TypeError(
                "`t_values` Must be an iterable of integers or floats, of length greater than 0 ."
            )
        if len(t_values) < 1:
            raise TypeError(
                "`t_values` Must be an iterable of integers or floats, of length greater than 0 ."
            )
        if not isinstance(t_values[0], (int, float)):
            raise TypeError(
                "`t_values` Must be an iterable of integers or floats, of length greater than 0 ."
            )

        curve = np.array([[0.0] * len(points[0])])
        for t in t_values:
            # print("curve                  \n", curve)
            # print("Bezier.Point(t, points) \n", Bezier.Point(t, points))

            curve = np.append(curve, [Bezier.Point(t, points)], axis=0)

            # print("curve after            \n", curve, "\n--- --- --- --- --- --- ")
        curve = np.delete(curve, 0, 0)
        # print("curve final            \n", curve, "\n--- --- --- --- --- --- ")
        return curve


"""
Calculate Dubins Curve between waypoints

author: Fischer, but I just copied the math from this paper:

fischergabbert@gmail.com

http://mems.eng.uci.edu/files/2014/04/Dubins_Set_Robotics_2001.pdf

Andrew Walker did this in C and I used that as a reference too..his github is out there somewhere

"""

"""
TODOS:

- Reduce computation time using the classification methods in the paper

"""


class TurnType(Enum):
    LSL = 1
    LSR = 2
    RSL = 3
    RSR = 4
    RLR = 5
    LRL = 6


class Waypoint:
    def __init__(self, x, y, psi):
        self.x = x
        self.y = y
        self.psi = psi / np.pi * 180

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", psi: " + str(self.psi)


class Param:
    def __init__(
        self,
        p_init,
        seg_final,
        turn_radius,
    ):
        self.p_init = p_init
        self.seg_final = seg_final
        self.turn_radius = turn_radius
        self.type = 0


class Trajectory:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def wrapTo360(angle):
    posIn = angle > 0
    angle = angle % 360
    if angle == 0 and posIn:
        angle = 360
    return angle


def wrapTo180(angle):
    q = (angle < -180) or (180 < angle)
    if q:
        angle = wrapTo360(angle + 180) - 180
    return angle


def headingToStandard(hdg):
    # Convert NED heading to standard unit cirlce...degrees only for now (Im lazy)
    thet = wrapTo360(90 - wrapTo180(hdg))
    return thet


def calcDubinsPath(wpt1, wpt2, vel, phi_lim):
    # Calculate a dubins path between two waypoints
    param = Param(wpt1, 0, 0)
    tz = [0, 0, 0, 0, 0, 0]
    pz = [0, 0, 0, 0, 0, 0]
    qz = [0, 0, 0, 0, 0, 0]
    param.seg_final = [0, 0, 0]
    # Convert the headings from NED to standard unit cirlce, and then to radians
    psi1 = headingToStandard(wpt1.psi) * math.pi / 180
    psi2 = headingToStandard(wpt2.psi) * math.pi / 180

    # Do math
    param.turn_radius = (vel * vel) / (9.8 * math.tan(phi_lim * math.pi / 180))
    dx = wpt2.x - wpt1.x
    dy = wpt2.y - wpt1.y
    D = math.sqrt(dx * dx + dy * dy)
    d = (
        D / param.turn_radius
    )  # Normalize by turn radius...makes length calculation easier down the road.

    # Angles defined in the paper
    theta = math.atan2(dy, dx) % (2 * math.pi)
    alpha = (psi1 - theta) % (2 * math.pi)
    beta = (psi2 - theta) % (2 * math.pi)
    best_word = -1
    best_cost = -1

    # Calculate all dubin's paths between points
    tz[0], pz[0], qz[0] = dubinsLSL(alpha, beta, d)
    tz[1], pz[1], qz[1] = dubinsLSR(alpha, beta, d)
    tz[2], pz[2], qz[2] = dubinsRSL(alpha, beta, d)
    tz[3], pz[3], qz[3] = dubinsRSR(alpha, beta, d)
    tz[4], pz[4], qz[4] = dubinsRLR(alpha, beta, d)
    tz[5], pz[5], qz[5] = dubinsLRL(alpha, beta, d)

    # Now, pick the one with the lowest cost
    for x in range(6):
        if tz[x] != -1:
            cost = tz[x] + pz[x] + qz[x]
            if cost < best_cost or best_cost == -1:
                best_word = x + 1
                best_cost = cost
                param.seg_final = [tz[x], pz[x], qz[x]]

    param.type = TurnType(best_word)
    return param


# Here's all of the dubins path math
def dubinsLSL(alpha, beta, d):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    tmp1 = math.atan2((math.cos(beta) - math.cos(alpha)), tmp0)
    p_squared = (
        2
        + d * d
        - (2 * math.cos(alpha - beta))
        + (2 * d * (math.sin(alpha) - math.sin(beta)))
    )
    if p_squared < 0:
        # print('No LSL Path')
        p = -1
        q = -1
        t = -1
    else:
        t = (tmp1 - alpha) % (2 * math.pi)
        p = math.sqrt(p_squared)
        q = (beta - tmp1) % (2 * math.pi)
    return t, p, q


def dubinsRSR(alpha, beta, d):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    tmp1 = math.atan2((math.cos(alpha) - math.cos(beta)), tmp0)
    p_squared = (
        2
        + d * d
        - (2 * math.cos(alpha - beta))
        + 2 * d * (math.sin(beta) - math.sin(alpha))
    )
    if p_squared < 0:
        # print('No RSR Path')
        p = -1
        q = -1
        t = -1
    else:
        t = (alpha - tmp1) % (2 * math.pi)
        p = math.sqrt(p_squared)
        q = (-1 * beta + tmp1) % (2 * math.pi)
    return t, p, q


def dubinsRSL(alpha, beta, d):
    tmp0 = d - math.sin(alpha) - math.sin(beta)
    p_squared = (
        -2
        + d * d
        + 2 * math.cos(alpha - beta)
        - 2 * d * (math.sin(alpha) + math.sin(beta))
    )
    if p_squared < 0:
        # print('No RSL Path')
        p = -1
        q = -1
        t = -1
    else:
        p = math.sqrt(p_squared)
        tmp2 = math.atan2((math.cos(alpha) + math.cos(beta)), tmp0) - math.atan2(2, p)
        t = (alpha - tmp2) % (2 * math.pi)
        q = (beta - tmp2) % (2 * math.pi)
    return t, p, q


def dubinsLSR(alpha, beta, d):
    tmp0 = d + math.sin(alpha) + math.sin(beta)
    p_squared = (
        -2
        + d * d
        + 2 * math.cos(alpha - beta)
        + 2 * d * (math.sin(alpha) + math.sin(beta))
    )
    if p_squared < 0:
        # print('No LSR Path')
        p = -1
        q = -1
        t = -1
    else:
        p = math.sqrt(p_squared)
        tmp2 = math.atan2((-1 * math.cos(alpha) - math.cos(beta)), tmp0) - math.atan2(
            -2, p
        )
        t = (tmp2 - alpha) % (2 * math.pi)
        q = (tmp2 - beta) % (2 * math.pi)
    return t, p, q


def dubinsRLR(alpha, beta, d):
    tmp_rlr = (
        6
        - d * d
        + 2 * math.cos(alpha - beta)
        + 2 * d * (math.sin(alpha) - math.sin(beta))
    ) / 8
    if abs(tmp_rlr) > 1:
        # print('No RLR Path')
        p = -1
        q = -1
        t = -1
    else:
        p = (2 * math.pi - math.acos(tmp_rlr)) % (2 * math.pi)
        t = (
            alpha
            - math.atan2(
                (math.cos(alpha) - math.cos(beta)), d - math.sin(alpha) + math.sin(beta)
            )
            + p / 2 % (2 * math.pi)
        ) % (2 * math.pi)
        q = (alpha - beta - t + (p % (2 * math.pi))) % (2 * math.pi)

    return t, p, q


def dubinsLRL(alpha, beta, d):
    tmp_lrl = (
        6
        - d * d
        + 2 * math.cos(alpha - beta)
        + 2 * d * (-1 * math.sin(alpha) + math.sin(beta))
    ) / 8
    if abs(tmp_lrl) > 1:
        # print('No LRL Path')
        p = -1
        q = -1
        t = -1
    else:
        p = (2 * math.pi - math.acos(tmp_lrl)) % (2 * math.pi)
        t = (
            -1 * alpha
            - math.atan2(
                (math.cos(alpha) - math.cos(beta)), d + math.sin(alpha) - math.sin(beta)
            )
            + p / 2
        ) % (2 * math.pi)
        q = ((beta % (2 * math.pi)) - alpha - t + (p % (2 * math.pi))) % (2 * math.pi)
        # print(t,p,q,beta,alpha)
    return t, p, q


def dubins_traj(param, step):
    # Build the trajectory from the lowest-cost path
    x = 0
    i = 0
    length = (
        param.seg_final[0] + param.seg_final[1] + param.seg_final[2]
    ) * param.turn_radius
    length = math.floor(length / step)
    path = -1 * np.ones((length, 3))

    while x < length:
        path[i] = dubins_path(param, x)
        x += step
        i += 1
    return path


def dubins_path(param, t):
    # Helper function for curve generation
    tprime = t / param.turn_radius
    p_init = np.array([0, 0, headingToStandard(param.p_init.psi) * math.pi / 180])
    #
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    DIRDATA = np.array(
        [
            [L_SEG, S_SEG, L_SEG],
            [L_SEG, S_SEG, R_SEG],
            [R_SEG, S_SEG, L_SEG],
            [R_SEG, S_SEG, R_SEG],
            [R_SEG, L_SEG, R_SEG],
            [L_SEG, R_SEG, L_SEG],
        ]
    )
    #
    types = DIRDATA[param.type.value - 1][:]
    param1 = param.seg_final[0]
    param2 = param.seg_final[1]
    mid_pt1 = dubins_segment(param1, p_init, types[0])
    mid_pt2 = dubins_segment(param2, mid_pt1, types[1])

    if tprime < param1:
        end_pt = dubins_segment(tprime, p_init, types[0])
    elif tprime < (param1 + param2):
        end_pt = dubins_segment(tprime - param1, mid_pt1, types[1])
    else:
        end_pt = dubins_segment(tprime - param1 - param2, mid_pt2, types[2])

    end_pt[0] = end_pt[0] * param.turn_radius + param.p_init.x
    end_pt[1] = end_pt[1] * param.turn_radius + param.p_init.y
    end_pt[2] = end_pt[2] % (2 * math.pi)

    return end_pt


def dubins_segment(seg_param, seg_init, seg_type):
    # Helper function for curve generation
    L_SEG = 1
    S_SEG = 2
    R_SEG = 3
    seg_end = np.array([0.0, 0.0, 0.0])
    if seg_type == L_SEG:
        seg_end[0] = (
            seg_init[0] + math.sin(seg_init[2] + seg_param) - math.sin(seg_init[2])
        )
        seg_end[1] = (
            seg_init[1] - math.cos(seg_init[2] + seg_param) + math.cos(seg_init[2])
        )
        seg_end[2] = seg_init[2] + seg_param
    elif seg_type == R_SEG:
        seg_end[0] = (
            seg_init[0] - math.sin(seg_init[2] - seg_param) + math.sin(seg_init[2])
        )
        seg_end[1] = (
            seg_init[1] + math.cos(seg_init[2] - seg_param) - math.cos(seg_init[2])
        )
        seg_end[2] = seg_init[2] - seg_param
    elif seg_type == S_SEG:
        seg_end[0] = seg_init[0] + math.cos(seg_init[2]) * seg_param
        seg_end[1] = seg_init[1] + math.sin(seg_init[2]) * seg_param
        seg_end[2] = seg_init[2]

    return seg_end


def dubins(pts, goal_theta):
    pts_size = len(pts)
    path = np.empty((pts_size - 1, 95, 3))
    print(pts)
    for i in range(pts_size - 1):
        theta_0 = math.atan2((pts[i, 1] - pts[i + 1, 1]), (pts[i, 0] - pts[i + 1, 0]))
        theta_1 = (
            math.atan2((pts[i + 1, 1] - pts[i + 2, 1]), (pts[i + 1, 0] - pts[i + 2, 0]))
            if i + 2 != pts_size
            else goal_theta
        )
        wp1 = Waypoint(pts[i, 0], pts[i, 1], theta_0)
        print(wp1)
        wp2 = Waypoint(pts[i + 1, 0], pts[i + 1, 1], theta_1)
        print(wp2)
        param = calcDubinsPath(wp1, wp2, 90, 20)
        dub_path = dubins_traj(param, 150)
        path[i] = dub_path
        # path = np.concatenate([path,dubins_traj(param,1)])
    path = path.reshape(((pts_size - 1) * 95, 3))
    return path
