import math
from operator import sub, mul

from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np


MAX_VELOCITY = 13
MAX_ACCELERATION = 0.7
last_target_vel = [0,0]

ROBOT_POS = (0, 0)
LOOKAHEAD = 8

closest_point_test = 0
last_look_index = 0
last_look_point = (0, 0)
pose = [ROBOT_POS[0], ROBOT_POS[1], math.pi / 2]
output = [0,0]

pos = (0,0)
path = [(0, 0), (10, 30), (20, 40), (30, 30), (40, 60), (20, 80), (50, 80)]

angle = math.atan2(path[1][0], path[1][1])
t = 0
t_i = 0
wheels = [0,0]

dt=0.05

itt = 0

def injection(pathList, spacing):
    new_points = []
    for i in range(len(pathList) - 1):
        vec = tuple(map(sub, pathList[i + 1], pathList[i]))
        mag = (vec[0] ** 2 + vec[1] ** 2) ** 0.5

        num_points_fit = math.ceil(mag / spacing)
        vec = (vec[0] / mag * spacing, vec[1] / mag * spacing)

        for j in range(num_points_fit):
            new_points.append(tuple(((vec[0] * j) + pathList[i][0], (vec[1] * j) + pathList[i][1])))

    return new_points


def smoothing(pathList, weight_data, weight_smooth, tolerance):
    new_points = [list(ele) for ele in pathList]
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(pathList) - 1):
            for j in range(len(pathList[i])):
                aux = new_points[i][j]
                new_points[i][j] += weight_data * (pathList[i][j] - aux) + weight_smooth * (
                        new_points[i - 1][j] + new_points[i + 1][j] - 2.0 * new_points[i][j])
                change += abs(aux - new_points[i][j])
    return [tuple(l) for l in new_points]


def dist_along_path(path):
    dist = 0
    dists = [0]
    for i in range(1, len(path)):
        dist += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        dists.append(dist)
    return dists


def target_velocity(path, max_velocity, max_acceleration, k):
    curve = curvature_p(path)
    target_vel = [min(max_velocity, k / (curve[i] + 1e-30)) for i in range(len(curve))]

    target_vel[len(target_vel) - 1] = 0
    for i in range(len(target_vel) - 2, 0, -1):
        distance = math.dist(path[i], path[i + 1])
        target_vel[i] = min(target_vel[i], math.sqrt(target_vel[i + 1] ** 2 + 2 * max_acceleration * distance))

    return target_vel


def curvature_p(path):
    curvatures = [0]
    for i in range(1, len(path) - 1):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        x3, y3 = path[i + 1]

        k1 = 0.5 * (x1 ** 2 + y1 ** 2 - x2 ** 2 - y2 ** 2) / (x1 - x2) if x1 != x2 else (x1 + 1E-10 - x2)
        k2 = (y1 - y2) / (x1 - x2) if x1 != x2 else (x1 + 1E-10 - x2)

        b = 0.5 * (x2 ** 2 - 2 * x2 * k1 + y2 ** 2 - x3 ** 2 + 2 * x3 * k1 - y3 ** 2) / (x3 * k2 - y3 + y2 - x2 * k2)
        a = k1 - k2 * b

        r = math.sqrt((x1 - a) ** 2 + (y1 - b) ** 2)

        curvature = 1 / r

        if math.isnan(curvature):
            curvature = 0

        curvatures.append(curvature)

    curvatures.append(0)
    return curvatures


# TODO: Implement slip controller
def controller(target_vel, target_accel, actual_vel, kv, ka, kp, ki, kd, last_error, integral, last_time, current_time):
    error = target_vel - actual_vel
    dt = current_time - last_time
    integral += error * dt
    derivative = (error - last_error) / dt

    last_error = error
    last_time = current_time

    return kv * target_vel + ka * target_accel + kp * error + ki * integral + kd * derivative, last_error, integral, last_time


def closest():
    global path, pos

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]
def lookahead():
    global path, t, t_i, pos

    for i, p in enumerate(reversed(path[:-1])):
        i_ = len(path)-2 - i
        d = (path[i_+1][0]-p[0], path[i_+1][1]-p[1])
        f = (p[0]-pos[0], p[1]-pos[1])

        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(LOOKAHEAD)**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            disc = math.sqrt(disc)
            t1 = (-b + disc)/(2*a)
            t2 = (-b - disc)/(2*a)
            # print("t1=" + str(t1) + ", t2=" + str(t2))
            if 0<=t1<=1:
                # if (t1 >= t and i == t_i) or i > t_i:
                    t = t1
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
            if 0<=t2<=1:
                # if (t2 >= t and i == t_i) or i > t_i:
                    t = t2
                    t_i = i_
                    # print("hit")
                    return p[0]+t*d[0], p[1]+t*d[1]
    t = 0
    t_i = 0
    return path[closest()][0:2]
def curvature(lookahead):
    global path, pos, angle
    side = np.sign(math.sin(3.1415/2 - angle)*(lookahead[0]-pos[0]) - math.cos(3.1415/2 - angle)*(lookahead[1]-pos[1]))
    a = -math.tan(3.1415/2 - angle)
    c = math.tan(3.1415/2 - angle)*pos[0] - pos[1]
    # x = abs(-math.tan(3.1415/2 - angle) * lookahead[0] + lookahead[1] + math.tan(3.1415/2 - angle)*pos[0] - pos[1]) / math.sqrt((math.tan(3.1415/2 - angle))**2 + 1)
    x = abs(a*lookahead[0] + lookahead[1] + c) / math.sqrt(a**2 + 1)
    return side * (2*x/(float(LOOKAHEAD)**2))


def turn(curv, vel, trackwidth):
    return  [vel*(2+curv*trackwidth)/2, vel*(2-curv*trackwidth)/2]

def main():
    global pos, angle, t, t_i, wheels, itt, path
    print("Simulating...")
    path2 = injection(path, 6)

    path = smoothing(path2, 0.2, 0.8, 0.00001)

    target_vels = target_velocity(path, MAX_VELOCITY, MAX_ACCELERATION, 5)
    print(target_vels)
    circle2 = plt.Circle(ROBOT_POS, LOOKAHEAD, color='b', fill=False)
    circle1 = plt.Circle((0, 0), 0.1, color='r', fill=True)
    vector = plt.quiver(ROBOT_POS[0], ROBOT_POS[1], 1, 1, angles='xy', scale_units='xy', scale=1)
    circles = [circle2, circle1]

    def init():
        ax.add_patch(circle2)
        ax.add_patch(circle1)
        return circles

    # print(len(path3))
    fig, ax = plt.subplots()
    ax.scatter(*zip(*path))

    def animate(i):
        global pos, angle, t, t_i, wheels, itt, path
        look = lookahead()
        circles[1].center = look
        close = closest()
        curv = curvature(look) if t_i>close else 0.00001
        vel = target_vels[close]
        last_wheels = wheels
        wheels = turn(curv, vel, 4)

        for i, w in enumerate(wheels):
            wheels[i] = last_wheels[i] + min(float(2*dt), max(-float(2*dt), w-last_wheels[i]))

        pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
        angle += math.atan((wheels[0]-wheels[1])/4*dt)
        # pose = forward_kinematics(wheels[0], wheels[1], 4, (pos[0], pos[1], angle), dt * i)
        # pos = pose[0:2]
        # angle = pose[2]


        circles[0].center = pos[0], pos[1]
        return circles

        # print(str(wheels) + ", " + str(angle))

    ax.set_aspect('equal', adjustable='box')

    anim = animation.FuncAnimation(fig, animate,
                                   init_func=init,
                                   frames=360 * 4,
                                   interval=20,
                                   blit=True)

    anim.save('animation.mp4', fps=30,
              extra_args=['-vcodec', 'h264',
                          '-pix_fmt', 'yuv420p'])

    print("Done!")



# def main():
#     print("Simulating...")
#     path = [(0, 0), (10, 30), (20, 40), (30, 30), (40, 60), (20, 80)]
#     path2 = injection(path, 6)
#
#     path3 = smoothing(path2, 0.2, 0.8, 0.00001)
#
#     # print(len(path3))
#     fig, ax = plt.subplots()
#     ax.scatter(*zip(*path3))
#
#     # print(len(dist_along_path(path3)))
#     # print(len(curvature(path3)))
#
#     target_vels = target_velocity(path3, MAX_VELOCITY, MAX_ACCELERATION, 5)
#     print(target_vels)
#     circle2 = plt.Circle(ROBOT_POS, LOOKAHEAD, color='b', fill=False)
#     circle1 = plt.Circle((0,0), 0.1, color='r', fill=True)
#     vector = plt.quiver(ROBOT_POS[0], ROBOT_POS[1], 1, 1, angles='xy', scale_units='xy', scale=1)
#     circles = [circle2, circle1]
#
#     def init():
#         ax.add_patch(circle2)
#         ax.add_patch(circle1)
#         return circles
#
#     def animate(i):
#         global closest_point_test
#         global last_look_index
#         global last_look_point
#         global pose
#         global last_target_vel
#         global output
#
#         change = 1 / 100
#         closest_point_test = closest_point(path3, closest_point_test, (pose[0], pose[1]))
#         # print("Close point index: ", closest_point_test)
#         # print(calc_lookahead_point(path3, closest_point_test, ROBOT_POS, LOOKAHEAD))
#         look_ahead_pt, look_ahead_index = calc_lookahead_point(path3, closest_point_test, (pose[0], pose[1]), LOOKAHEAD,
#                                                                last_look_index, last_look_point)
#         last_look_index = look_ahead_index
#         last_look_point = look_ahead_pt
#         # print("Look ahead point: ", look_ahead_pt)
#         circles[1].center = look_ahead_pt
#         # ax.plot(float(look_ahead_pt[0]), float(look_ahead_pt[1]), "ro")
#
#         # print(look_ahead_index)
#
#         curv_look = curvature_of_lookahead(path3, LOOKAHEAD, look_ahead_pt, (pose[0], pose[1]), pose[2])
#         target_vel = target_vels[math.ceil(look_ahead_index)]
#         wheel_velocities = wheel_vel(curv_look, target_vel, 4)
#         output[0] += rate_limiter(change * 0.5, wheel_velocities[0], last_target_vel[0])
#         output[1] += rate_limiter(change * 0.5, wheel_velocities[1], last_target_vel[1])
#         last_target_vel = output
#
#         print(output)
#
#
#         # print("Wheel velocities: ", wheel_velocities)
#
#         # right_vector_start = [pose[0] + math.cos(pose[2]) * LOOKAHEAD, pose[1] + math.sin(pose[2]) * LOOKAHEAD]
#         # left_vector_start = [pose[0] - math.cos(pose[2]) * LOOKAHEAD, pose[1] - math.sin(pose[2]) * LOOKAHEAD]
#         #
#         # right_vector_end = [right_vector_start[0] + math.cos(pose[2]) * wheel_velocities[1], right_vector_start[1] + math.sin(pose[2]) * wheel_velocities[1]]
#         # left_vector_end = [left_vector_start[0] + math.cos(pose[2]) * wheel_velocities[0], left_vector_start[1] + math.sin(pose[2]) * wheel_velocities[0]]
#
#         vector.set_UVC(5 * math.cos(pose[2]), 5 * math.sin(pose[2]))
#
#         # pose = forward_kinematics(output[0], output[1], 4, pose, change * i)
#         pose = (pose[0] + (output[0] + output[1]) / 2 * change * math.sin(pose[2]),
#                pose[1] + (output[0] + output[1]) / 2 * change * math.cos(pose[2]), pose[2] - math.atan((output[1] - output[0]) / 4 * change))
#         print("Pose: ", pose)
#
#         circles[0].center = pose[0], pose[1]
#
#         return circles
#
#     ax.set_aspect('equal', adjustable='box')
#
#     anim = animation.FuncAnimation(fig, animate,
#                                    init_func=init,
#                                    frames=360*4,
#                                    interval=20,
#                                    blit=True)
#
#     anim.save('animation.mp4', fps=30,
#               extra_args=['-vcodec', 'h264',
#                           '-pix_fmt', 'yuv420p'])
#
#     print("Done!")

main()
