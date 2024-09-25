import math
import random
from operator import sub, mul

from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np


class PurePursuitController:
    def __init__(self, path, MAX_VELOCITY, MAX_ACCEL, MAX_VEL_CHANGE, LOOKAHEAD, robot_pos):
        self.path = path
        self.MAX_VELOCITY = MAX_VELOCITY
        self.MAX_ACCELERATION = MAX_ACCEL
        self.MAX_VEL_CHANGE = MAX_VEL_CHANGE

        self.robot_pose = robot_pos
        self.LOOKAHEAD = LOOKAHEAD

        self.t = 0
        self.t_i = 0

        self.wheels = [0.0, 0.0]
        self.pos = self.robot_pose[0:2]
        self.angle = self.robot_pose[2]

    # To 
    def injection(self, spacing):
        new_points = []
        for i in range(len(self.path) - 1):
            vec = tuple(map(sub, self.path[i + 1], self.path[i]))
            mag = (vec[0] ** 2 + vec[1] ** 2) ** 0.5

            num_points_fit = math.ceil(mag / spacing)
            vec = (vec[0] / mag * spacing, vec[1] / mag * spacing)

            for j in range(num_points_fit):
                new_points.append(tuple(((vec[0] * j) + self.path[i][0], (vec[1] * j) + self.path[i][1])))

        new_points.append(self.path[len(self.path) - 1])
        self.path = new_points

    def smoothing(self, weight_data, weight_smooth, tolerance):
        new_points = [list(ele) for ele in self.path]
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path) - 1):
                for j in range(len(self.path[i])):
                    aux = new_points[i][j]
                    new_points[i][j] += weight_data * (self.path[i][j] - aux) + weight_smooth * (
                            new_points[i - 1][j] + new_points[i + 1][j] - 2.0 * new_points[i][j])
                    change += abs(aux - new_points[i][j])
        self.path = [tuple(l) for l in new_points]

    def dist_along_path(self):
        dist = 0
        dists = [0]
        for i in range(1, len(self.path)):
            dist += math.hypot(self.path[i][0] - self.path[i - 1][0], self.path[i][1] - self.path[i - 1][1])
            dists.append(dist)
        return dists

    def target_velocity(self, k):
        curve = self.curvature_p()
        target_vel = [min(self.MAX_VELOCITY, k / (curve[i] + 1e-30)) for i in range(len(curve))]

        target_vel[len(target_vel) - 1] = 0
        for i in range(len(target_vel) - 2, 0, -1):
            distance = math.dist(self.path[i], self.path[i + 1])
            target_vel[i] = min(target_vel[i], math.sqrt(target_vel[i + 1] ** 2 + 2 * self.MAX_ACCELERATION * distance))

        return target_vel

    def curvature_p(self):
        curvatures = [0]
        for i in range(1, len(self.path) - 1):
            x1, y1 = self.path[i - 1]
            x2, y2 = self.path[i]
            x3, y3 = self.path[i + 1]

            k1 = 0.5 * (x1 ** 2 + y1 ** 2 - x2 ** 2 - y2 ** 2) / (x1 - x2) if x1 != x2 else (x1 + 1E-10 - x2)
            k2 = (y1 - y2) / (x1 - x2) if x1 != x2 else (x1 + 1E-10 - x2)

            b = 0.5 * (x2 ** 2 - 2 * x2 * k1 + y2 ** 2 - x3 ** 2 + 2 * x3 * k1 - y3 ** 2) / (
                    x3 * k2 - y3 + y2 - x2 * k2 + 1e-50)
            a = k1 - k2 * b

            r = math.sqrt((x1 - a) ** 2 + (y1 - b) ** 2)

            curvature = 1 / r

            if math.isnan(curvature):
                curvature = 0

            curvatures.append(curvature)

        curvatures.append(0)
        return curvatures

    def controller(self, target_vel, target_accel, actual_vel, kv, ka, kp, ki, kd, last_error, integral, last_time,
                   current_time):
        error = target_vel - actual_vel
        dt = current_time - last_time
        integral += error * dt
        derivative = (error - last_error) / dt

        last_error = error
        last_time = current_time

        return kv * target_vel + ka * target_accel + kp * error + ki * integral + kd * derivative, last_error, integral, last_time

    def combine_wheel_vels(self, left, right, wheel_base):
        linear = (left + right) / 2
        angular = (right - left) / wheel_base
        return [linear, angular]

    def closest(self):
        mindist = (
            0, math.sqrt((self.path[0][0] - self.robot_pose[0]) ** 2 + (self.path[0][1] - self.robot_pose[1]) ** 2))
        for i, p in enumerate(self.path):
            dist = math.sqrt((p[0] - self.robot_pose[0]) ** 2 + (p[1] - self.robot_pose[1]) ** 2)
            if dist < mindist[1]:
                mindist = (i, dist)

        return mindist[0]

    def lookahead(self):
        for i, p in enumerate(reversed(self.path[:-1])):
            i_ = len(self.path) - 2 - i
            d = (self.path[i_ + 1][0] - p[0], self.path[i_ + 1][1] - p[1])
            f = (p[0] - self.robot_pose[0], p[1] - self.robot_pose[1])

            a = sum(j ** 2 for j in d)
            b = 2 * sum(j * k for j, k in zip(d, f))
            c = sum(j ** 2 for j in f) - float(self.LOOKAHEAD) ** 2
            disc = b ** 2 - 4 * a * c
            if disc >= 0:
                disc = math.sqrt(disc)
                t1 = (-b + disc) / (2 * a)
                t2 = (-b - disc) / (2 * a)
                # print("t1=" + str(t1) + ", t2=" + str(t2))
                if 0 <= t1 <= 1:
                    # if (t1 >= t and i == t_i) or i > t_i:
                    t = t1
                    self.t_i = i_
                    # print("hit")
                    return p[0] + t * d[0], p[1] + t * d[1]
                if 0 <= t2 <= 1:
                    # if (t2 >= t and i == t_i) or i > t_i:
                    self.t = t2
                    self.t_i = i_
                    # print("hit")
                    return p[0] + self.t * d[0], p[1] + self.t * d[1]
        self.t = 0
        self.t_i = 0
        return self.path[self.closest()][0:2]
    
    def calc_heading_err(self, lookahead):
        vec_look = (lookahead[0] - self.robot_pose[0], lookahead[1] - self.robot_pose[1])
        robot_vec = (math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2]))
        return self.angle_between_vectors(vec_look, robot_vec)
    
    def forward_kinematics(self, wheels, robot_pose, wheelbase, dt):
        return (robot_pose[0] + (wheels[0] + wheels[1]) / 2 * dt * math.sin(robot_pose[2]),
               robot_pose[1] + (wheels[0] + wheels[1]) / 2 * dt * math.cos(robot_pose[2]), 
               robot_pose[2] + math.atan((wheels[0] - wheels[1]) / wheelbase * dt))
        
    def calc_slip(self, robot_pose_kf, actual_robot_pose):
        return self.subtract_vectors(robot_pose_kf, actual_robot_pose)
    
    def subtract_vectors(v1, v2):
        return tuple(map(sub, v1, v2))
        
    def angle_between_vectors(v1, v2):
    # Implementation of angle_between_vectors method
    # dot = v1[0] * v2[0] + v1[1] * v2[1]
    # mag = magnitude(v1) * magnitude(v2)
    # return math.acos(dot / mag)
        v1_u = unit_vector(v1)
        v2_u = unit_vector(v2)
        return 0.0 if math.isnan(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))) else np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        

    def curvature(self, lookahead):
        side = np.sign(math.sin(3.1415 / 2 - self.robot_pose[2]) * (lookahead[0] - self.robot_pose[0]) - math.cos(
            3.1415 / 2 - self.robot_pose[2]) * (
                               lookahead[1] - self.robot_pose[1]))
        a = -math.tan(3.1415 / 2 - self.robot_pose[2])
        c = math.tan(3.1415 / 2 - self.robot_pose[2]) * self.robot_pose[0] - self.robot_pose[1]
        # x = abs(-math.tan(3.1415/2 - angle) * lookahead[0] + lookahead[1] + math.tan(3.1415/2 - angle)*pos[0] - pos[1]) / math.sqrt((math.tan(3.1415/2 - angle))**2 + 1)
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(self.LOOKAHEAD) ** 2))

    def turn(self, curv, vel, trackwidth):
        return [vel * (2 + curv * trackwidth) / 2, vel * (2 - curv * trackwidth) / 2]

    def generate_path(self, spacing, weight_data, weight_smooth, tolerance):
        self.injection(spacing)
        self.smoothing(weight_data, weight_smooth, tolerance)

    def update_vel(self, dt, current_pos, current_vel):
        self.robot_pose = current_pos
        target_vels = self.target_velocity(5)
        look = self.lookahead()
        close = self.closest()
        curv = self.curvature(look) if self.t_i > close else 0.00001
        vel = target_vels[close]
        last_wheels = self.wheels
        self.wheels = self.turn(curv, vel, 4)

        for i, w in enumerate(self.wheels):
            self.wheels[i] = last_wheels[i] + min(float(self.MAX_VEL_CHANGE * dt),
                                                  max(-float(self.MAX_VEL_CHANGE * dt), w - last_wheels[i]))

        # TODO: Add controller when done and output velocities

    def simulate(self, dt):
        print("Simulating...")
        self.generate_path(6, 0.1, 0.1, 0.00001)

        target_vels = self.target_velocity(5)
        circle2 = plt.Circle((self.robot_pose[0], self.robot_pose[1]), self.LOOKAHEAD, color='b', fill=False)
        circle1 = plt.Circle((0, 0), 0.1, color='r', fill=True)
        vector = plt.quiver(self.robot_pose[0], self.robot_pose[1], 1, 1, angles='xy', scale_units='xy', scale=1)
        circles = [circle2, circle1]

        def init():
            ax.add_patch(circle2)
            ax.add_patch(circle1)
            return circles

        # print(len(path3))
        fig, ax = plt.subplots()
        ax.scatter(*zip(*self.path))

        def animate(i):
            # TODO: Add wheelbase var
            look = self.lookahead()
            circles[1].center = look
            close = self.closest()
            curv = self.curvature(look) if self.t_i > close else 0.00001
            vel = target_vels[close]
            last_wheels = self.wheels
            self.wheels = self.turn(curv, vel, 4)

            for i, w in enumerate(self.wheels):
                self.wheels[i] = last_wheels[i] + min(float(self.MAX_VEL_CHANGE * dt),
                                                      max(-float(self.MAX_VEL_CHANGE * dt), w - last_wheels[i]))
                # self.wheels[i] = random.randrange(850, 1000) * self.wheels[i] / 1000
                self.wheels[i] = max(min(np.random.normal(0.90, 0.05), 1), 0.5) * self.wheels[i]

            self.pos = (self.pos[0] + (self.wheels[0] + self.wheels[1]) / 2 * dt * math.sin(self.angle),
                        self.pos[1] + (self.wheels[0] + self.wheels[1]) / 2 * dt * math.cos(self.angle))
            self.angle += math.atan((self.wheels[0] - self.wheels[1]) / 4 * dt)
            # pose = forward_kinematics(wheels[0], wheels[1], 4, (pos[0], pos[1], angle), dt * i)
            # pos = pose[0:2]
            # angle = pose[2]

            circles[0].center = self.pos[0], self.pos[1]
            self.robot_pose = (self.pos[0], self.pos[1], self.angle)
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


def main():
    controller = PurePursuitController([(0, 0), (10, 30), (20, 40), (30, 30), (40, 60), (20, 80), (70, 70), (100, 13)],
                                       12, 5, 15, 8, (0, 0, 0))
    controller.simulate(0.1)


main()
