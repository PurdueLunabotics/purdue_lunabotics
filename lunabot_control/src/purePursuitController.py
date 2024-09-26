import math
import random
from operator import sub, mul

from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
from slipController import SlipController

# TODO: Add case for endpoint lookahead
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
        self.actual_lin_ang_wel = [0, 0]

    def injection(self, spacing):
        """Adds points in between sparse waypoints

        Args:
            spacing (int): distance between points
        """
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

    def smoothing(self, weight_data: float, weight_smooth: float, tolerance: float):
        """Optimization algorithm to smooth waypoints

        Args:
            weight_data (float): Usually 1 - weight_smooth
            weight_smooth (float): How smooth curve should be
            tolerance (float): Usually 0.00001
        """
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
        """Distance along path for each point

        Returns:
            List[float]: Distance along curve of waypoint at index i
        """
        dist = 0
        dists = [0]
        for i in range(1, len(self.path)):
            dist += math.hypot(self.path[i][0] - self.path[i - 1][0], self.path[i][1] - self.path[i - 1][1])
            dists.append(dist)
        return dists

    def target_velocity(self, k: int):
        """Generates target velocity using MAX_VEL and MAX_ACCEL and kinematics to calculate desired velocity based on distance between points


        Args:
            k (int): Between 1-5 based on show slow around turns

        Returns:
            List[float]: Target velocity for waypoint at index i
        """
        curve = self.curvature_p()
        target_vel = [min(self.MAX_VELOCITY, k / (curve[i] + 1e-30)) for i in range(len(curve))]

        target_vel[len(target_vel) - 1] = 0
        for i in range(len(target_vel) - 2, 0, -1):
            distance = math.dist(self.path[i], self.path[i + 1])
            target_vel[i] = min(target_vel[i], math.sqrt(target_vel[i + 1] ** 2 + 2 * self.MAX_ACCELERATION * distance))

        return target_vel

    def target_velocity(self):
        """Generates target velocity using MAX_VEL as constant across all points

        Returns:
            List[float]: Target velocity for waypoint at index i
        """
        return [self.MAX_VELOCITY] * len(self.path)

    def curvature_p(self):
        """Calculate curvature at each waypoint

        Returns:
            List[float]: Curvature of waypoint at index i
        """
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

    def combine_wheel_vels(self, left, right, wheel_base):
        linear = (left + right) / 2
        angular = (right - left) / wheel_base
        return [linear, angular]

    def get_wheel_vels(self, combined_vels, trackwidth):
        return [combined_vels[0] + (combined_vels[1] * trackwidth / 2),
                combined_vels[0] - (combined_vels[1] * trackwidth / 2)]

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
        print("Vec look: ", vec_look)
        robot_vec = (math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2]))
        print("Robot vec: ", robot_vec)
        return self.angle_between_vectors(vec_look, robot_vec)

    # def forward_kinematics(self, wheels, robot_pose, wheelbase, dt):
    #     return (robot_pose[0] + (wheels[0] + wheels[1]) / 2 * dt * math.sin(robot_pose[2] + math.pi/2),
    #             robot_pose[1] + (wheels[0] + wheels[1]) / 2 * dt * math.cos(robot_pose[2] + math.pi/2),
    #             robot_pose[2] + math.atan((wheels[0] - wheels[1]) / wheelbase * dt))

    def forward_kinematics(self, wheels, robot_pose, wheel_base, delta_time):
        if wheels[0] == wheels[1]:
            return (robot_pose[0] + wheels[0] * math.cos(robot_pose[2]) * delta_time,
                    robot_pose[1] + wheels[0] * math.sin(robot_pose[2]) * delta_time, robot_pose[2])
        elif wheels[0] == -wheels[1]:
            return (robot_pose[0], robot_pose[1], robot_pose[2] + (wheels[0] - wheels[1]) / wheel_base * delta_time)
        else:
            r = (wheel_base / 2) * ((wheels[0] + wheels[1]) / (wheels[1] - wheels[0]))
            # print("r", r)
            w = (wheels[1] - wheels[0]) / wheel_base
            # print("w", w)

            ICC = [robot_pose[0] - r * math.sin(robot_pose[2]), robot_pose[1] + r * math.cos(robot_pose[2])]
            # print("ICC", ICC)

            matrix_transform = np.matrix([[math.cos(w * delta_time), -1 * math.sin(w * delta_time), 0],
                                          [math.sin(w * delta_time), math.cos(w * delta_time), 0], [0, 0, 1]])
            matrix_a = np.matrix([[robot_pose[0] - ICC[0]], [robot_pose[1] - ICC[1]], [robot_pose[2]]])
            matrix_b = np.matrix([[ICC[0]], [ICC[1]], [w * delta_time]])

            # print("matrix_transform", matrix_transform)
            # print("matrix_a", matrix_a)
            # print("matrix_b", matrix_b)
            new_pose = matrix_transform * matrix_a + matrix_b
            # print(new_pose)

            return [float(new_pose[0][0].item()), float(new_pose[1][0].item()), float(new_pose[2][0].item())]

    # Takes velocity slippage
    def calc_slip(self, robot_vel, target_vel):
        return self.subtract_vectors(target_vel, robot_vel)

    def subtract_vectors(self, v1, v2):
        return tuple(map(sub, v1, v2))

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between_vectors(self, v1, v2):
        # Implementation of angle_between_vectors method
        # dot = v1[0] * v2[0] + v1[1] * v2[1]
        # mag = magnitude(v1) * magnitude(v2)
        # return math.acos(dot / mag)
        return math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1])

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


    def simulate(self, dt):
        print("Simulating...")
        self.generate_path(6, 0.1, 0.1, 0.00001)

        target_vels = self.target_velocity()
        circle2 = plt.Circle((self.robot_pose[0], self.robot_pose[1]), self.LOOKAHEAD, color='b', fill=False)
        circle1 = plt.Circle((0, 0), 0.1, color='r', fill=True)
        circles = [circle2, circle1]

        vel_controller = SlipController(k_1=0.03, k_2=0.03, kp=0.03, ki=0.0001, kd=0.004, kv=1, ka=0.0002)

        def init():
            ax.add_patch(circle2)
            ax.add_patch(circle1)
            vector = plt.quiver(self.robot_pose[0], self.robot_pose[1], 1, 1, angles='xy', scale_units='xy', scale=1)
            circles.append(vector)
            return circles

        # print(len(path3))
        fig, ax = plt.subplots()
        ax.scatter(*zip(*self.path))

        def animate(i):
            print("Frame: ", i)
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

            target_lin_ang_vel = self.combine_wheel_vels(self.wheels[0], self.wheels[1], 4)
            print("target: ", target_lin_ang_vel)

            slip = self.calc_slip(self.actual_lin_ang_wel, target_lin_ang_vel)
            print("Slip: ", slip)
            heading_error = self.calc_heading_err(look)
            print("Heading err: ", heading_error)

            control_loop_vels = vel_controller.update_vel(slip, heading_error, dt, target_vel=vel, target_accel=self.MAX_ACCELERATION)
            print("Control loop lin/ang: ", control_loop_vels)
            self.wheels = self.get_wheel_vels(control_loop_vels, 4)

            print("control loop R/L: ", self.wheels)

            for i in range(len(self.wheels)):
                self.wheels[i] = max(min(max(min(np.random.normal(0.40, 0.1), 1), 0.1) * self.wheels[i], 1), 0)

            print("Control real R/L: ", self.wheels)

            self.actual_lin_ang_wel = self.combine_wheel_vels(self.wheels[0], self.wheels[1], 4)

            new_pose = self.forward_kinematics(self.wheels, self.robot_pose, 4, dt)
            self.pos = [new_pose[0], new_pose[1]]
            self.angle = new_pose[2]
            # pose = forward_kinematics(wheels[0], wheels[1], 4, (pos[0], pos[1], angle), dt * i)
            # pos = pose[0:2]
            # angle = pose[2]

            circles[0].center = self.pos[0], self.pos[1]
            self.robot_pose = (self.pos[0], self.pos[1], self.angle)
            print("Robot pose: ", self.robot_pose)

            circles[2].set_UVC(5 * math.cos(self.robot_pose[2]), 5 * math.sin(self.robot_pose[2]))
            circles[2].set_offsets((self.robot_pose[0], self.robot_pose[1]))

            print("-" * 24)

            return circles

            # print(str(wheels) + ", " + str(angle))

        ax.set_aspect('equal', adjustable='box')

        anim = animation.FuncAnimation(fig, animate,
                                       init_func=init,
                                       frames=360*10,
                                       interval=20,
                                       blit=True)

        anim.save('animation.mp4', fps=30,
                  extra_args=['-vcodec', 'h264',
                              '-pix_fmt', 'yuv420p'])

        print("Done!")


def main():
    controller = PurePursuitController([(0, 0), (10, 30), (20, 40), (30, 30), (40, 60), (20, 80), (70, 70), (100, 13)],
                                       1.5, 1, 2, 8, (0, 0, math.pi / 2))
    controller.simulate(0.1)

main()
