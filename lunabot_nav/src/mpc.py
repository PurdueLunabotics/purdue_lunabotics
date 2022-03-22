import numpy as np
from conversions import pose_to_array
from global_path_planner import rotate

class MPC:
    def __init__(
        self,
        g,
        k,
        w_linear,
        w_angular,
        w_goal,
        w_line,
        w_occupied,
        horizon_t,
        dt,
        footprint,
        robot_w,
        robot_h,
    ):
        """_summary_

        Args:
            g (_type_): _description_
            k (_type_): _description_
            w_linear (_type_): _description_
            w_angular (_type_): _description_
            w_goal (_type_): _description_
            w_line (_type_): _description_
            w_occupied (_type_): _description_
            horizon_t (_type_): _description_
            dt (_type_): _description_
            footprint (_type_): _description_
            robot_w (_type_): _description_
            robot_h (_type_): _description_
        """
        self.rollout_count = g
        self.top_rollouts = k
        self.T = 1000
        self.w_linear = w_linear
        self.w_angular = w_angular
        self.w_goal = w_goal
        self.w_line = w_line
        self.w_occupied = w_occupied
        self.path = None
        self.goal = None
        self.robot_pos = None  # [x, y, theta]
        self.horizon_t = horizon_t
        self.dt = dt

        self.grid = None
        self.last_map_update = None
        self.resolution = None
        self.origin = None
        self.maxwidth = None
        self.maxheight = None
        self.footprint = footprint
        self.robot_w = robot_w
        self.robot_h = robot_h

    def __check_collision(self, robot_pos):  # TODO check collision
        """
        Checks whether a given configuration is valid. (collides with obstacles)
        """
        pos = robot_pos[0:2] - self.origin[0:2]
        translated_robot = self.footprint + pos  # maps to origin in 2d
        # robot bounding box = [x_min,y_min, x_max,y_max]
        r_bnds = np.hstack(
            (np.min(translated_robot, axis=0), np.max(translated_robot, axis=0))
        )

        occupied_i = np.array(np.nonzero(self.grid > 0.5))  # flat coords of obstacles
        occupied_i = np.vstack(
            (occupied_i / self.robot_w, occupied_i % self.robot_w)
        )  # xy coords
        occupied_i *= self.resolution  # maps to real-world coordinates
        occupied_i -= self.origin[0:2].reshape(2, 1)  # maps to map frame in 2d
        occupied_i = rotate(
            occupied_i, -robot_pos[2], pos.reshape(2, 1)
        )  # rotate points onto robot to check collisions at angle

        is_colliding = np.any(
            (occupied_i >= r_bnds[:2].reshape(2, 1))
            & (occupied_i <= r_bnds[2:].reshape(2, 1))
        )  # colliding
        return not is_colliding

    def __find_closest_distance(self, robot_pos):
        path = self.path
        if path is None:
            return -1.0
        p1 = [path[0,0], path[0,1]]
        dist = -1.0
        for i in range(1, len(path)):
            p2 = [path[i,0], path[i,1]]
            # Building parametric between points (0 < t < 1)
            y_vel = p2[1] - p1[1]
            x_vel = p2[0] - p1[0]

            x_pos = p1[0]
            y_pos = p1[1]

            t = (x_vel * (robot_pos[0] - x_pos) + y_vel * (robot_pos[1] - y_pos)) / (
                x_vel**2 + y_vel**2
            )
            if t < 0:
                t = 0
            elif t > 1:
                t = 1

            line_pos = [x_pos + t * x_vel, y_pos + t * y_vel]
            temp_dist = (line_pos[0] - robot_pos[0]) ** 2 + (
                line_pos[1] - robot_pos[1]
            ) ** 2  # Using squared since can just change weight later, saves computation
            if dist == -1 or temp_dist < dist:
                dist = temp_dist
        return dist

    def update_grid(self, grid_msg):  # Takes in Occupancy Grid msg
        self.last_map_update = grid_msg.header.stamp
        self.grid = np.array(grid_msg.data)
        self.resolution = grid_msg.info.resolution  # m/cell
        self.origin, _ = np.array(
            pose_to_array(grid_msg.info.origin)
        )  # (pose, ori = usually zero, so same frame as 'map')
        self.maxwidth = grid_msg.info.width  # m/cell
        self.maxheight = grid_msg.info.height  # m/cell
        self.grid = np.array(grid_msg.data)

    def update_path(self, path):  # Takes in Path msg
        self.path = np.zeros((len(path.poses), 2))
        for i in range(len(path.poses)):
            self.path[i][0] = path.poses[i].pose.position.x
            self.path[i][1] = path.poses[i].pose.position.y

    def update_goal(self, goal):  # Takes in PoseStamped msg
        self.goal = np.array([goal.pose.position.x, goal.pose.position.y])

    def update_robot_pos(self, odom):  # Takes in Odometry msg
        pos, ori = pose_to_array(odom.pose.pose)
        self.robot_pos = np.array([pos[0], pos[1], ori[0]])

    def __calculate_cost(self, rollout):
        cost = 0.0
        for state in rollout:
            cost += self.w_linear * (
                (self.robot_pos[0] - state[0]) ** 2
                + (self.robot_pos[1] - state[1]) ** 2
            )
            cost += self.w_angular * (state[2] - self.robot_pos[2]) ** 2
            cost += self.w_goal * (
                (self.goal[0] - state[0]) ** 2 + (self.goal[1] - state[1]) ** 2
            )

            cost += self.w_line * self.__find_closest_distance(state)
            cost += self.w_occupied * self.__check_collision(state)
        return cost

    def __calculate_model(self, actions):
        positions = np.zeros((self.horizon_t, 3))  # x, y, theta
        current_pos = self.robot_pos.copy()
        A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        for i in range(self.horizon_t):
            # Adding new position
            positions[i] = current_pos

            B = np.array(
                [
                    [np.cos(current_pos[2]) * self.dt, 0],
                    [np.sin(current_pos[2]) * self.dt, 0],
                    [0, self.dt],
                ]
            )  # TODO unsure if this works

            current_pos = A @ current_pos + B @ actions[i]
        return positions

    def calculate_velocity(self):
        if self.robot_pos is None or self.goal is None:  # Cannot calculate w/o these
            return None

        means = np.zeros((self.horizon_t, 2))  # v, omega
        std_devs = np.ones((self.horizon_t, 2))  # v, omega
        counter = 0
        while counter < self.T:
            rollout_count = self.rollout_count
            costs = np.zeros(rollout_count) # cost
            actions = np.zeros((rollout_count,self.horizon_t,2)) #rollout, cost, action
            # Generating random velocities
            random_velocities = np.random.normal(
                means, std_devs, size=(rollout_count,self.horizon_t,2)
            )
            # Generating new states and costs
            for i in range(len(random_velocities)):
                rollout = self.__calculate_model(random_velocities[i])
                costs[i] = self.__calculate_cost(rollout)
                actions[i] = random_velocities[i]

            counter += 100
            # Finding good states
            actions = actions[costs.argsort()]
            good_actions = actions[np.arange(self.top_rollouts)]
            print(good_actions.shape)

            # Calculating new means
            means = np.mean(good_actions,axis=0)
            print(means.shape)
            std_devs = np.std(good_actions,axis=0)
        # Returning velocities
        print(good_actions[0,0,0])
        return [good_actions[0,0,0], good_actions[0,0,1]]
