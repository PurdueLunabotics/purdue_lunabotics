from asyncio.windows_events import NULL
from os import stat
import numpy as np
from tf.transformations import euler_from_quaternion

class MPC:
    def __init__(self, g, k, w_linear, w_angular, w_goal, w_line, w_occupied, horizon_t, dt):
        self.g = g
        self.k = k
        self.w_linear = w_linear
        self.w_angular = w_angular
        self.w_goal = w_goal
        self.w_line = w_line
        self.w_occupied = w_occupied
        self.grid = None
        self.path = None
        self.goal = None
        self.robot_pos = None # [x, y, theta]
        self.horizon_t = horizon_t
        self.dt = dt

    def __check_collision(self, robot_pos): #TODO check collision
        pass

    def __find_closest_distance(self, robot_pos):
        path = self.path
        if path == None:
            return -1.0
        p1 = [path.poses[0].pose.position.x, path.poses[0].pose.position.y]
        dist = -1.0
        for i in range(1, len(path.poses)):
            p2 = [path.poses[i].pose.position.x, path.poses[i].pose.position.y]
            #Building parametric between points (0 < t < 1)
            y_vel = p2[1] - p1[1]
            x_vel = p2[0] - p1[0]

            x_pos = p1[0]
            y_pos = p1[1]

            t = (x_vel * (robot_pos[0] - x_pos) + y_vel * (robot_pos[1] - y_pos)) / (x_vel ** 2 + y_vel ** 2)
            if t < 0:
                t = 0
            elif t > 1:
                t = 1
            
            line_pos = [x_pos + t * x_vel, y_pos + t * y_vel]
            temp_dist = (line_pos[0] - robot_pos[0]) ** 2 + (line_pos[1] - robot_pos[1]) ** 2 #Using squared since can just change weight later, saves computation
            if dist == -1 or temp_dist < dist:
                dist = temp_dist
        return dist

    def update_grid(self, grid_msg): #Takes in Occupancy Grid msg
        self.grid = np.array(grid_msg.data)

    def update_path(self, path): #Takes in Path msg
        self.path = np.zeros((len(path.poses), 2))
        for i in range(len(path.poses)):
            self.path[i][0] = path.poses[i].pose.position.x
            self.path[i][1] = path.poses[i].pose.position.y

    def update_goal(self, goal): #Takes in Pose msg
        self.goal = np.array(goal.position.x, goal.position.y)

    def update_robot_pos(self, odom): #Takes in Odometry msg
        self.robot_pos = np.zeros(3)
        self.robot_pos[0] = odom.pose.pose.position.x
        self.robot_pos[1] = odom.pose.pose.position.y
        _, _, self.robot_pos[2] = euler_from_quaternion(odom.pose.pose.orientation)

    def __calculate_cost(self, state):
        robot_pos = self.robot_pos
        cost = 0.0
        for state_t in state:
            robot_pos = self.__calculate_model(robot_pos, state_t)

            cost += self.w_linear * ((self.robot_pos[0] - robot_pos[0]) ** 2 + (self.robot_pos[1] - robot_pos[1]) ** 2)
            cost += self.w_angular * (robot_pos[2] - self.robot_pos[2]) ** 2
            cost += self.w_goal * ((self.goal[0] - robot_pos[0]) ** 2 + (self.goal[1] - robot_pos[1]) ** 2)

            cost += self.w_line * self.__find_closest_distance(robot_pos)
            cost += self.w_occupied * self.__check_collision(robot_pos)
        return cost

    def __calculate_model(self, state):
        positions = np.zeros(self.horizon_t, 3) #x, y, theta
        current_pos = self.robot_pos
        A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        for i in range(self.horizon_t):
            #Adding new position
            positions[i] = current_pos

            B = np.array([[np.cos(current_pos[2]) * self.dt, 0], [np.sin(current_pos[2]) * self.dt, 0], [0, self.dt]]) #TODO unsure if this works

            current_pos = A @ current_pos + B @ state[i]
        return positions

    def calculate_velocity(self):
        if self.robot_pos == None or self.goal == None: #Cannot calculate w/o these
            return None

        means = np.zeros(self.horizon_t, 2) #v, omega
        std_devs = np.ones(self.horizon_t, 2) #v, omega
        counter = 0
        states = {}
        actions = {}
        while(counter < self.current_g):
            #Generating random velocities
            random_velocities = np.random.normal(means, std_devs, self.current_g - counter)

            #Generating new states and costs
            for i in range(len(random_velocities)):
                calculated_state = self.__calculate_cost(random_velocities[i])
                states[calculated_state] = self.__calculate_cost(calculated_state)
                actions[calculated_state] = random_velocities[i]
            states = dict(sorted(states.items(), key=lambda item: item[1])) #Sorting the costs
            
            counter += self.k
            #Finding good states
            new_states = {}
            new_actions = {}
            temp_counter = 0
            for key, value in states:
                if temp_counter == counter:
                    break
                temp_counter += 1
                new_states[key] = value
                new_actions[key] = actions[key]
            states = new_states
            actions = new_actions

            #Calculating new means
            means = np.mean(new_actions.keys())
            std_devs = np.std(new_actions.keys())

        #Returning velocities
        best_state = list(states.keys())[0]
        return [actions[best_state][0], actions[best_state][1]]
