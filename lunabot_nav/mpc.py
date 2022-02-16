import numpy as np
import torch
from tf.transformations import euler_from_quaternion

class MPC:
    def __init__(self, g, k, w_linear, w_angular, w_goal, w_line, w_occupied, horizon_t):
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

    def __check_collision(self, robot_pos): #TODO check collision
        pass

    def __find_closest_point(self, robot_pos): #TODO find closest point on path
        pass

    def update_grid(self, grid): #Takes in Occupancy Grid msg
        self.grid = np.zeros((grid.info.width, grid.info.height))
        idx = 0
        for i in range(grid.info.width):
            for j in range(grid.info.height):
                grid[i][j] = grid.data[idx]
                idx += 1

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

            line_pos = self.__find_closest_point(robot_pos)
            cost += self.w_line * ((line_pos[0] - robot_pos[0]) ** 2 + (line_pos[1] - robot_pos[1]) ** 2)
            cost += self.w_occupied * self.__check_collision(robot_pos)
        return cost

    def __calculate_model(self, prev_state, input): #TODO implement proper model
        pass

    def calculate_velocity(self):
        if self.robot_pos == None or self.goal == None: #Cannot calculate w/o these
            return None

        means = np.zeros(3, self.horizon_t) #x, y, theta
        std_devs = np.ones(3, self.horizon_t)
        counter = 0
        states = {}
        while(counter < self.current_g):
            random_states = torch.normal(means, std_devs, self.current_g - counter)
            for state in random_states:
                states[state] = self.__calculate_cost(state)
            states = dict(sorted(states.items(), key=lambda item: item[1])) #Sorting my cost
            
            counter += self.k
            #Finding good states
            new_states = {}
            temp_counter = 0
            for key, value in states:
                if temp_counter == counter:
                    break
                temp_counter += 1
                new_states[key] = value
            states = new_states

            means = torch.mean(states.keys())
            std_devs = torch.std(states.keys())

        #TODO calculate velocities to return
