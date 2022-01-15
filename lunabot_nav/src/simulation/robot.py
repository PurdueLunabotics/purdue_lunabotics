import time
import math
class Robot:
    def __init__(self, distance_between_wheels, x, y, heading):
        self.distance_between_wheels = distance_between_wheels
        self.x = x
        self.y = y
        self.heading = heading
    
    def update_pos(self, l_vel, r_vel, delta_t):
        if(l_vel == r_vel):
            dist = l_vel * delta_t
            self.x = self.x + dist * math.cos(self.heading)
            self.y = self.y + dist * math.sin(self.heading)
        else:
            angular_vel = (r_vel - l_vel) / (self.distance_between_wheels)
            radius = (l_vel + r_vel) / (r_vel - l_vel) * self.distance_between_wheels / 2
            curvature_center = [self.x - radius * math.sin(self.heading), self.y + radius * math.cos(self.heading)]
            self.x = math.cos(angular_vel * delta_t) * (self.x - curvature_center[0]) - math.sin(angular_vel * delta_t) * (self.y - curvature_center[1]) + curvature_center[0]
            self.y = math.sin(angular_vel * delta_t) * (self.x - curvature_center[0]) + math.cos(angular_vel * delta_t) * (self.y - curvature_center[1]) + curvature_center[1]
            self.heading += angular_vel * delta_t
    
    def get_pos(self):
        return [self.x, self.y]