from robot import Robot
from renderer import Renderer


class Simulation:
    def __init__(
        self,
        robot_length,
        robot_width,
        field_length,
        field_width,
        pixels_per_meter,
        start_x=0,
        start_y=0,
        start_angle=0,
    ):  # All in meters
        self.robot = Robot(robot_width, start_x, start_y, start_angle)
        self.renderer = Renderer(
            robot_length, robot_width, field_length, field_width, pixels_per_meter
        )
        self.simulate = True

    def update(self, left_vel, right_vel, delta_t):
        if self.simulate:
            self.robot.update_pos(left_vel, right_vel, delta_t)
            print(self.robot.x, self.robot.y)
            self.renderer.render(self.robot.x, self.robot.y, self.robot.heading)

    def start(self):
        self.simulate = True

    def stop(self):
        self.simulate = False
