import pygame
import math
from threading import Thread

class Renderer:

    def __init__(self, robot_length, robot_width, field_length, field_width, pixels_per_meter, robot_color=(128, 128, 128), background_color=(255, 255, 255)):
        self.robot_length = round(robot_length * pixels_per_meter)
        self.robot_width = round(robot_width * pixels_per_meter)
        pygame.display.init()
        self.screen = pygame.display.set_mode((field_length * pixels_per_meter, field_width * pixels_per_meter))
        self.pixels_per_meter = pixels_per_meter
        self.ROBOT_COLOR = robot_color
        self.BACKGROUND_COLOR = background_color

        self.thread = None
    
    def render(self, robot_x, robot_y, angle):

        if(self.thread != None and self.thread.is_alive()):
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        robot_x = round(robot_x * self.pixels_per_meter)
        robot_y = round(robot_y * self.pixels_per_meter)
        
        self.screen.fill(self.BACKGROUND_COLOR)

        top_left = (-self.robot_width / 2, -self.robot_length / 2)
        bottom_left = (-self.robot_width / 2, -self.robot_length / 2)
        top_right = (-self.robot_width / 2, -self.robot_length / 2)
        bottom_right = (-self.robot_width / 2, -self.robot_length / 2)

        sin = math.sin(angle)
        cos = math.cos(angle)

        rotated_top_left = (robot_x + top_left[0] * cos - top_left[1] * sin, robot_y + top_left[0] * sin + top_left[1] * cos)
        rotated_bottom_left = (robot_x + bottom_left[0] * cos - bottom_left[1] * sin, robot_y + bottom_left[0] * sin + bottom_left[1] * cos)
        rotated_top_right = (robot_x + top_right[0] * cos - top_right[1] * sin, robot_y + top_right[0] * sin + top_right[1] * cos)
        rotated_bottom_right = (robot_x + bottom_right[0] * cos - bottom_right[1] * sin, robot_y + bottom_right[0] * sin + bottom_right[1] * cos)
        
        points = [rotated_top_left, rotated_bottom_left, rotated_bottom_right, rotated_top_right]
        pygame.draw.polygon(self.screen, self.ROBOT_COLOR, points)

        self.thread = Thread(target=pygame.display.flip)
        self.thread.start()