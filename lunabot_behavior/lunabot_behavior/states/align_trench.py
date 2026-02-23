from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.state import Events
from rclpy.node import Node
import numpy as np


class AlignTrench(AlignToAngle):
  def __init__(self, **kwargs):
    super().__init__(0)
    
  def setup(self, manager:Node):
    super().__init__(manager)
    self.min_angle = np.deg2rad(-90)
    self.max_angle = np.deg2rad(90)
    self.angle_step = np.deg2rad(30)

    
  def start(self):
    self.target_angle += self.angle_step
    if self.target_angle > self.max_angle:
      self.target_angle = self.min_angle
    