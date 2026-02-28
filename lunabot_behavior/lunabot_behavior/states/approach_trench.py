from rclpy.node import Node

from lunabot_behavior.state import Events
from lunabot_behavior.states.trench import Drive

class ApproachTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, False)

  def setup(self, manager: Node):
    self.manager = manager
    self.num_cycles = 7 # cycles before stepping forward
    self.distance_step = 0.5 # meters
    self.counter = 0
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
    self.counter += 1
    if self.counter % self.num_cycles == 0:
      self.target_distance += self.distance_step
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self):
    return super().exit()

class RetreatTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, True)

  def setup(self, manager: Node):
    self.manager = manager
    self.num_cycles = 7 # cycles before stepping forward
    self.distance_step = 0.5 # meters
    self.counter = 0
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
    self.counter += 1
    if self.counter % self.num_cycles == 0:
      self.target_distance += self.distance_step
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self):
    return super().exit()
