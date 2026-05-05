from rclpy.node import Node

from lunabot_behavior.state import Events
from lunabot_behavior.states.trench import Drive

class ApproachTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, False, 0.1)

  def setup(self, manager: Node):
    super().setup(manager)
    self.manager = manager
    self.num_cycles = 7 # cycles before stepping forward
    self.distance_step = 0.5 # meters
    self.counter = 0
    self.stalled = False
  
  def start(self):
    super().start()
    self.start_time = self.manager.get_clock().now()
    self.counter += 1
    if self.counter % self.num_cycles == 0:
      self.target_distance += self.distance_step
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self, event):
    return super().exit(event)

class RetreatTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, True, 0.1)

  def setup(self, manager: Node):
    super().setup(manager)
    self.manager = manager
    self.num_cycles = 7 # cycles before stepping forward
    self.distance_step = 0.5 # meters
    self.counter = 0
    self.target_distance = self.distance_step
  
  def start(self):
    super().start()
    self.start_time = self.manager.get_clock().now()
    self.counter += 1
    if self.counter % self.num_cycles == 0:
      self.target_distance += self.distance_step
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self, event):
    return super().exit(event)
