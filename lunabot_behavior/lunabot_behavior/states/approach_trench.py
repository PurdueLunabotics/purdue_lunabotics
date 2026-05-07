from rclpy.node import Node
from rclpy.parameter import Parameter

from lunabot_behavior.state import Events
from lunabot_behavior.states.trench import Drive


INIT_DIST = 0.3

class ApproachTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0.3, False, 0.1)

  def setup(self, manager: Node):
    super().setup(manager)
    self.manager = manager
    self.num_cycles = 7 # cycles before stepping forward
    self.distance_step = 0.5 # meters
    self.counter = 0
    self.stalled = False

    # self.manager.declare_parameter("exc_approach_dist", self.target_distance, Parameter.Type.DOUBLE)
  
  def start(self):
    super().start()
    self.start_time = self.manager.get_clock().now()

    self.target_distance = self.manager.get_parameter("exc_approach_dist").get_parameter_value().double_value
    # self.counter += 1
    # if self.counter % self.num_cycles == 0:
    #   self.target_distance += self.distance_step

    #   exc_approach_dist_param = Parameter("exc_approach_dist", Parameter.Type.DOUBLE, self.target_distance)
    #   self.manager.set_parameters([exc_approach_dist_param])
  
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
