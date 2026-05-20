from rclpy.node import Node
from rclpy.parameter import Parameter

from lunabot_behavior.state import Events
from lunabot_behavior.states.trench import Drive


class ApproachTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, False, 0.1, use_pid=True)

  def setup(self, manager: Node):
    super().setup(manager)
    self.manager = manager

  def start(self):
    super().start()
    self.start_time = self.manager.get_clock().now()

    # set in align_trench.py for dynamic approach distances
    self.target_distance = self.manager.get_parameter("exc_approach_dist").get_parameter_value().double_value
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self, event):
    return super().exit(event)

class RetreatTrench(Drive):
  def __init__(self) -> None:
    super().__init__(0, True, 0.1, tolerance=0.1, use_pid=True) # tolerance is sum of approach + trench tolerances (avoids going farther back)

  def setup(self, manager: Node):
    super().setup(manager)
    self.manager = manager
  
  def start(self):
    super().start()
    self.start_time = self.manager.get_clock().now()

    # set in align_trench.py for dynamic retreat distances
    self.target_distance = self.manager.get_parameter("exc_retreat_dist").get_parameter_value().double_value
  
  def periodic(self) -> None | Events:
    return super().periodic()
  
  def exit(self, event):
    return super().exit(event)
