from lunabot_behavior.states.traverse import Traverse
from lunabot_behavior.state import Events
from geometry_msgs.msg import PoseStamped

class ManualTraverse(Traverse):
  def __init__(self):
    super().__init__(None, True)

  def setup(self, manager):
    super().setup(manager)
    self.goal_sub = manager.create_subscription(PoseStamped, "goal", self.goal_cb, 10)
    self.started = False
    self.started_traverse = False

  def goal_cb(self, goal):
    if self.started:
      self.goal = goal

  def start(self):
    self.logger.info("Press goal pose")
    self.started = True

  def periodic(self) -> None | Events:
    if self.started_traverse:
      return super().periodic()
    elif self.goal is not None:
      super().start()
      self.started_traverse = True

  def exit(self, event):
    self.started = False
    super().exit(event)
