from lunabot_behavior.states.traverse import Traverse
from lunabot_behavior.state import Events
from geometry_msgs.msg import PoseStamped

class ManualTraverse(Traverse):
  def __init__(self, is_main: bool=True):
    super().__init__(PoseStamped(), is_main)

  def setup(self, manager):
    super().setup(manager)
    self.goal_sub = manager.create_subscription(PoseStamped, "goal", self.goal_cb, 10)
    self.has_goal = False
    self.started_traverse = False

  def goal_cb(self, goal):
    self.goal = goal
    self.has_goal = True

  def start(self):
    self.logger.info("Press goal pose")
    self.has_goal = False
    self.started_traverse = False

  def periodic(self) -> None | Events:
    if self.started_traverse:
      return super().periodic()
    elif self.has_goal:
      super().start()
      self.started_traverse = True

  def exit(self, event):
    super().exit(event)
