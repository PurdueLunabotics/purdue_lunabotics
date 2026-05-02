from rclpy.node import Node

from lunabot_behavior.state import State, Events

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SeparateFromMiniState(State):
  """
  Main diverging from mini bot after collecting regolith
  """

  def __init__(self):
    self.MOVE_TIME = 5 #seconds
    self.LINEAR_SPEED = 0.1 #m/s

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/cmd_vel", 10)
    self.diverged_msg_publisher = manager.create_publisher(Bool, "/behavior/main_diverged", 10)

    self.manager = manager

  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    elapsed_time = self.manager.get_clock().now() - self.start_time
    elapsed_time = elapsed_time.nanoseconds / 1_000_000_000

    output = Twist()
    output.linear.x = self.LINEAR_SPEED
    self.cmd_vel_publisher.publish(output)

    if (elapsed_time > self.MOVE_TIME):
      return Events.SUCCESS
    return None  
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())

    msg = Bool()
    msg.data = True
    for i in range(10):
      self.diverged_msg_publisher.publish(msg)
