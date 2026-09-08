import rclpy
from lunabot_behavior.state import State, Events
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32


class Wait(State):
  def __init__(self, timeout: float):
    self.timeout = timeout

  def setup(self, manager):
    self.manager = manager

  def start(self):
    self.start_time = self.manager.get_clock().now().nanoseconds / float(1e9)
  
  def periodic(self):
    curr_time = self.manager.get_clock().now().nanoseconds / float(1e9)
    elapsed_time = curr_time - self.start_time
    if elapsed_time > self.timeout:
      return Events.SUCCESS
    
    return None