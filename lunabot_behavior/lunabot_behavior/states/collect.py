from rclpy.node import Node

from lunabot_behavior.state import State, Events

from std_msgs.msg import Int32, Bool

class Collect(State):
  def setup(self, manager: Node):
    self.dep_pub = manager.create_publisher(Int32, "deposition", 10)
    self.manager = manager
  
  def start(self):
    self.start_time = self.manager.get_clock().now()
  
  def periodic(self) -> None | Events:
    self.dep_pub.publish(1000)
    return None  
  
  def exit(self):
    self.dep_pub.publish(0)
      