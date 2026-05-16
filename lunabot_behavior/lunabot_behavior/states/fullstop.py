import rclpy
from lunabot_behavior.state import State, Events
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32


class Stop(State):
  def setup(self, manager: Node):
    self.dep_pub = manager.create_publisher(Int32, "deposition", 10)
    self.exc_pub = manager.create_publisher(Int32, "excavate", 10)
    self.cmd_vel_pub = manager.create_publisher(Twist, "cmd_vel", 10)
    self.lin_act_pub = manager.create_publisher(Int32, "lin_act", 10)
    self.traversal = manager.create_publisher(Bool, "traversal/enabled", 10)

  def start(self):
    self.stop()
  
  def periodic(self):
    return Events.SUCCESS

  def stop(self):
    self.dep_pub.publish(Int32(data = 0))
    self.exc_pub.publish(Int32(data = 0))
    self.cmd_vel_pub.publish(Twist())
    self.lin_act_pub.publish(Int32(data = 0))
    self.traversal.publish(Bool(data = False))


if __name__ == "__main__":
  rclpy.init()
  n = Node("stop")
  stop = Stop()
  stop.setup(n)
  stop.stop()