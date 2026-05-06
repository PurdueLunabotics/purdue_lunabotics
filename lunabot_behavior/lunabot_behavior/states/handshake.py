from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lunabot_behavior.state import Events, State
from std_msgs.msg import String

qos_profile = QoSProfile(reliability = ReliabilityPolicy.RELIABLE, depth = 10)

class Handshake(State):
  def __init__(self, is_main: bool, key: str):
    self.key = key
    self.is_main = is_main
    self.running = False
    self.ready = False

  def setup(self, manager: Node):
    if self.is_main:
      self.handshake_pub = manager.create_publisher(String, "/mini/handshake", qos_profile)
      self.handshake_sub = manager.create_subscription(String, "/handshake", self.handshake_cb, qos_profile)
    else:
      self.handshake_pub = manager.create_publisher(String, "/handshake", qos_profile)
      self.handshake_sub = manager.create_subscription(String, "/mini/handshake", self.handshake_cb, qos_profile)

  def handshake_cb(self, incoming_key: String):
    if self.running and incoming_key == self.key:
      self.ready = True

  def periodic(self):
    self.handshake_pub.publish(self.key)

    if self.ready:
      return Events.SUCCESS

  def start(self):
    self.running = True
    self.ready = False

  def exit(self, event):
    self.running = False
