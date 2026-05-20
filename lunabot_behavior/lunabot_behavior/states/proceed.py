from lunabot_msgs.msg import Event
from lunabot_behavior.state import Events, State
from rclpy.node import Node

class Proceed(State):
    def __init__(self, is_main: bool) -> None:
      self.is_main = is_main

    def setup(self, manager: Node):
        self.event_pub = manager.create_publisher(Event, "/events" if not self.is_main else "/mini/events", 10)

    def start(self):
        self.event_pub.publish(Event(data = Event.PROCEED))

    def periodic(self) -> None | Events:
        return Events.SUCCESS

    def exit(self, event):
        pass
