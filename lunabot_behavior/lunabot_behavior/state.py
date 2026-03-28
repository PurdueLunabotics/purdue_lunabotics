from enum import Enum, auto
from rclpy.node import Node
from rclpy.task import Future
from lunabot_msgs.msg import Event

class Events(Enum):
    # Interal
    SUCCESS = Event.SUCCESS
    FAIL = Event.FAIL

    # External
    STALL = Event.STALL
    NO_PATH = Event.NO_PATH
    STUCK = Event.STUCK
    ARRIVED = Event.ARRIVED
    PROCEED = Event.PROCEED

class State:
    def setup(self, manager: Node) -> Future | None:
        pass
    
    def start(self):
        pass

    def periodic(self) -> None | Events:
        return None

    def exit(self):
        pass
