from enum import Enum, auto
from rclpy.node import Node

class Events(Enum):
    # Interal
    SUCCESS = auto()
    FAIL = auto()

    # External
    STALL = auto()
    NO_PATH = auto()
    STUCK = auto()

class State:
    def setup(self, manager: Node):
        pass
    
    def start(self):
        pass

    def periodic(self) -> None | Events:
        return None

    def exit(self):
        pass
