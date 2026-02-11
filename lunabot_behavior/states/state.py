#!/usr/bin/env python3

from enum import Enum, auto
from rclpy.node import Node
from abc import ABC, abstractmethod

class Events(Enum):
    # Interal
    SUCCESS = auto()
    FAIL = auto()

    # External
    STALL = auto()
    NO_PATH = auto()
    STUCK = auto()

class State(ABC):
    @abstractmethod
    def setup(self, manager: Node):
        pass
    
    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def periodic(self) -> None | Events:
        return None

    @abstractmethod
    def exit(self):
        pass
