from state import State, Events
from rclpy.node import Node

class Traverse(State):
    def __init__(self):
        pass

    def setup(self, manager: Node):
        self.manager = manager

    def start(self):
        pass
    
    def periodic(self):
        pass
    
    def exit(self):
        pass