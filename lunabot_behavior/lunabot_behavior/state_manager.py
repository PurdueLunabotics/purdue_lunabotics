#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class StateManager(Node):
    def __init__(self, states, initial_state):
        super().__init__("state_manager")

        for state in states:
            state.value.setup(self)

        self.states = states
        self.state = initial_state

    def process_event(self, event):
        next_state = self.states.get_transition(self.state, event)

        if next_state is not None:
            self.state.exit()
            self.state = next_state
            self.state.start()

    def periodic(self):
        event = self.state.value.periodic()

        if event is not None:
            self.process_event(event)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
