#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import UInt8

class StateManager(Node):
    def __init__(self, states, initial_state, events, event_type):
        super().__init__("state_manager")

        for state in states:
            state.value.setup(self)

        self.states = states
        self.events = events
        self.state = initial_state
        self.event_sub = self.create_subscription(event_type, "events", self.event_cb, 10)
        self.timer = self.create_timer(0.1, self.periodic)
        self.get_logger().info(f"starting at state {self.state}")

    def event_cb(self, event: UInt8):
        self.process_event(self.events(event.data))

    def process_event(self, event):
        self.get_logger().info(f"got event {event}")
        next_state = self.states.get_transition(self.state, event)
        self.get_logger().info(f"switching to state {next_state}")

        if next_state is not None:
            self.state.value.exit()
            self.state = next_state
            self.state.value.start()

    def periodic(self):
        event = self.state.value.periodic()

        if event is not None:
            self.process_event(event)
