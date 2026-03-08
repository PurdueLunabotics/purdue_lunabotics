#!/usr/bin/env python3

from rclpy import Future
from rclpy.node import Node
from std_msgs.msg import UInt8
from enum import Enum
from state import Events, State
from typing import Type
from lunabot_msgs.msg import Event

class StateManager(Node):
    def __init__(self, states: Type[Enum], initial_state: Enum, events: Type[Events], event_type: Type[Event]):
        super().__init__("state_manager")

        for state in states:
            # self.get_logger().info(f"starting state: {state}")
            state.value.setup(self)

        self.get_logger().info("started all states")
        self.states = states
        self.events = events
        self.state = initial_state
        self.stopped = False
        self.event_sub = self.create_subscription(event_type, "events", self.event_cb, 10)
        self.timer = self.create_timer(0.1, self.periodic)
        self.get_logger().info(f"starting at state {self.state}")
        self.state.value.start()

    def event_cb(self, event: UInt8):
        self.process_event(self.events(event.data))

    def process_event(self, event):
        self.get_logger().info(f"got event {event} @ {self.state}")
        next_state = self.states.get_transition(self.state, event)

        if next_state is not None:
            self.get_logger().info(f"switching to state {next_state}")
            self.state.value.exit()
            self.state = next_state
            self.state.value.start()

    def periodic(self):
        if not self.stopped:
            event = self.state.value.periodic()

            if event is not None:
                self.process_event(event)

    def stop_current_state(self):
        self.state.value.exit()
        self.stopped = True
