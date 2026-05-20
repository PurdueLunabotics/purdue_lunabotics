#!/usr/bin/env python3

from rclpy import Future
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8, Int32
from geometry_msgs.msg import PoseStamped, Twist
from enum import Enum
from lunabot_behavior.state import Events
from typing import Type
from lunabot_msgs.msg import Event

from lunabot_config.led_colors import colorsToInteger, LedColor

ODOM_TIME_TOLERANCE = 0.5 # seconds

class StateManager(Node):
    def __init__(self, states: Type[Enum], initial_state: Enum, events: Type[Events], event_type: Type[Event]):
        super().__init__("state_manager")

        for state in states:
            self.get_logger().info(f"starting state: {state}")
            state.value[0].setup(self)

        self.get_logger().info("started all states")
        self.states = states
        self.events = events
        self.state = initial_state
        self.stopped = False
        self.autonomy = True

        self.last_pos_time = self.get_clock().now().nanoseconds / float(1e9)

        self.event_sub = self.create_subscription(event_type, "events", self.event_cb, 10)
        self.autonomy_sub = self.create_subscription(Bool, "autonomy", self.autonomy_cb, 10)
        self.autonomy_pub = self.create_publisher(Bool, "autonomy", 10)
        self.led_pub = self.create_publisher(Int32, "led_color", 10)

        self.odom_pub = self.create_subscription(PoseStamped, "position", self.pos_cb, 10)

        # stop states
        self.traversal_pub = self.create_publisher(Bool, "traversal/enabled", 10)
        self.dep_pub = self.create_publisher(Int32, "deposition", 10)
        self.exc_pub = self.create_publisher(Int32, "excavate", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lin_act_pub = self.create_publisher(Int32, "lin_act", 10)

        self.timer = self.create_timer(0.1, self.periodic)

        self.get_logger().info(f"starting at state {self.state}")
        self.state.value[0].start()
        
        colors = self.state.value[1]
        self.led_pub.publish(Int32(data = colorsToInteger(colors)))
        
        self.autonomy_pub.publish(Bool(data=True))

    def event_cb(self, event: UInt8):
        self.process_event(self.events(event.data))
    
    def autonomy_cb(self, msg: Bool):
        self.autonomy = msg.data
        
    def process_event(self, event):
        self.get_logger().info(f"got event {event} @ {self.state}")
        next_state = self.states.get_transition(self.state, event)

        if next_state is not None:
            self.get_logger().info(f"switching to state {next_state}")
            self.state.value[0].exit(event)
            self.state = next_state
            self.state.value[0].start()

    def pos_cb(self, msg: PoseStamped):
        self.last_pos_time = self.get_clock().now().nanoseconds / float(1e9)

    def stop_everything(self):
        self.traversal_pub.publish(Bool(data = False))
        self.dep_pub.publish(Int32(data = 0))
        self.exc_pub.publish(Int32(data = 0))
        self.cmd_vel_pub.publish(Twist())
        self.lin_act_pub.publish(Int32(data = 0))

    def periodic(self):
        if self.autonomy:
            curr_time = self.get_clock().now().nanoseconds / float(1e9)
            pos_time_diff = curr_time - self.last_pos_time
            if not self.stopped and pos_time_diff < ODOM_TIME_TOLERANCE:
                event = self.state.value[0].periodic()
                colors = self.state.value[1]
                self.led_pub.publish(Int32(data = colorsToInteger(colors)))

                if event is not None:
                    self.process_event(event)
            else:
                self.stop_everything()

    def stop_current_state(self):
        self.get_logger().warn("stopping")
        self.state.value[0].exit(None)
        self.stopped = True
        self.led_pub.publish(Int32(data= colorsToInteger((LedColor.RED, LedColor.GREEN))))
