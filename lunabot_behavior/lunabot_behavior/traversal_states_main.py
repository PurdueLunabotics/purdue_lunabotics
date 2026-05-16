#!/usr/bin/env python3

from enum import Enum
from lunabot_msgs.msg import Event
from lunabot_msgs.msg import Event
import sys

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.manual_traverse import ManualTraverse
from lunabot_behavior.states.traverse import NoPath, Stall
from lunabot_behavior.states.fullstop import Stop

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class States(Enum):
    INIT = (Stop(), (LedColor.GREEN, LedColor.BLUE))
    TRAVERSE = (ManualTraverse(), (LedColor.GREEN, LedColor.BLUE))
    STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    NO_PATH = (NoPath(), (LedColor.GREEN, LedColor.ORANGE))
    END = (Stop(), (LedColor.GREEN, LedColor.BLUE))

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (States.INIT, Events.SUCCESS): States.TRAVERSE,
            (States.TRAVERSE, Events.STALL): States.STALL,
            (States.TRAVERSE, Events.NO_PATH): States.NO_PATH,
            (States.STALL, Events.SUCCESS): States.TRAVERSE,
            (States.NO_PATH, Events.SUCCESS): States.TRAVERSE,
            (States.TRAVERSE, Events.SUCCESS): States.END,
        }

        return transitions.get((state, event), None)

def main():
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(States, States.INIT, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
