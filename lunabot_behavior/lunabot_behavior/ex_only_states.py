#!/usr/bin/env python3

from enum import Enum
import sys

from lunabot_msgs.msg import Event

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.plunge import Plunge
from lunabot_behavior.states.raise_act import Raise
from lunabot_behavior.states.fullstop import Stop

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MainStates(Enum):
    
    # ===== INIT SECTION (1) =====
    
    STARTING_PLUNGE = (Plunge(), (LedColor.GREEN, LedColor.BLUE))
    STARTING_PLUNGE_STALL = (State(), (LedColor.GREEN, LedColor.RED))
    
    STARTING_RAISE = (Raise(), (LedColor.GREEN, LedColor.MAGENTA))
    STARTING_RAISE_STALL = (State(), (LedColor.GREEN, LedColor.RED))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    
    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.STARTING_PLUNGE, Events.SUCCESS): MainStates.STARTING_RAISE,
            (MainStates.STARTING_PLUNGE, Events.STALL): MainStates.STARTING_PLUNGE_STALL,
            (MainStates.STARTING_PLUNGE_STALL, Events.SUCCESS): MainStates.STARTING_PLUNGE,
            
            (MainStates.STARTING_RAISE, Events.STALL): MainStates.STARTING_RAISE_STALL,
            (MainStates.STARTING_RAISE_STALL, Events.SUCCESS): MainStates.STARTING_RAISE,
            (MainStates.STARTING_RAISE, Events.SUCCESS): MainStates.STOP,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MainStates, MainStates.STARTING_PLUNGE, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
