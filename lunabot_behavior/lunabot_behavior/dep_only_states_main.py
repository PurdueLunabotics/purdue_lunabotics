#!/usr/bin/env python3

from enum import Enum
import sys

from lunabot_msgs.msg import Event

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.align_to_berm import AlignToBerm
from lunabot_behavior.states.traverse import Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.fullstop import Stop
from lunabot_behavior.states.trench import Drive

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MainStates(Enum):   
    APPROACH_BERM = (Drive(1000000, True, 0.2, timeout=5), (LedColor.BLUE, LedColor.TEAL))
    APPROACH_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    # ===== SINGLE ROBOT DEPOSIT SECTION (4) =====
    
    DEPOSIT_BERM = (Deposit(), (LedColor.MAGENTA, LedColor.YELLOW))
    DEPOSIT_BERM_STALL = (State(), (LedColor.MAGENTA, LedColor.RED))
    
    RETREAT_BERM = (RetreatBerm(False), (LedColor.MAGENTA, LedColor.GREEN))
    RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))

    IDLE = (State(), (LedColor.RED, LedColor.RED))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    STOP_END = (Stop(), (LedColor.RED, LedColor.GREEN))

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.STOP, Events.SUCCESS): MainStates.APPROACH_BERM,
            
            (MainStates.APPROACH_BERM, Events.SUCCESS): MainStates.DEPOSIT_BERM,
            
            (MainStates.DEPOSIT_BERM, Events.SUCCESS): MainStates.RETREAT_BERM,
            (MainStates.DEPOSIT_BERM, Events.STALL): MainStates.DEPOSIT_BERM_STALL,
            (MainStates.DEPOSIT_BERM_STALL, Events.SUCCESS): MainStates.DEPOSIT_BERM,
            
            (MainStates.RETREAT_BERM, Events.SUCCESS): MainStates.STOP_END,
            (MainStates.RETREAT_BERM, Events.STALL): MainStates.RETREAT_BERM_STALL,
            (MainStates.RETREAT_BERM_STALL, Events.SUCCESS): MainStates.RETREAT_BERM,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MainStates, MainStates.STOP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
