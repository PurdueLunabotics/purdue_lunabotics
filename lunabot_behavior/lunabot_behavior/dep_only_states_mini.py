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

class MiniDepStates(Enum):    
    APPROACH_BERM = (Drive(1000000, False, 0.1, timeout=10), (LedColor.BLUE, LedColor.TEAL))
    APPROACH_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    # ===== SINGLE ROBOT DEPOSIT SECTION (4) =====
    
    DEPOSIT_BERM = (Deposit(is_main=False), (LedColor.MAGENTA, LedColor.YELLOW))
    DEPOSIT_BERM_STALL = (State(), (LedColor.MAGENTA, LedColor.RED))
    
    RETREAT_BERM = (RetreatBerm(), (LedColor.MAGENTA, LedColor.GREEN))
    RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))

    IDLE = (State(), (LedColor.RED, LedColor.RED))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    STOP_END = (Stop(), (LedColor.RED, LedColor.GREEN))

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MiniDepStates.STOP, Events.SUCCESS): MiniDepStates.APPROACH_BERM,

            (MiniDepStates.APPROACH_BERM, Events.SUCCESS): MiniDepStates.DEPOSIT_BERM,
            
            (MiniDepStates.DEPOSIT_BERM, Events.SUCCESS): MiniDepStates.RETREAT_BERM,
            (MiniDepStates.DEPOSIT_BERM, Events.STALL): MiniDepStates.DEPOSIT_BERM_STALL,
            (MiniDepStates.DEPOSIT_BERM_STALL, Events.SUCCESS): MiniDepStates.DEPOSIT_BERM,
            
            (MiniDepStates.RETREAT_BERM, Events.SUCCESS): MiniDepStates.STOP_END,
            (MiniDepStates.RETREAT_BERM, Events.STALL): MiniDepStates.RETREAT_BERM_STALL,
            (MiniDepStates.RETREAT_BERM_STALL, Events.SUCCESS): MiniDepStates.RETREAT_BERM,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MiniDepStates, MiniDepStates.STOP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
