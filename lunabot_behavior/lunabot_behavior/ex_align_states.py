#!/usr/bin/env python3

from enum import Enum
import sys

from lunabot_msgs.msg import Event

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.plunge import Plunge
from lunabot_behavior.states.raise_act import Raise
from lunabot_behavior.states.align_trench import AlignTrench
from lunabot_behavior.states.approach_trench import ApproachTrench, RetreatTrench
from lunabot_behavior.states.trench import Trench
from lunabot_behavior.states.traverse import Stall
from lunabot_behavior.states.fullstop import Stop

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MainStates(Enum):
    
    # ===== EXCAVATION SECTION (1) =====
    
    ALIGN_TO_TRENCH = (AlignTrench(), (LedColor.WHITE, LedColor.YELLOW))
    ALIGN_TO_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    APPROACH_TRENCH = (ApproachTrench(), (LedColor.WHITE, LedColor.GREEN))
    APPROACH_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    JUST_PLUNGE = (Plunge(), (LedColor.WHITE, LedColor.TEAL))
    JUST_PLUNGE_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    PLUNGE_ACT = (Plunge(), (LedColor.WHITE, LedColor.TEAL))
    PLUNGE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    TRENCH = (Trench(), (LedColor.WHITE, LedColor.BLUE))
    TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    RAISE_ACT = (Raise(), (LedColor.WHITE, LedColor.MAGENTA))
    RAISE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    RETREAT_TRENCH = (RetreatTrench(), (LedColor.WHITE, LedColor.WHITE))
    RETREAT_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    
    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.ALIGN_TO_TRENCH, Events.SUCCESS): MainStates.APPROACH_TRENCH,
            (MainStates.ALIGN_TO_TRENCH, Events.STALL): MainStates.ALIGN_TO_TRENCH_STALL,
            (MainStates.ALIGN_TO_TRENCH, Events.NO_PATH): MainStates.JUST_PLUNGE,
            (MainStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,

            # ====================================================================
            # Option 1: safe angle exists
            # ====================================================================
            
            (MainStates.APPROACH_TRENCH, Events.SUCCESS): MainStates.PLUNGE_ACT,
            (MainStates.APPROACH_TRENCH, Events.STALL): MainStates.APPROACH_TRENCH_STALL,
            (MainStates.APPROACH_TRENCH_STALL, Events.SUCCESS): MainStates.APPROACH_TRENCH,
            
            (MainStates.PLUNGE_ACT, Events.SUCCESS): MainStates.TRENCH,
            (MainStates.PLUNGE_ACT, Events.STALL): MainStates.PLUNGE_ACT_STALL,
            (MainStates.PLUNGE_ACT_STALL, Events.SUCCESS): MainStates.PLUNGE_ACT,
            
            (MainStates.TRENCH, Events.SUCCESS): MainStates.RAISE_ACT,
            (MainStates.TRENCH, Events.STALL): MainStates.TRENCH_STALL,
            (MainStates.TRENCH_STALL, Events.SUCCESS): MainStates.TRENCH,
            
            (MainStates.RAISE_ACT, Events.SUCCESS): MainStates.RETREAT_TRENCH,
            (MainStates.RAISE_ACT, Events.STALL): MainStates.RAISE_ACT_STALL,
            (MainStates.RAISE_ACT_STALL, Events.SUCCESS): MainStates.RAISE_ACT,

            (MainStates.RETREAT_TRENCH, Events.SUCCESS): MainStates.STOP,
            (MainStates.RETREAT_TRENCH, Events.STALL): MainStates.RETREAT_TRENCH_STALL,
            (MainStates.RETREAT_TRENCH_STALL, Events.SUCCESS): MainStates.RETREAT_TRENCH,

            # ===================================================================
            # Option 2: in case of no safe angles from align to trench
            # ===================================================================

            (MainStates.JUST_PLUNGE, Events.STALL): MainStates.JUST_PLUNGE_STALL,
            (MainStates.JUST_PLUNGE, Events.SUCCESS): MainStates.STOP,
            (MainStates.JUST_PLUNGE_STALL, Events.SUCCESS): MainStates.JUST_PLUNGE,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MainStates, MainStates.ALIGN_TO_TRENCH, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
