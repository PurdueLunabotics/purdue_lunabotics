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

class TrenchStates(Enum):
    
    # ===== EXCAVATION SECTION (1) =====

    PLUNGE_ACT = (Plunge(), (LedColor.WHITE, LedColor.TEAL))
    PLUNGE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    TRENCH = (Trench(), (LedColor.WHITE, LedColor.BLUE))
    TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    RAISE_ACT = (Raise(), (LedColor.WHITE, LedColor.MAGENTA))
    RAISE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    STOP_END = (Stop(), (LedColor.RED, LedColor.GREEN))
    
    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (TrenchStates.STOP, Events.SUCCESS): TrenchStates.PLUNGE_ACT,
            
            (TrenchStates.PLUNGE_ACT, Events.SUCCESS): TrenchStates.TRENCH,
            (TrenchStates.PLUNGE_ACT, Events.STALL): TrenchStates.PLUNGE_ACT_STALL,
            (TrenchStates.PLUNGE_ACT_STALL, Events.SUCCESS): TrenchStates.PLUNGE_ACT,
            
            (TrenchStates.TRENCH, Events.SUCCESS): TrenchStates.RAISE_ACT,
            (TrenchStates.TRENCH, Events.STALL): TrenchStates.TRENCH_STALL,
            (TrenchStates.TRENCH_STALL, Events.SUCCESS): TrenchStates.TRENCH,
            
            (TrenchStates.RAISE_ACT, Events.SUCCESS): TrenchStates.STOP_END,
            (TrenchStates.RAISE_ACT, Events.STALL): TrenchStates.RAISE_ACT_STALL,
            (TrenchStates.RAISE_ACT_STALL, Events.SUCCESS): TrenchStates.RAISE_ACT,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(TrenchStates, TrenchStates.STOP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
