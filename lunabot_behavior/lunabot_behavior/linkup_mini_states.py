#!/usr/bin/env python3

from enum import Enum
from lunabot_behavior.states.fullstop import Stop
from lunabot_msgs.msg import Event
from lunabot_msgs.msg import Event
import sys

from lunabot_config.led_colors import LedColor


from lunabot_behavior.states.mini_wait_for_first_main_align import MiniWaitForFirstAlignState
from lunabot_behavior.states.align_to_main_bot import AlignToMainBotState
from lunabot_behavior.states.mini_wait_for_main_align import MiniWaitForAlignState
from lunabot_behavior.states.wait_for_main_approach import WaitForMainApproachState
from lunabot_behavior.states.collect import CollectRegolithState
from lunabot_behavior.states.wait_for_main_diverge import WaitForMainDivergeState

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MiniStates(Enum):
    
    # ===== LINKUP SECTION (2) =====
    
    ALIGN_TO_MAIN = (AlignToMainBotState(), (LedColor.YELLOW, LedColor.GREEN))
    ALIGN_TO_MAIN_STALL = (State(), (LedColor.YELLOW, LedColor.RED))

    WAIT_FOR_MAIN_ALIGN = (MiniWaitForAlignState(), (LedColor.YELLOW, LedColor.TEAL))
    
    WAIT_FOR_MAIN_APPROACH = (WaitForMainApproachState(),(LedColor.YELLOW, LedColor.BLUE))

    COLLECT_REGOLITH = (CollectRegolithState(), (LedColor.YELLOW, LedColor.MAGENTA))

    WAIT_FOR_DIVERGE = (WaitForMainDivergeState(), (LedColor.YELLOW, LedColor.WHITE))

    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    STOP_END = (Stop(), (LedColor.RED, LedColor.GREEN))
    
    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MiniStates.STOP, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.ALIGN_TO_MAIN, Events.SUCCESS): MiniStates.WAIT_FOR_MAIN_ALIGN,
            (MiniStates.ALIGN_TO_MAIN, Events.STALL): MiniStates.ALIGN_TO_MAIN_STALL,
            (MiniStates.ALIGN_TO_MAIN_STALL, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.WAIT_FOR_MAIN_ALIGN, Events.SUCCESS): MiniStates.WAIT_FOR_MAIN_APPROACH,

            (MiniStates.WAIT_FOR_MAIN_APPROACH, Events.SUCCESS): MiniStates.COLLECT_REGOLITH,
            (MiniStates.WAIT_FOR_MAIN_APPROACH, Events.NEED_REALIGN): MiniStates.ALIGN_TO_MAIN,
            
            (MiniStates.COLLECT_REGOLITH, Events.SUCCESS): MiniStates.WAIT_FOR_DIVERGE,

            (MiniStates.WAIT_FOR_DIVERGE, Events.SUCCESS): MiniStates.STOP_END,
        }

        return transitions.get((state, event), None)

def main():
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MiniStates, MiniStates.STOP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
