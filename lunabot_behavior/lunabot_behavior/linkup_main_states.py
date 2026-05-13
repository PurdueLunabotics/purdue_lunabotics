#!/usr/bin/env python3

from enum import Enum
import sys

from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.handshake import Handshake
from lunabot_behavior.states.align_to_berm import AlignToBerm
from lunabot_behavior.states.approach_trench import ApproachTrench, RetreatTrench
from lunabot_behavior.states.align_trench import AlignTrench
from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.main_wait_for_mini_gone import MainWaitForMiniGoneState
from lunabot_behavior.states.plunge import Plunge
from lunabot_behavior.states.raise_act import Raise
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.traverse_to_linkup import TraverseToLinkup
from lunabot_behavior.states.trench import Trench, Drive
from lunabot_behavior.states.align_to_linkup import AlignToLinkup
from lunabot_behavior.states.main_wait_for_mini_align import MainWaitForAlignState
from lunabot_behavior.states.align_to_mini_bot import AlignToMiniBotState
from lunabot_behavior.states.wait_for_approach import WaitForApproachState
from lunabot_behavior.states.approach_mini import ApproachMiniState
from lunabot_behavior.states.wait_for_diverge import WaitForDivergeState
from lunabot_behavior.states.separate_from_mini import SeparateFromMiniState
from lunabot_behavior.states.init import InitRetreat, SetupMap, SetupObstacles
from lunabot_behavior.states.wait_for_linkup import WaitForLinkup
from lunabot_behavior.states.proceed import Proceed

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy
import math

class MainStates(Enum):
    
    # ===== LINKUP SECTION (2) =====

    ALIGN_TO_LINKUP = (AlignToLinkup(), (LedColor.YELLOW, LedColor.YELLOW))
    ALIGN_TO_LINKUP_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    WAIT_FOR_MINI_ALIGN = (MainWaitForAlignState(), (LedColor.YELLOW, LedColor.GREEN))

    ALIGN_TO_MINI = (AlignToMiniBotState(), (LedColor.YELLOW, LedColor.TEAL))

    APPROACH_MINI = (ApproachMiniState(), (LedColor.YELLOW, LedColor.BLUE))
    APPROACH_MINI_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    DEPOSIT = (Deposit(transfer=True), (LedColor.YELLOW, LedColor.MAGENTA))
    DEPOSIT_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    SEPARATE_FROM_MINI = (SeparateFromMiniState(), (LedColor.YELLOW, LedColor.WHITE))
    SEPARATE_FROM_MINI_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    IDLE = (State(), (LedColor.RED, LedColor.YELLOW))
    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
  
            (MainStates.ALIGN_TO_LINKUP, Events.SUCCESS): MainStates.WAIT_FOR_MINI_ALIGN,
            (MainStates.ALIGN_TO_LINKUP, Events.STALL): MainStates.RETREAT_TRENCH_STALL,
            (MainStates.ALIGN_TO_LINKUP_STALL, Events.SUCCESS): MainStates.ALIGN_TO_LINKUP,

            (MainStates.WAIT_FOR_MINI_ALIGN, Events.SUCCESS): MainStates.ALIGN_TO_MINI,

            (MainStates.ALIGN_TO_MINI, Events.SUCCESS): MainStates.APPROACH_MINI,

            (MainStates.APPROACH_MINI, Events.SUCCESS): MainStates.DEPOSIT,
            (MainStates.APPROACH_MINI, Events.STALL): MainStates.APPROACH_MINI_STALL,
            (MainStates.APPROACH_MINI, Events.NEED_REALIGN): MainStates.WAIT_FOR_MINI_ALIGN,
            (MainStates.APPROACH_MINI_STALL, Events.SUCCESS): MainStates.APPROACH_MINI,

            (MainStates.DEPOSIT, Events.SUCCESS): MainStates.SEPARATE_FROM_MINI,
            (MainStates.DEPOSIT, Events.STALL): MainStates.DEPOSIT_STALL,
            (MainStates.DEPOSIT_STALL, Events.SUCCESS): MainStates.DEPOSIT,
            
            (MainStates.SEPARATE_FROM_MINI, Events.SUCCESS): MainStates.IDLE,
            (MainStates.SEPARATE_FROM_MINI, Events.STALL): MainStates.SEPARATE_FROM_MINI_STALL,
            (MainStates.SEPARATE_FROM_MINI_STALL, Events.SUCCESS): MainStates.SEPARATE_FROM_MINI,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MainStates, MainStates.ALIGN_TO_LINKUP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
