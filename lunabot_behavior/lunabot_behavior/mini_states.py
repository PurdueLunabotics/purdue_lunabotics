#!/usr/bin/env python3

from enum import Enum
from lunabot_msgs.msg import Event
from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event
import sys

from lunabot_config.led_colors import LedColor

from lunabot_behavior.states.find_linkup import FindLinkup
from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.separate_from_main import SeparateFromMainState
from lunabot_behavior.states.proceed import Proceed
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.find_linkup_secondary import FindLinkupSecondary
from lunabot_behavior.states.align_to_main_bot import AlignToMainBotState
from lunabot_behavior.states.mini_wait_for_main_align import MiniWaitForAlignState
from lunabot_behavior.states.approach_main import ApproachMainState
from lunabot_behavior.states.collect import CollectRegolithState
from lunabot_behavior.states.init import InitRetreat, SetupMap

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

from lunabot_behavior.states.traverse_to_linkup import TraverseToLinkup

class MiniStates(Enum):
    # ===== INIT SECTION (1) =====
    INIT_MAP = (SetupMap(False), (LedColor.GREEN, LedColor.YELLOW))
    INIT_WAIT = (State(), (LedColor.GREEN, LedColor.GREEN))
    INIT_MOVE = (InitRetreat(False), (LedColor.GREEN, LedColor.TEAL))
    INIT_STALL = (State(), (LedColor.GREEN, LedColor.RED))
    
    FIND_LINKUP = (FindLinkup(), (LedColor.GREEN, LedColor.BLUE))
    FIND_LINKUP_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    FIND_LINKUP_NO_PATH = (NoPath(), (LedColor.GREEN, LedColor.ORANGE))
    SEND_FOUND_LINKUP = (Proceed(False), (LedColor.GREEN, LedColor.BLUE))

    FIND_LINKUP_SECONDARY = (FindLinkupSecondary(), (LedColor.GREEN, LedColor.MAGENTA))
    FIND_LINKUP_SECONDARY_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    FIND_LINKUP_SECONDARY_NO_PATH = (NoPath(), (LedColor.GREEN, LedColor.ORANGE))
    
    # ===== LINKUP SECTION (2) =====

    WAIT_FOR_ALIGN_AT_LINKUP = (State(), (LedColor.YELLOW, LedColor.YELLOW))
    
    ALIGN_TO_MAIN = (AlignToMainBotState(), (LedColor.YELLOW, LedColor.YELLOW))
    ALIGN_TO_MAIN_STALL = (State(), (LedColor.YELLOW, LedColor.RED))

    WAIT_FOR_MAIN_ALIGN = (MiniWaitForAlignState(), (LedColor.YELLOW, LedColor.GREEN))
    
    APPROACH_MAIN = (ApproachMainState(),(LedColor.YELLOW, LedColor.TEAL))
    APPROACH_MAIN_STALL = (State(), (LedColor.YELLOW, LedColor.RED))

    COLLECT_REGOLITH = (CollectRegolithState(), (LedColor.YELLOW, LedColor.BLUE))

    SEPARATE_FROM_MAIN = (SeparateFromMainState(), (LedColor.YELLOW, LedColor.MAGENTA))
    SEPARATE_FROM_MAIN_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    # ===== TRAVERSAL SECTION (3) =====

    TRAVERSE_TO_BERM = (TraverseToBerm(False), (LedColor.BLUE, LedColor.YELLOW))
    TRAVERSE_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), (LedColor.BLUE, LedColor.ORANGE))

    ALIGN_TO_BERM = (AlignToAngle(90), (LedColor.BLUE, LedColor.GREEN))
    ALIGN_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    MOVE_TO_STAGING = (TraverseToLinkup(False, True), (LedColor.BLUE, LedColor.TEAL))
    MOVE_TO_STAGING_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    MOVE_TO_STAGING_NO_PATH = (NoPath(), (LedColor.BLUE, LedColor.ORANGE))
    # ===== DEPOSIT SECTION (4) =====
    
    APPROACH_BERM = (ApproachBerm(), (LedColor.MAGENTA, LedColor.YELLOW))
    APPROACH_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))
    
    DEPOSIT = (Deposit(), (LedColor.MAGENTA, LedColor.GREEN))
    DEPOSIT_STALL = (State(), (LedColor.MAGENTA, LedColor.RED))

    RETREAT_BERM = (RetreatBerm(), (LedColor.MAGENTA, LedColor.TEAL))
    RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MiniStates.INIT_MAP, Events.SUCCESS): MiniStates.INIT_WAIT,
            (MiniStates.INIT_WAIT, Events.PROCEED): MiniStates.INIT_MOVE,
            (MiniStates.INIT_MOVE, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.INIT_MOVE, Events.STALL): MiniStates.INIT_STALL,
            (MiniStates.INIT_STALL, Events.SUCCESS): MiniStates.INIT_MOVE,
            
            
            (MiniStates.FIND_LINKUP, Events.SUCCESS): MiniStates.SEND_FOUND_LINKUP,
            (MiniStates.FIND_LINKUP, Events.STALL): MiniStates.FIND_LINKUP_STALL,
            (MiniStates.FIND_LINKUP, Events.NO_PATH): MiniStates.FIND_LINKUP_NO_PATH,
            (MiniStates.FIND_LINKUP_STALL, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.FIND_LINKUP_NO_PATH, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.SEND_FOUND_LINKUP, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,

            (MiniStates.TRAVERSE_TO_BERM, Events.SUCCESS): MiniStates.ALIGN_TO_BERM,
            (MiniStates.TRAVERSE_TO_BERM, Events.STALL): MiniStates.TRAVERSE_TO_BERM_STALL,
            (MiniStates.TRAVERSE_TO_BERM, Events.NO_PATH): MiniStates.TRAVERSE_TO_BERM_NO_PATH,
            (MiniStates.TRAVERSE_TO_BERM_STALL, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            (MiniStates.TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            
            (MiniStates.ALIGN_TO_BERM, Events.SUCCESS): MiniStates.APPROACH_BERM,
            (MiniStates.ALIGN_TO_BERM, Events.STALL): MiniStates.ALIGN_TO_BERM_STALL,
            (MiniStates.ALIGN_TO_BERM_STALL, Events.SUCCESS): MiniStates.ALIGN_TO_BERM,
            
            (MiniStates.APPROACH_BERM, Events.SUCCESS): MiniStates.DEPOSIT,
            (MiniStates.APPROACH_BERM, Events.STALL): MiniStates.APPROACH_BERM_STALL,
            (MiniStates.APPROACH_BERM_STALL, Events.SUCCESS): MiniStates.APPROACH_BERM,
            
            (MiniStates.DEPOSIT, Events.SUCCESS): MiniStates.RETREAT_BERM,
            (MiniStates.DEPOSIT, Events.STALL): MiniStates.DEPOSIT_STALL,
            (MiniStates.DEPOSIT_STALL, Events.SUCCESS): MiniStates.DEPOSIT,
            
            (MiniStates.RETREAT_BERM, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.RETREAT_BERM, Events.STALL): MiniStates.RETREAT_BERM_STALL,
            (MiniStates.RETREAT_BERM_STALL, Events.SUCCESS): MiniStates.RETREAT_BERM,
            
            (MiniStates.MOVE_TO_STAGING, Events.SUCCESS): MiniStates.WAIT_FOR_ALIGN_AT_LINKUP,
            (MiniStates.MOVE_TO_STAGING, Events.STALL): MiniStates.MOVE_TO_STAGING_STALL,
            (MiniStates.MOVE_TO_STAGING, Events.NO_PATH): MiniStates.MOVE_TO_STAGING_NO_PATH,
            (MiniStates.MOVE_TO_STAGING_STALL, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.MOVE_TO_STAGING_NO_PATH, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,

            (MiniStates.WAIT_FOR_ALIGN_AT_LINKUP, Events.PROCEED): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.ALIGN_TO_MAIN, Events.SUCCESS): MiniStates.WAIT_FOR_MAIN_ALIGN,
            (MiniStates.ALIGN_TO_MAIN, Events.STALL): MiniStates.ALIGN_TO_MAIN_STALL,
            (MiniStates.ALIGN_TO_MAIN_STALL, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.WAIT_FOR_MAIN_ALIGN, Events.SUCCESS): MiniStates.APPROACH_MAIN,

            (MiniStates.APPROACH_MAIN, Events.SUCCESS): MiniStates.COLLECT_REGOLITH,
            (MiniStates.APPROACH_MAIN, Events.STALL): MiniStates.APPROACH_MAIN_STALL,
            (MiniStates.APPROACH_MAIN_STALL, Events.SUCCESS): MiniStates.APPROACH_MAIN,
            
            (MiniStates.COLLECT_REGOLITH, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN,

            (MiniStates.SEPARATE_FROM_MAIN, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            (MiniStates.SEPARATE_FROM_MAIN, Events.STALL): MiniStates.SEPARATE_FROM_MAIN_STALL,
            (MiniStates.SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN
        }

        return transitions.get((state, event), None)

def main():
    rclpy.init(args=sys.argv)

    manager = StateManager(MiniStates, MiniStates.INIT_MAP, Events, Event)

    try:
        rclpy.spin(manager)
    finally:
        manager.stop_current_state()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
