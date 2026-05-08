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
from lunabot_behavior.states.init import InitRetreat, SetupMap
from lunabot_behavior.states.wait_for_linkup import WaitForLinkup
from lunabot_behavior.states.proceed import Proceed

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy
import math

class MainStates(Enum):
    
    # ===== INIT SECTION (1) =====
    INIT_MAP = (SetupMap(True), (LedColor.GREEN, LedColor.YELLOW))
    INIT_WAIT = (State(), (LedColor.GREEN, LedColor.GREEN)) # This will stay as State(), no logic needed
    INIT_MOVE = (InitRetreat(True, 0.1, 5.0), (LedColor.GREEN, LedColor.TEAL))
    INIT_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    
    WAIT_FOR_MINI_GONE = (MainWaitForMiniGoneState(), (LedColor.GREEN, LedColor.GREEN))

    INIT_RETREAT = (Drive(0.5, True, 0.2, 10.0), (LedColor.GREEN, LedColor.TEAL))
    INIT_RETREAT_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))

    LINKUP_HANDSHAKE = (Handshake(True, "linkup"), (LedColor.GREEN, LedColor.TEAL))

    STARTING_PLUNGE = (Plunge(), (LedColor.GREEN, LedColor.BLUE))
    STARTING_PLUNGE_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    
    STARTING_RAISE = (Raise(), (LedColor.GREEN, LedColor.MAGENTA))
    STARTING_RAISE_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    
    WAIT_FOR_LINKUP = (WaitForLinkup(), (LedColor.GREEN, LedColor.GREEN)) # This will stay as State(), no logic needed.
    
    TRAVERSE_TO_LINKUP = (TraverseToLinkup(True, True), (LedColor.GREEN, LedColor.WHITE))
    TRAVERSE_TO_LINKUP_STALL =  (Stall(), (LedColor.GREEN, LedColor.RED))
    TRAVERSE_TO_LINKUP_NO_PATH = (NoPath(), (LedColor.GREEN, LedColor.ORANGE))
    
    # ===== EXCAVATION SECTION (5) =====
    
    ALIGN_TO_TRENCH = (AlignTrench(), (LedColor.WHITE, LedColor.YELLOW))
    ALIGN_TO_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    JUST_PLUNGE = (Plunge(), (LedColor.WHITE, LedColor.TEAL))
    JUST_PLUNGE_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    APPROACH_TRENCH = (ApproachTrench(), (LedColor.WHITE, LedColor.GREEN))
    APPROACH_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    PLUNGE_ACT = (Plunge(), (LedColor.WHITE, LedColor.TEAL))
    PLUNGE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    TRENCH = (Trench(), (LedColor.WHITE, LedColor.BLUE))
    TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))
    
    RAISE_ACT = (Raise(), (LedColor.WHITE, LedColor.MAGENTA))
    RAISE_ACT_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

    RETREAT_TRENCH = (RetreatTrench(), (LedColor.WHITE, LedColor.WHITE))
    RETREAT_TRENCH_STALL = (Stall(), (LedColor.WHITE, LedColor.RED))

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

    # ===== SINGLE ROBOT TRAVERSAL SECTION (3) =====

    TRAVERSE_TO_BERM = (TraverseToBerm(True), (LedColor.BLUE, LedColor.YELLOW))
    TRAVERSE_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), (LedColor.BLUE, LedColor.ORANGE))

    ALIGN_TO_BERM = (AlignToBerm(True), (LedColor.BLUE, LedColor.GREEN))
    ALIGN_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    APPROACH_BERM = (ApproachBerm(True), (LedColor.BLUE, LedColor.TEAL))
    APPROACH_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    # ===== SINGLE ROBOT DEPOSIT SECTION (4) =====
    
    DEPOSIT_BERM = (Deposit(), (LedColor.MAGENTA, LedColor.YELLOW))
    DEPOSIT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))
    
    RETREAT_BERM = (RetreatBerm(False), (LedColor.MAGENTA, LedColor.GREEN))
    RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))

    IDLE = (State(), (LedColor.RED, LedColor.RED))

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.INIT_MAP, Events.SUCCESS): MainStates.INIT_WAIT,
            (MainStates.INIT_WAIT, Events.PROCEED): MainStates.INIT_MOVE,
            
            (MainStates.INIT_MOVE, Events.SUCCESS_AND_DONT_MINE): MainStates.WAIT_FOR_MINI_GONE,
            (MainStates.INIT_MOVE, Events.SUCCESS): MainStates.STARTING_PLUNGE,
            (MainStates.INIT_MOVE, Events.STALL): MainStates.INIT_STALL,
            (MainStates.INIT_STALL, Events.SUCCESS): MainStates.INIT_MOVE,
            
            (MainStates.STARTING_PLUNGE, Events.SUCCESS): MainStates.STARTING_RAISE,
            (MainStates.STARTING_PLUNGE, Events.STALL): MainStates.STARTING_PLUNGE_STALL,
            (MainStates.STARTING_PLUNGE_STALL, Events.SUCCESS): MainStates.STARTING_PLUNGE,

            (MainStates.WAIT_FOR_MINI_GONE, Events.SUCCESS): MainStates.INIT_RETREAT,

            (MainStates.INIT_RETREAT, Events.SUCCESS): MainStates.STARTING_PLUNGE,
            (MainStates.INIT_RETREAT, Events.STALL): MainStates.INIT_RETREAT_STALL,
            (MainStates.INIT_RETREAT_STALL, Events.SUCCESS): MainStates.INIT_RETREAT,

            (MainStates.STARTING_RAISE, Events.SUCCESS): MainStates.LINKUP_HANDSHAKE,
            (MainStates.STARTING_RAISE, Events.STALL): MainStates.STARTING_RAISE_STALL,
            (MainStates.STARTING_RAISE_STALL, Events.SUCCESS): MainStates.STARTING_RAISE,

            (MainStates.LINKUP_HANDSHAKE, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
                        
            (MainStates.TRAVERSE_TO_LINKUP, Events.SUCCESS): MainStates.ALIGN_TO_LINKUP, # TODO: Go to link up
            (MainStates.TRAVERSE_TO_LINKUP, Events.STALL): MainStates.TRAVERSE_TO_LINKUP_STALL,
            (MainStates.TRAVERSE_TO_LINKUP, Events.NO_PATH): MainStates.TRAVERSE_TO_LINKUP_NO_PATH,
            (MainStates.TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,

            (MainStates.ALIGN_TO_TRENCH, Events.SUCCESS): MainStates.APPROACH_TRENCH,
            (MainStates.ALIGN_TO_TRENCH, Events.NO_PATH): MainStates.JUST_PLUNGE,
            (MainStates.ALIGN_TO_TRENCH, Events.STALL): MainStates.ALIGN_TO_TRENCH_STALL,
            (MainStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,

            (MainStates.JUST_PLUNGE, Events.STALL): MainStates.JUST_PLUNGE_STALL,
            (MainStates.JUST_PLUNGE, Events.SUCCESS): MainStates.ALIGN_TO_LINKUP,
            (MainStates.JUST_PLUNGE_STALL, Events.SUCCESS): MainStates.JUST_PLUNGE,
            
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

            (MainStates.RETREAT_TRENCH, Events.SUCCESS): MainStates.ALIGN_TO_LINKUP,
            (MainStates.RETREAT_TRENCH, Events.STALL): MainStates.RETREAT_TRENCH_STALL,
            (MainStates.RETREAT_TRENCH_STALL, Events.SUCCESS): MainStates.RETREAT_TRENCH,

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
            
            (MainStates.SEPARATE_FROM_MINI, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            (MainStates.SEPARATE_FROM_MINI, Events.STALL): MainStates.SEPARATE_FROM_MINI_STALL,
            (MainStates.SEPARATE_FROM_MINI_STALL, Events.SUCCESS): MainStates.SEPARATE_FROM_MINI,

            # in case minibot is indisposed and big bot has to make full cycles
            (MainStates.TRAVERSE_TO_BERM, Events.SUCCESS): MainStates.ALIGN_TO_BERM,
            (MainStates.TRAVERSE_TO_BERM, Events.STALL): MainStates.TRAVERSE_TO_BERM_STALL,
            (MainStates.TRAVERSE_TO_BERM, Events.NO_PATH): MainStates.TRAVERSE_TO_BERM_NO_PATH,
            (MainStates.TRAVERSE_TO_BERM_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_BERM,
            (MainStates.TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_BERM,

            (MainStates.ALIGN_TO_BERM, Events.SUCCESS): MainStates.APPROACH_BERM,
            (MainStates.APPROACH_BERM, Events.SUCCESS): MainStates.DEPOSIT_BERM,
            
            (MainStates.DEPOSIT_BERM, Events.SUCCESS): MainStates.RETREAT_BERM,
            (MainStates.DEPOSIT_BERM, Events.STALL): MainStates.DEPOSIT_BERM_STALL,
            (MainStates.DEPOSIT_BERM_STALL, Events.SUCCESS): MainStates.DEPOSIT_BERM,
            
            (MainStates.RETREAT_BERM, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.RETREAT_BERM, Events.STALL): MainStates.RETREAT_BERM_STALL,
            (MainStates.RETREAT_BERM_STALL, Events.SUCCESS): MainStates.RETREAT_BERM,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    manager = StateManager(MainStates, MainStates.INIT_MAP, Events, Event)

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.stop_current_state()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
