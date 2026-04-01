#!/usr/bin/env python3

from enum import Enum
import sys

from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event

from lunabot_behavior.states.approach_trench import ApproachTrench, RetreatTrench
from lunabot_behavior.states.align_trench import AlignTrench
from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.plunge import Plunge
from lunabot_behavior.states.raise_act import Raise
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.traverse_to_linkup import TraverseToLinkup
from lunabot_behavior.states.trench import Trench
from lunabot_behavior.states.align_to_linkup import AlignToLinkup
from lunabot_behavior.states.main_wait_for_mini_align import MainWaitForAlignState
from lunabot_behavior.states.align_to_mini_bot import AlignToMiniBotState
from lunabot_behavior.states.wait_for_approach import WaitForApproachState
from lunabot_behavior.states.wait_for_diverge import WaitForDivergeState

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy
import math

class MainStates(Enum):
    
    INIT = State()
    INIT_STALL = State()
    
    STARTING_PLUNGE = Plunge()
    STARTING_PLUNGE_STALL = State()
    
    STARTING_RAISE = Raise()
    STARTING_RAISE_STALL = State()
    
    WAIT_FOR_LINKUP = State() # This will stay as State(), no logic needed
    
    TRAVERSE_TO_LINKUP = TraverseToLinkup(True, True)
    TRAVERSE_TO_LINKUP_STALL = Stall()
    TRAVERSE_TO_LINKUP_NO_PATH = NoPath()
    
    ALIGN_TO_TRENCH = AlignTrench()
    ALIGN_TO_TRENCH_STALL = State()
    
    APPROACH_TRENCH = ApproachTrench()
    APPROACH_TRENCH_STALL = Stall()
    
    PLUNGE_ACT = Plunge()
    PLUNGE_ACT_STALL = State()
    
    TRENCH = Trench()
    TRENCH_STALL = State()
    
    RAISE_ACT = Raise()
    RAISE_ACT_STALL = State()

    RETREAT_TRENCH = RetreatTrench()
    RETREAT_TRENCH_STALL = Stall()

    ALIGN_TO_LINKUP = AlignToLinkup()
    ALIGN_TO_LINKUP_STALL = State()

    WAIT_FOR_MINI_ALIGN = MainWaitForAlignState()

    ALIGN_TO_MINI = AlignToMiniBotState()

    WAIT_FOR_APPROACH = WaitForApproachState()

    WAIT_FOR_DIVERGE = WaitForDivergeState()

    TRAVERSE_TO_BERM = TraverseToBerm(True)
    TRAVERSE_TO_BERM_STALL = Stall()
    TRAVERSE_TO_BERM_NO_PATH = NoPath()

    ALIGN_TO_BERM = AlignToAngle(270)
    ALIGN_TO_BERM_STALL = Stall()
    
    APPROACH_BERM = ApproachBerm()
    APPROACH_BERM_STALL = Stall()
    
    DEPOSIT = Deposit(transfer=True)
    DEPOSIT_STALL = State()
    
    DEPOSIT_BERM = Deposit()
    DEPOSIT_BERM_STALL = State()
    
    RETREAT_BERM = RetreatBerm()
    RETREAT_BERM_STALL = Stall()

    IDLE = State()

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.INIT, Events.SUCCESS): MainStates.STARTING_PLUNGE,
            (MainStates.INIT, Events.STALL): MainStates.INIT_STALL,
            (MainStates.INIT_STALL, Events.SUCCESS): MainStates.INIT,
            
            (MainStates.STARTING_PLUNGE, Events.SUCCESS): MainStates.STARTING_RAISE,
            (MainStates.STARTING_PLUNGE, Events.STALL): MainStates.STARTING_PLUNGE_STALL,
            (MainStates.STARTING_PLUNGE_STALL, Events.SUCCESS): MainStates.STARTING_PLUNGE,
            
            (MainStates.STARTING_RAISE, Events.SUCCESS): MainStates.WAIT_FOR_LINKUP,
            (MainStates.STARTING_RAISE, Events.STALL): MainStates.STARTING_RAISE_STALL,
            (MainStates.STARTING_RAISE_STALL, Events.SUCCESS): MainStates.STARTING_RAISE,
            
            (MainStates.WAIT_FOR_LINKUP, Events.PROCEED): MainStates.TRAVERSE_TO_LINKUP,
                        
            (MainStates.TRAVERSE_TO_LINKUP, Events.SUCCESS): MainStates.IDLE, # TODO: Go to link up
            (MainStates.TRAVERSE_TO_LINKUP, Events.STALL): MainStates.TRAVERSE_TO_LINKUP_STALL,
            (MainStates.TRAVERSE_TO_LINKUP, Events.NO_PATH): MainStates.TRAVERSE_TO_LINKUP_NO_PATH,
            (MainStates.TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,

            (MainStates.ALIGN_TO_TRENCH, Events.SUCCESS): MainStates.APPROACH_TRENCH,
            (MainStates.ALIGN_TO_TRENCH, Events.STALL): MainStates.ALIGN_TO_TRENCH_STALL,
            (MainStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            
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

            (MainStates.ALIGN_TO_LINKUP, Events.SUCCESS): MainStates.IDLE, # TODO: Go to linkup
            (MainStates.ALIGN_TO_LINKUP, Events.STALL): MainStates.RETREAT_TRENCH_STALL,
            (MainStates.ALIGN_TO_LINKUP_STALL, Events.SUCCESS): MainStates.ALIGN_TO_LINKUP,

            (MainStates.WAIT_FOR_MINI_ALIGN, Events.SUCCESS): MainStates.ALIGN_TO_MINI,

            (MainStates.ALIGN_TO_MINI, Events.SUCCESS): MainStates.WAIT_FOR_APPROACH,

            (MainStates.WAIT_FOR_APPROACH, Events.SUCCESS): MainStates.DEPOSIT,

            (MainStates.DEPOSIT, Events.SUCCESS): MainStates.WAIT_FOR_DIVERGE,
            (MainStates.DEPOSIT, Events.STALL): MainStates.DEPOSIT_STALL,
            (MainStates.DEPOSIT_STALL, Events.SUCCESS): MainStates.DEPOSIT,
            
            (MainStates.WAIT_FOR_DIVERGE, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,

            # in case minibot is indisposed and big bot has to make full cycles
            (MainStates.TRAVERSE_TO_BERM, Events.ARRIVED): MainStates.ALIGN_TO_BERM,
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
    rclpy.init(args=sys.argv)

    manager = StateManager(MainStates, MainStates.WAIT_FOR_MINI_ALIGN, Events, Event)

    rclpy.spin(manager)

    manager.stop_current_state()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
