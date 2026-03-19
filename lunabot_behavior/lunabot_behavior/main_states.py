#!/usr/bin/env python3

from enum import Enum

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

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy
import math

class MainStates(Enum):
    
    # ===== INIT SECTION (1) =====
    INIT = (State(), 10)
    INIT_STALL = (State(), 19)
    
    STARTING_PLUNGE = (Plunge(), 11)
    STARTING_PLUNGE_STALL = (State(), 19)
    
    STARTING_RAISE = (Raise(), 12)
    STARTING_RAISE_STALL = (State(), 19)
    
    WAIT_FOR_LINKUP = (State(), 13) # This will stay as State(), no logic needed
    
    TRAVERSE_TO_LINKUP = (TraverseToLinkup(True, True), 14)
    TRAVERSE_TO_LINKUP_STALL = (Stall(), 19)
    TRAVERSE_TO_LINKUP_NO_PATH = (NoPath(), 18)
    
    # ===== EXCAVATION SECTION (5) =====
    
    ALIGN_TO_TRENCH = (AlignTrench(), 50)
    ALIGN_TO_TRENCH_STALL = (State(), 59)
    
    APPROACH_TRENCH = (ApproachTrench(), 51)
    APPROACH_TRENCH_STALL = (Stall(), 59)
    
    PLUNGE_ACT = (Plunge(), 52)
    PLUNGE_ACT_STALL = (State(), 59)
    
    TRENCH = (Trench(), 53)
    TRENCH_STALL = (State(), 59)
    
    RAISE_ACT = (Raise(), 54)
    RAISE_ACT_STALL = (State(), 59)

    RETREAT_TRENCH = (RetreatTrench(), 55)
    RETREAT_TRENCH_STALL = (Stall(), 59)
    
    # ===== LINKUP SECTION (2) =====

    ALIGN_TO_LINKUP = (AlignToLinkup(), 20)
    ALIGN_TO_LINKUP_STALL = (State(), 29)

    DEPOSIT = (Deposit(True), 21)
    DEPOSIT_STALL = (State(), 29)
    
    WAIT_FOR_DIVERGE = (State(), 22) # This will stay as State(), no logic needed

    # ===== SINGLE ROBOT TRAVERSAL SECTION (3) =====

    TRAVERSE_TO_BERM = (TraverseToBerm(True), 30)
    TRAVERSE_TO_BERM_STALL = (Stall(), 39)
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), 38)

    ALIGN_TO_BERM = (AlignToAngle(270), 31)
    ALIGN_TO_BERM_STALL = (Stall(), 39)
    
    APPROACH_BERM = (ApproachBerm(), 32)
    APPROACH_BERM_STALL = (Stall(), 39)
    
    # ===== SINGLE ROBOT DEPOSIT SECTION (4) =====
    
    
    DEPOSIT_BERM = (Deposit(), 40)
    DEPOSIT_BERM_STALL = (State(), 49)
    
    RETREAT_BERM = (RetreatBerm(), 41)
    RETREAT_BERM_STALL = (Stall(), 49)

    IDLE = (State(), 99)

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

            (MainStates.DEPOSIT, Events.SUCCESS): MainStates.WAIT_FOR_DIVERGE,
            (MainStates.DEPOSIT, Events.STALL): MainStates.DEPOSIT_STALL,
            (MainStates.DEPOSIT_STALL, Events.SUCCESS): MainStates.DEPOSIT,
            
            (MainStates.WAIT_FOR_DIVERGE, Events.PROCEED): MainStates.ALIGN_TO_TRENCH,

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
    rclpy.init(args=args)

    manager = StateManager(MainStates, MainStates.PLUNGE_ACT, Events, Event)

    rclpy.spin(manager)

    manager.stop_current_state()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
