#!/usr/bin/env python3

from enum import Enum

from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event

from lunabot_behavior.states.traverse_to_linkup import TraverseToLinkup
from lunabot_behavior.states.align_to_linkup import AlignToLinkup
from lunabot_behavior.states.find_linkup import FindLinkup
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
from lunabot_behavior.states.trench import Trench

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class SingleStates(Enum):
    
    INIT = (State(), 0)
    INIT_STALL = (State(), 0)
    
    FIND_LINKUP = (FindLinkup(), 0)
    FIND_LINKUP_STALL = (Stall(), 0)
    FIND_LINKUP_NO_PATH = (NoPath(), 0)
    
    STARTING_PLUNGE = (Plunge(), 0)
    STARTING_PLUNGE_STALL = (State(), 0)
    
    STARTING_RAISE = (Raise(), 0)
    STARTING_RAISE_STALL = (State(), 0)
    
    WAIT_FOR_LINKUP = (State(), 0) # This will stay as State(), no logic needed
    
    TRAVERSE_TO_LINKUP = (TraverseToLinkup(True, False), 0)
    TRAVERSE_TO_LINKUP_STALL = (Stall(), 0)
    TRAVERSE_TO_LINKUP_NO_PATH = (NoPath(), 0)
    
    TRAVERSE_TO_LINKUP_BACKWARDS = (TraverseToLinkup(True, True), 0)
    TRAVERSE_TO_LINKUP_BACKWARDS_STALL = (Stall(), 0)
    TRAVERSE_TO_LINKUP_BACKWARDS_NO_PATH = (NoPath(), 0)
    
    ALIGN_TO_TRENCH = (AlignTrench(), 0)
    ALIGN_TO_TRENCH_STALL = (State(), 0)
    
    APPROACH_TRENCH = (ApproachTrench(), 0)
    APPROACH_TRENCH_STALL = (State(), 0)
    
    PLUNGE_ACT = (Plunge(), 0)
    PLUNGE_ACT_STALL = (State(), 0)
    
    TRENCH = (Trench(), 0)
    TRENCH_STALL = (State(), 0)
    
    RAISE_ACT = (Raise(), 0)
    RAISE_ACT_STALL = (State(), 0)
    
    RETREAT_TRENCH = (RetreatTrench(), 0)
    RETREAT_TRENCH_STALL = (Stall(), 0)

    ALIGN_TO_LINKUP = (AlignToLinkup(), 0)
    ALIGN_TO_LINKUP_STALL = (State(), 0)

    WAIT_FOR_DIVERGE = (State(), 0) # This will stay as State(), no logic needed

    TRAVERSE_TO_BERM = (TraverseToBerm(True), 0)
    TRAVERSE_TO_BERM_STALL = (Stall(), 0)
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), 0)

    ALIGN_TO_BERM = (AlignToAngle(270), 0)
    ALIGN_TO_BERM_STALL = (Stall(), 0)
    
    APPROACH_BERM = (ApproachBerm(), 0)
    APPROACH_BERM_STALL = (Stall(), 0)
    
    DEPOSIT = (Deposit(True), 0)
    DEPOSIT_STALL = (State(), 0)
    
    DEPOSIT_BERM = (Deposit(), 0)
    DEPOSIT_BERM_STALL = (State(), 0)
    
    RETREAT_BERM = (RetreatBerm(), 0)
    RETREAT_BERM_STALL = (Stall(), 0)

    IDLE = (State(), 0)

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (SingleStates.INIT, Events.SUCCESS): SingleStates.STARTING_PLUNGE,
            (SingleStates.INIT, Events.STALL): SingleStates.INIT_STALL,
            (SingleStates.INIT_STALL, Events.SUCCESS): SingleStates.INIT,
            
            (SingleStates.STARTING_PLUNGE, Events.SUCCESS): SingleStates.STARTING_RAISE,
            (SingleStates.STARTING_PLUNGE, Events.STALL): SingleStates.STARTING_PLUNGE_STALL,
            (SingleStates.STARTING_PLUNGE_STALL, Events.SUCCESS): SingleStates.STARTING_PLUNGE,
            
            (SingleStates.STARTING_RAISE, Events.SUCCESS): SingleStates.FIND_LINKUP,
            (SingleStates.STARTING_RAISE, Events.STALL): SingleStates.STARTING_RAISE_STALL,
            (SingleStates.STARTING_RAISE_STALL, Events.SUCCESS): SingleStates.STARTING_RAISE,
                        
            (SingleStates.FIND_LINKUP, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP,
            (SingleStates.FIND_LINKUP, Events.STALL): SingleStates.FIND_LINKUP_STALL,
            (SingleStates.FIND_LINKUP, Events.NO_PATH): SingleStates.FIND_LINKUP_NO_PATH,
            (SingleStates.FIND_LINKUP_STALL, Events.SUCCESS): SingleStates.FIND_LINKUP,
            (SingleStates.FIND_LINKUP_NO_PATH, Events.SUCCESS): SingleStates.FIND_LINKUP,
                        
            (SingleStates.TRAVERSE_TO_LINKUP, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,
            (SingleStates.TRAVERSE_TO_LINKUP, Events.STALL): SingleStates.TRAVERSE_TO_LINKUP_STALL,
            (SingleStates.TRAVERSE_TO_LINKUP, Events.NO_PATH): SingleStates.TRAVERSE_TO_LINKUP_NO_PATH,
            (SingleStates.TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP,
            (SingleStates.TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP,

            (SingleStates.ALIGN_TO_TRENCH, Events.SUCCESS): SingleStates.APPROACH_TRENCH,
            (SingleStates.ALIGN_TO_TRENCH, Events.STALL): SingleStates.ALIGN_TO_TRENCH_STALL,
            (SingleStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,
            
            (SingleStates.APPROACH_TRENCH, Events.SUCCESS): SingleStates.PLUNGE_ACT,
            (SingleStates.APPROACH_TRENCH, Events.STALL): SingleStates.APPROACH_TRENCH_STALL,
            (SingleStates.APPROACH_TRENCH_STALL, Events.SUCCESS): SingleStates.APPROACH_TRENCH,
            
            (SingleStates.PLUNGE_ACT, Events.SUCCESS): SingleStates.TRENCH,
            (SingleStates.PLUNGE_ACT, Events.STALL): SingleStates.PLUNGE_ACT_STALL,
            (SingleStates.PLUNGE_ACT_STALL, Events.SUCCESS): SingleStates.PLUNGE_ACT,
            
            (SingleStates.TRENCH, Events.SUCCESS): SingleStates.RAISE_ACT,
            (SingleStates.TRENCH, Events.STALL): SingleStates.TRENCH_STALL,
            (SingleStates.TRENCH_STALL, Events.SUCCESS): SingleStates.TRENCH,
            
            (SingleStates.RAISE_ACT, Events.SUCCESS): SingleStates.RETREAT_TRENCH,
            (SingleStates.RAISE_ACT, Events.STALL): SingleStates.RAISE_ACT_STALL,
            (SingleStates.RAISE_ACT_STALL, Events.SUCCESS): SingleStates.RAISE_ACT,
            
            (SingleStates.RETREAT_TRENCH, Events.SUCCESS): SingleStates.ALIGN_TO_LINKUP,
            (SingleStates.RETREAT_TRENCH, Events.STALL): SingleStates.RETREAT_TRENCH_STALL,
            (SingleStates.RETREAT_TRENCH_STALL, Events.SUCCESS): SingleStates.RETREAT_TRENCH,

            (SingleStates.ALIGN_TO_LINKUP, Events.SUCCESS): SingleStates.TRAVERSE_TO_BERM, # TODO: Go to linkup
            (SingleStates.ALIGN_TO_LINKUP, Events.STALL): SingleStates.RETREAT_TRENCH_STALL,
            (SingleStates.ALIGN_TO_LINKUP_STALL, Events.SUCCESS): SingleStates.ALIGN_TO_LINKUP,

            (SingleStates.TRAVERSE_TO_BERM, Events.SUCCESS): SingleStates.ALIGN_TO_BERM,
            (SingleStates.TRAVERSE_TO_BERM, Events.STALL): SingleStates.TRAVERSE_TO_BERM_STALL,
            (SingleStates.TRAVERSE_TO_BERM, Events.NO_PATH): SingleStates.TRAVERSE_TO_BERM_NO_PATH,
            (SingleStates.TRAVERSE_TO_BERM_STALL, Events.SUCCESS): SingleStates.TRAVERSE_TO_BERM,
            (SingleStates.TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): SingleStates.TRAVERSE_TO_BERM,

            (SingleStates.ALIGN_TO_BERM, Events.SUCCESS): SingleStates.APPROACH_BERM,
            
            (SingleStates.APPROACH_BERM, Events.SUCCESS): SingleStates.DEPOSIT_BERM,
            
            (SingleStates.DEPOSIT_BERM, Events.SUCCESS): SingleStates.RETREAT_BERM,
            (SingleStates.DEPOSIT_BERM, Events.STALL): SingleStates.DEPOSIT_BERM_STALL,
            (SingleStates.DEPOSIT_BERM_STALL, Events.SUCCESS): SingleStates.DEPOSIT_BERM,
            
            (SingleStates.RETREAT_BERM, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP,
            (SingleStates.RETREAT_BERM, Events.STALL): SingleStates.RETREAT_BERM_STALL,
            (SingleStates.RETREAT_BERM_STALL, Events.SUCCESS): SingleStates.RETREAT_BERM,
            
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS, Events.STALL): SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS_STALL,
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS, Events.NO_PATH): SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS_NO_PATH,
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS_STALL, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS,
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS_NO_PATH, Events.SUCCESS): SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(SingleStates, SingleStates.FIND_LINKUP, Events, Event)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
