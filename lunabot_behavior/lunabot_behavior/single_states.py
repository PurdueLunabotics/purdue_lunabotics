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
    
    INIT = State()
    INIT_STALL = State()
    
    FIND_LINKUP = FindLinkup()
    FIND_LINKUP_STALL = Stall()
    FIND_LINKUP_NO_PATH = NoPath()
    
    STARTING_PLUNGE = Plunge()
    STARTING_PLUNGE_STALL = State()
    
    STARTING_RAISE = Raise()
    STARTING_RAISE_STALL = State()
    
    WAIT_FOR_LINKUP = State() # This will stay as State(), no logic needed
    
    TRAVERSE_TO_LINKUP = TraverseToLinkup(True, False)
    TRAVERSE_TO_LINKUP_STALL = Stall()
    TRAVERSE_TO_LINKUP_NO_PATH = NoPath()
    
    TRAVERSE_TO_LINKUP_BACKWARDS = TraverseToLinkup(True, True)
    TRAVERSE_TO_LINKUP_BACKWARDS_STALL = Stall()
    TRAVERSE_TO_LINKUP_BACKWARDS_NO_PATH = NoPath()
    
    ALIGN_TO_TRENCH = AlignTrench()
    ALIGN_TO_TRENCH_STALL = State()
    
    APPROACH_TRENCH = ApproachTrench()
    APPROACH_TRENCH_STALL = State()
    
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

    WAIT_FOR_DIVERGE = State() # This will stay as State(), no logic needed

    TRAVERSE_TO_BERM = TraverseToBerm(True)
    TRAVERSE_TO_BERM_STALL = Stall()
    TRAVERSE_TO_BERM_NO_PATH = NoPath()

    ALIGN_TO_BERM = AlignToAngle(270)
    ALIGN_TO_BERM_STALL = Stall()
    
    APPROACH_BERM = ApproachBerm()
    APPROACH_BERM_STALL = Stall()
    
    DEPOSIT = Deposit(True)
    DEPOSIT_STALL = State()
    
    DEPOSIT_BERM = Deposit()
    DEPOSIT_BERM_STALL = State()
    
    RETREAT_BERM = RetreatBerm()
    RETREAT_BERM_STALL = Stall()

    IDLE = State()

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
                        
            (SingleStates.TRAVERSE_TO_LINKUP, Events.ARRIVED): SingleStates.ALIGN_TO_TRENCH,
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

            (SingleStates.TRAVERSE_TO_BERM, Events.ARRIVED): SingleStates.ALIGN_TO_BERM,
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
            
            (SingleStates.TRAVERSE_TO_LINKUP_BACKWARDS, Events.ARRIVED): SingleStates.ALIGN_TO_TRENCH,
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
