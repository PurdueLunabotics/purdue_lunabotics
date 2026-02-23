#!/usr/bin/env python3

from enum import Enum
from states.traverse_to_berm import TraverseToBerm
from states.traverse import NoPath, Traverse, Stall
from geometry_msgs.msg import PoseStamped
from deposit_state import Deposit
from approach_berm_state import ApproachBerm
from plunge_states import Plunge
from raise_states import Raise
from retreat_berm_state import RetreatBerm
from trench_states import Trench
from state import Events, State
import rclpy
from align_to_angle_state import AlignToAngleState
from state_manager import StateManager
from lunabot_msgs.msg import Event

class MainStates(Enum):
    
    INIT = State()
    INIT_STALL = State()
    
    TRAVERSE_TO_LINKUP = Traverse(PoseStamped(), False)
    TRAVERSE_TO_LINKUP_STALL = Stall()
    TRAVERSE_TO_LINKUP_NO_PATH = NoPath()
    
    ALIGN_TO_TRENCH = State()
    ALIGN_TO_TRENCH_STALL = State()
    
    PLUNGE_ACT = Plunge()
    PLUNGE_ACT_STALL = State()
    
    TRENCH = Trench()
    TRENCH_STALL = State()
    
    RAISE_ACT = Raise()
    RAISE_ACT_STALL = State()

    WAIT_FOR_LINKUP = State()
    
    WAIT_FOR_DIVERGE = State() # This will stay as State(), no logic needed

    TRAVERSE_TO_BERM = TraverseToBerm()
    TRAVERSE_TO_BERM_STALL = Stall()
    TRAVERSE_TO_BERM_NO_PATH = NoPath()

    ALIGN_TO_BERM = AlignToAngleState(180)
    ALIGN_TO_BERM_STALL = State()
    
    APPROACH_BERM = ApproachBerm()
    APPROACH_BERM_STALL = State()
    
    DEPOSIT = Deposit(True)
    DEPOSIT_STALL = State()
    
    DEPOSIT_BERM = Deposit()
    DEPOSIT_BERM_STALL = State()
    
    RETREAT_BERM = RetreatBerm()
    RETREAT_BERM_STALL = State()

    IDLE = State()

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.INIT, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.INIT, Events.STALL): MainStates.INIT_STALL,
            (MainStates.INIT_STALL, Events.SUCCESS): MainStates.INIT,
            
            (MainStates.TRAVERSE_TO_LINKUP, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            (MainStates.TRAVERSE_TO_LINKUP, Events.STALL): MainStates.TRAVERSE_TO_LINKUP_STALL,
            (MainStates.TRAVERSE_TO_LINKUP, Events.NO_PATH): MainStates.TRAVERSE_TO_LINKUP_NO_PATH,
            (MainStates.TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,

            (MainStates.ALIGN_TO_TRENCH, Events.SUCCESS): MainStates.PLUNGE_ACT,
            (MainStates.ALIGN_TO_TRENCH, Events.STALL): MainStates.ALIGN_TO_TRENCH_STALL,
            (MainStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            
            (MainStates.PLUNGE_ACT, Events.SUCCESS): MainStates.TRENCH,
            (MainStates.PLUNGE_ACT, Events.STALL): MainStates.PLUNGE_ACT_STALL,
            (MainStates.PLUNGE_ACT_STALL, Events.SUCCESS): MainStates.PLUNGE_ACT,
            
            (MainStates.TRENCH, Events.SUCCESS): MainStates.RAISE_ACT,
            (MainStates.TRENCH, Events.STALL): MainStates.TRENCH_STALL,
            (MainStates.TRENCH_STALL, Events.SUCCESS): MainStates.TRENCH,
            
            (MainStates.RAISE_ACT, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.RAISE_ACT, Events.STALL): MainStates.RAISE_ACT_STALL,
            (MainStates.RAISE_ACT_STALL, Events.SUCCESS): MainStates.RAISE_ACT,
            
            (MainStates.TRAVERSE_TO_LINKUP, Events.SUCCESS): MainStates.DEPOSIT,
            (MainStates.TRAVERSE_TO_LINKUP, Events.STALL): MainStates.TRAVERSE_TO_LINKUP_STALL,
            (MainStates.TRAVERSE_TO_LINKUP, Events.NO_PATH): MainStates.TRAVERSE_TO_LINKUP_NO_PATH,
            (MainStates.TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,
            (MainStates.TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_LINKUP,

            (MainStates.DEPOSIT, Events.SUCCESS): MainStates.WAIT_FOR_DIVERGE,
            (MainStates.DEPOSIT, Events.STALL): MainStates.DEPOSIT_STALL,
            (MainStates.DEPOSIT_STALL, Events.SUCCESS): MainStates.DEPOSIT,
            
            (MainStates.WAIT_FOR_DIVERGE, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            
            (MainStates.WAIT_FOR_LINKUP, Events.SUCCESS): MainStates.DEPOSIT,
            (MainStates.WAIT_FOR_LINKUP, Events.FAIL): MainStates.TRAVERSE_TO_BERM,

            # in case minibot is indisposed and big bot has to make full cycles
            (MainStates.TRAVERSE_TO_BERM, Events.ARRIVED): MainStates.IDLE,
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
            (MainStates.RETREAT_BERM_STALL, Events.SUCCESS): MainStates.RETREAT_BERM
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(MainStates, MainStates.INIT, Events, Event)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
