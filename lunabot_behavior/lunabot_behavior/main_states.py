#!/usr/bin/env python3

from enum import Enum
from state import Events, State
import rclpy
from state_manager import StateManager
from lunabot_msgs.msg import Event

class MainStates(Enum):
    INIT_TO_LINKUP = State()
    INIT_TO_LINKUP_STALL = State()
    INIT_TO_LINKUP_NO_PATH = State()
    
    ALIGN_TO_TRENCH = State()
    ALIGN_TO_TRENCH_STALL = State()
    
    PLUNGE_ACT = State()
    PLUNGE_ACT_STALL = State()
    
    TRENCH = State()
    TRENCH_STALL = State()
    
    RAISE_ACT = State()
    RAISE_ACT_STALL = State()
    
    TRAVERSE_TO_LINKUP = State()
    TRAVERSE_TO_LINKUP_STALL = State()
    TRAVERSE_TO_LINKUP_NO_PATH = State()

    WAIT_FOR_LINKUP = State()

    TRAVERSE_TO_BERM = State()
    TRAVERSE_TO_BERM_STALL = State()
    TRAVERSE_TO_BERM_NO_PATH = State()

    ALIGN_TO_BERM = State()
    ALIGN_TO_BERM_STALL = State()
    
    APPROACH_BERM = State()
    APPROACH_BERM_STALL = State()
    
    DEPOSIT = State()
    DEPOSIT_STALL = State()

    IDLE = State()

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MainStates.INIT_TO_LINKUP, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            (MainStates.INIT_TO_LINKUP, Events.STALL): MainStates.INIT_TO_LINKUP_STALL,
            (MainStates.INIT_TO_LINKUP, Events.NO_PATH): MainStates.INIT_TO_LINKUP_NO_PATH,
            (MainStates.INIT_TO_LINKUP_NO_PATH, Events.SUCCESS): MainStates.INIT_TO_LINKUP,
            (MainStates.INIT_TO_LINKUP_STALL, Events.SUCCESS): MainStates.INIT_TO_LINKUP,

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

            (MainStates.WAIT_FOR_LINKUP, Events.SUCCESS): MainStates.DEPOSIT,
            (MainStates.WAIT_FOR_LINKUP, Events.FAIL): MainStates.TRAVERSE_TO_BERM,

            # in case minibot is indisposed and big bot has to make full cycles
            (MainStates.TRAVERSE_TO_BERM, Events.SUCCESS): MainStates.ALIGN_TO_BERM,
            (MainStates.TRAVERSE_TO_BERM, Events.STALL): MainStates.ALIGN_TO_BERM,
            (MainStates.TRAVERSE_TO_BERM, Events.NO_PATH): MainStates.TRAVERSE_TO_BERM_NO_PATH,
            (MainStates.TRAVERSE_TO_BERM_STALL, Events.SUCCESS): MainStates.TRAVERSE_TO_BERM,
            (MainStates.TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): MainStates.TRAVERSE_TO_BERM,

            (MainStates.ALIGN_TO_BERM, Events.SUCCESS): MainStates.APPROACH_BERM,
            (MainStates.APPROACH_BERM, Events.SUCCESS): MainStates.DEPOSIT,
            
            (MainStates.DEPOSIT, Events.SUCCESS): MainStates.ALIGN_TO_TRENCH,
            (MainStates.DEPOSIT, Events.STALL): MainStates.DEPOSIT_STALL,
            (MainStates.DEPOSIT_STALL, Events.SUCCESS): MainStates.DEPOSIT
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(MainStates, MainStates.INIT_TO_LINKUP, Events, Event)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
