#!/usr/bin/env python3

from enum import Enum
from approach_berm_state import ApproachBerm
from deposit_state import Deposit
from align_state import AlignToMainBotState
from separate_main_state import SeparateMain
from retreat_berm_state import RetreatBerm
from state import Events, State
import rclpy
from state_manager import StateManager
from lunabot_msgs.msg import Event
from align_to_angle_state import AlignToAngleState

class MiniStates(Enum):
    INIT = State()
    INIT_STALL = State()
    
    FIND_LINKUP = State()
    FIND_LINKUP_STALL = State()
    FIND_LINKUP_NO_PATH = State()

    TRAVERSE_TO_BERM = State()
    TRAVERSE_TO_BERM_STALL = State()
    TRAVERSE_TO_BERM_NO_PATH = State()

    ALIGN_TO_BERM = AlignToAngleState(180)
    ALIGN_TO_BERM_STALL = State()
    
    APPROACH_BERM = ApproachBerm()
    APPROACH_BERM_STALL = State()
    
    RETREAT_BERM = RetreatBerm()
    RETREAT_BERM_STALL = State()

    DEPOSIT = Deposit()
    DEPOSIT_STALL = State()

    MOVE_TO_STAGING = State()
    MOVE_TO_STAGING_STALL = State()
    MOVE_TO_STAGING_NO_PATH = State()

    ALIGN_TO_MAIN = AlignToMainBotState()
    ALIGN_TO_MAIN_STALL = State()
    
    APPROACH_MAIN = State()
    APPROACH_MAIN_STALL = State()

    COLLECT_REGOLITH = State()

    SEPARATE_FROM_MAIN = SeparateMain()
    SEPARATE_FROM_MAIN_STALL = State()

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MiniStates.INIT, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.INIT, Events.STALL): MiniStates.INIT_STALL,
            (MiniStates.INIT_STALL, Events.SUCCESS): MiniStates.INIT,
            
            
            (MiniStates.FIND_LINKUP, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.FIND_LINKUP, Events.STALL): MiniStates.FIND_LINKUP_STALL,
            (MiniStates.FIND_LINKUP, Events.NO_PATH): MiniStates.FIND_LINKUP_NO_PATH,
            (MiniStates.FIND_LINKUP_STALL, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.FIND_LINKUP_NO_PATH, Events.SUCCESS): MiniStates.FIND_LINKUP,

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
            
            (MiniStates.MOVE_TO_STAGING, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,
            (MiniStates.MOVE_TO_STAGING, Events.STALL): MiniStates.MOVE_TO_STAGING_STALL,
            (MiniStates.MOVE_TO_STAGING, Events.NO_PATH): MiniStates.MOVE_TO_STAGING_NO_PATH,
            (MiniStates.MOVE_TO_STAGING_STALL, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.MOVE_TO_STAGING_NO_PATH, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,

            (MiniStates.ALIGN_TO_MAIN, Events.SUCCESS): MiniStates.APPROACH_MAIN,
            (MiniStates.ALIGN_TO_MAIN, Events.STALL): MiniStates.ALIGN_TO_MAIN_STALL,
            (MiniStates.ALIGN_TO_MAIN_STALL, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.APPROACH_MAIN, Events.SUCCESS): MiniStates.COLLECT_REGOLITH,
            (MiniStates.APPROACH_MAIN, Events.STALL): MiniStates.APPROACH_MAIN_STALL,
            (MiniStates.APPROACH_MAIN_STALL, Events.SUCCESS): MiniStates.APPROACH_MAIN,
            
            (MiniStates.COLLECT_REGOLITH, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN,

            (MiniStates.SEPARATE_FROM_MAIN, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            (MiniStates.SEPARATE_FROM_MAIN, Events.STALL): MiniStates.SEPARATE_FROM_MAIN_STALL,
            (MiniStates.SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(MiniStates, MiniStates.ALIGN_TO_MAIN, Events, Event)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
