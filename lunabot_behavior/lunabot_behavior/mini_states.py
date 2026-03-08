#!/usr/bin/env python3

from enum import Enum
from state_manager import StateManager
from lunabot_msgs.msg import Event
from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event

from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.separate_main import SeparateMain
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.find_linkup import FindLinkup
from lunabot_behavior.states.align_to_main_bot import AlignToMainBotState

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MiniStates(Enum):
    INIT = State()
    INIT_STALL = State()
    
    FIND_LINKUP = FindLinkup()
    FIND_LINKUP_STALL = Stall()
    FIND_LINKUP_NO_PATH = NoPath()

    TRAVERSE_TO_BERM = TraverseToBerm(False)
    TRAVERSE_TO_BERM_STALL = Stall()
    TRAVERSE_TO_BERM_NO_PATH = NoPath()

    ALIGN_TO_BERM = AlignToAngle(270)
    ALIGN_TO_BERM_STALL = Stall()
    
    APPROACH_BERM = ApproachBerm()
    APPROACH_BERM_STALL = Stall()
    
    RETREAT_BERM = RetreatBerm()
    RETREAT_BERM_STALL = Stall()

    DEPOSIT = Deposit()
    DEPOSIT_STALL = State()

    MOVE_TO_STAGING = Traverse(PoseStamped(), False)
    MOVE_TO_STAGING_STALL = Stall()
    MOVE_TO_STAGING_NO_PATH = NoPath()

    ALIGN_TO_MAIN = AlignToMainBotState()
    ALIGN_TO_MAIN_STALL = State()
    
    APPROACH_MAIN = State()
    APPROACH_MAIN_STALL = State()

    COLLECT_REGOLITH = State()

    SEPARATE_FROM_MAIN = SeparateMain()
    SEPARATE_FROM_MAIN_STALL = Stall()

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
            
            (MiniStates.COLLECT_REGOLITH, Events.PROCEED): MiniStates.SEPARATE_FROM_MAIN,

            (MiniStates.SEPARATE_FROM_MAIN, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            (MiniStates.SEPARATE_FROM_MAIN, Events.STALL): MiniStates.SEPARATE_FROM_MAIN_STALL,
            (MiniStates.SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    manager = StateManager(MiniStates, MiniStates.ALIGN_TO_MAIN, Events, Event)

    rclpy.spin(manager)

    manager.stop_current_state()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
