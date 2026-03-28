from enum import Enum
from lunabot_msgs.msg import Event
from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event

from lunabot_behavior.states.collect import Collect
from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.separate_main import SeparateMain
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.find_linkup import FindLinkup
from lunabot_behavior.states.init import SetupMap

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class MiniStates(Enum):
    # ===== INIT SECTION (1) =====
    INIT = (SetupMap(False), 10)
    INIT_STALL = (State(), 19)
    
    FIND_LINKUP = (FindLinkup(), 11)
    FIND_LINKUP_STALL = (Stall(), 19)
    FIND_LINKUP_NO_PATH = (NoPath(), 18)
    
    # ===== LINKUP SECTION (2) =====
    ALIGN_TO_MAIN = (State(), 20)
    ALIGN_TO_MAIN_STALL = (State(), 29)
    
    APPROACH_MAIN = (State(), 21)
    APPROACH_MAIN_STALL = (State(), 29)

    COLLECT_REGOLITH = (Collect(), 22)
    COLLECT_REGOLITH_STALL = (State(), 29)

    SEPARATE_FROM_MAIN = (SeparateMain(), 23)
    SEPARATE_FROM_MAIN_STALL = (Stall(), 29)
    
    # ===== TRAVERSAL SECTION (3) =====

    TRAVERSE_TO_BERM = (TraverseToBerm(False), 30)
    TRAVERSE_TO_BERM_STALL = (Stall(), 39)
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), 38)

    ALIGN_TO_BERM = (AlignToAngle(270), 31)
    ALIGN_TO_BERM_STALL = (Stall(), 39)
    
    MOVE_TO_STAGING = (Traverse(PoseStamped(), False), 32)
    MOVE_TO_STAGING_STALL = (Stall(), 39)
    MOVE_TO_STAGING_NO_PATH = (NoPath(), 38)
    # ===== DEPOSIT SECTION (4) =====
    
    APPROACH_BERM = (ApproachBerm(), 40)
    APPROACH_BERM_STALL = (Stall(), 49)
    
    DEPOSIT = (Deposit(), 41)
    DEPOSIT_STALL = (State(), 49)

    RETREAT_BERM = (RetreatBerm(), 42)
    RETREAT_BERM_STALL = (Stall(), 49)
    

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
            (MiniStates.COLLECT_REGOLITH, Events.STALL): MiniStates.COLLECT_REGOLITH_STALL,
            (MiniStates.COLLECT_REGOLITH_STALL, Events.SUCCESS): MiniStates.COLLECT_REGOLITH,

            (MiniStates.SEPARATE_FROM_MAIN, Events.SUCCESS): MiniStates.TRAVERSE_TO_BERM,
            (MiniStates.SEPARATE_FROM_MAIN, Events.STALL): MiniStates.SEPARATE_FROM_MAIN_STALL,
            (MiniStates.SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    manager = StateManager(MiniStates, MiniStates.INIT, Events, Event)

    rclpy.spin(manager)

    manager.stop_current_state()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
