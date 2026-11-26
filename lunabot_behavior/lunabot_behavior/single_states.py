#!/usr/bin/env python3

from enum import Enum

from geometry_msgs.msg import PoseStamped
from lunabot_behavior.states.wait_for_linkup import WaitForLinkup
from lunabot_config.led_colors import LedColor
from lunabot_msgs.msg import Event

from lunabot_behavior.states.align_to_berm import AlignToBerm
from lunabot_behavior.states.traverse_to_linkup import TraverseToLinkup
from lunabot_behavior.states.align_to_linkup import AlignToLinkup
from lunabot_behavior.states.find_linkup import FindLinkup, TraverseToMiddle
from lunabot_behavior.states.approach_trench import ApproachTrench, RetreatTrench
from lunabot_behavior.states.align_trench import AlignTrench
from lunabot_behavior.states.align_to_angle import AlignToAngle
from lunabot_behavior.states.traverse_to_berm import TraverseToBerm
from lunabot_behavior.states.traverse_to_exc import TraverseToExc
from lunabot_behavior.states.traverse import NoPath, Traverse, Stall
from lunabot_behavior.states.deposit import Deposit
from lunabot_behavior.states.approach_berm import ApproachBerm, ApproachBermBackwards
from lunabot_behavior.states.plunge import Plunge
from lunabot_behavior.states.raise_act import Raise
from lunabot_behavior.states.retreat_berm import RetreatBerm
from lunabot_behavior.states.trench import Trench
from lunabot_behavior.states.fullstop import Stop
from lunabot_behavior.states.init import SetupObstacles
from lunabot_behavior.states.mapping_spin import MappingSpin

from lunabot_behavior.state import Events, State
from lunabot_behavior.state_manager import StateManager

import rclpy

class SingleStates(Enum):
    
    STOP = (Stop(), (LedColor.RED, LedColor.GREEN))
    
    INIT_RAISE = (Raise(False), (LedColor.GREEN, LedColor.GREEN))
    INIT_OBSTACLES = (SetupObstacles(True), (LedColor.GREEN, LedColor.BLUE))
    
    STARTING_PLUNGE = (Plunge(), (LedColor.GREEN, LedColor.BLUE))
    STARTING_PLUNGE_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))
    
    STARTING_RAISE = (Raise(), (LedColor.GREEN, LedColor.MAGENTA))
    STARTING_RAISE_STALL = (Stall(), (LedColor.GREEN, LedColor.RED))

    INIT_TRAVERSE_TO_BERM = (TraverseToBerm(True), (LedColor.BLUE, LedColor.YELLOW))
    INIT_TRAVERSE_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    INIT_TRAVERSE_TO_BERM_NO_PATH = (NoPath(), (LedColor.BLUE, LedColor.ORANGE))

    INIT_ALIGN_TO_BERM = (AlignToBerm(True), (LedColor.BLUE, LedColor.GREEN))
    INIT_ALIGN_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    INIT_APPROACH_BERM = (ApproachBermBackwards(True), (LedColor.BLUE, LedColor.TEAL))
    INIT_APPROACH_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))

    INIT_DEPOSIT_BERM = (Deposit(), (LedColor.MAGENTA, LedColor.YELLOW))
    INIT_DEPOSIT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))
    
    INIT_RETREAT_BERM = (RetreatBerm(False), (LedColor.MAGENTA, LedColor.GREEN))
    INIT_RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))
    
    TRAVERSE_TO_EXC_ZONE = (TraverseToExc(True), (LedColor.GREEN, LedColor.WHITE))
    TRAVERSE_TO_EXC_ZONE_STALL =  (Stall(), (LedColor.GREEN, LedColor.RED))
    TRAVERSE_TO_EXC_ZONE_NO_PATH = (NoPath(), (LedColor.GREEN, LedColor.ORANGE))
    
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

    ALIGN_TO_LINKUP = (AlignToLinkup(), (LedColor.YELLOW, LedColor.YELLOW))
    ALIGN_TO_LINKUP_STALL = (Stall(), (LedColor.YELLOW, LedColor.RED))

    TRAVERSE_TO_BERM = (TraverseToBerm(True), (LedColor.BLUE, LedColor.YELLOW))
    TRAVERSE_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    TRAVERSE_TO_BERM_NO_PATH = (NoPath(), (LedColor.BLUE, LedColor.ORANGE))

    ALIGN_TO_BERM = (AlignToBerm(True), (LedColor.BLUE, LedColor.GREEN))
    ALIGN_TO_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))
    
    APPROACH_BERM = (ApproachBermBackwards(True), (LedColor.BLUE, LedColor.TEAL))
    APPROACH_BERM_STALL = (Stall(), (LedColor.BLUE, LedColor.RED))

    DEPOSIT_BERM = (Deposit(), (LedColor.MAGENTA, LedColor.YELLOW))
    DEPOSIT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))
    
    RETREAT_BERM = (RetreatBerm(False), (LedColor.MAGENTA, LedColor.GREEN))
    RETREAT_BERM_STALL = (Stall(), (LedColor.MAGENTA, LedColor.RED))

    IDLE = (State(), 0)

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (SingleStates.STOP, Events.SUCCESS): SingleStates.INIT_OBSTACLES,
            
            (SingleStates.INIT_OBSTACLES, Events.SUCCESS): SingleStates.INIT_RAISE,
            (SingleStates.INIT_RAISE, Events.SUCCESS): SingleStates.STARTING_PLUNGE,
            
            (SingleStates.STARTING_PLUNGE, Events.SUCCESS): SingleStates.STARTING_RAISE,
            (SingleStates.STARTING_PLUNGE, Events.STALL): SingleStates.STARTING_PLUNGE_STALL,
            (SingleStates.STARTING_PLUNGE_STALL, Events.SUCCESS): SingleStates.STARTING_PLUNGE,
            
            (SingleStates.STARTING_RAISE, Events.SUCCESS): SingleStates.INIT_TRAVERSE_TO_BERM,
            (SingleStates.STARTING_RAISE, Events.STALL): SingleStates.STARTING_RAISE_STALL,
            (SingleStates.STARTING_RAISE_STALL, Events.SUCCESS): SingleStates.STARTING_RAISE,

            (SingleStates.INIT_TRAVERSE_TO_BERM, Events.SUCCESS): SingleStates.INIT_ALIGN_TO_BERM,
            (SingleStates.INIT_TRAVERSE_TO_BERM, Events.STALL): SingleStates.INIT_TRAVERSE_TO_BERM_STALL,
            (SingleStates.INIT_TRAVERSE_TO_BERM, Events.NO_PATH): SingleStates.INIT_TRAVERSE_TO_BERM_NO_PATH,
            (SingleStates.INIT_TRAVERSE_TO_BERM_STALL, Events.SUCCESS): SingleStates.INIT_TRAVERSE_TO_BERM,
            (SingleStates.INIT_TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): SingleStates.INIT_TRAVERSE_TO_BERM,

            (SingleStates.INIT_ALIGN_TO_BERM, Events.SUCCESS): SingleStates.INIT_APPROACH_BERM,
            
            (SingleStates.INIT_APPROACH_BERM, Events.SUCCESS): SingleStates.INIT_DEPOSIT_BERM,
            
            (SingleStates.INIT_DEPOSIT_BERM, Events.SUCCESS): SingleStates.INIT_RETREAT_BERM,
            (SingleStates.INIT_DEPOSIT_BERM, Events.STALL): SingleStates.INIT_DEPOSIT_BERM_STALL,
            (SingleStates.INIT_DEPOSIT_BERM_STALL, Events.SUCCESS): SingleStates.INIT_DEPOSIT_BERM,
            
            (SingleStates.INIT_RETREAT_BERM, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,
            (SingleStates.INIT_RETREAT_BERM, Events.STALL): SingleStates.INIT_RETREAT_BERM_STALL,
            (SingleStates.INIT_RETREAT_BERM_STALL, Events.SUCCESS): SingleStates.INIT_RETREAT_BERM,
                        
            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,
            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.STALL): SingleStates.TRAVERSE_TO_EXC_ZONE_STALL,
            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.NO_PATH): SingleStates.TRAVERSE_TO_EXC_ZONE_NO_PATH,
            (SingleStates.TRAVERSE_TO_EXC_ZONE_NO_PATH, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,
            (SingleStates.TRAVERSE_TO_EXC_ZONE_STALL, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,

            (SingleStates.ALIGN_TO_TRENCH, Events.SUCCESS): SingleStates.APPROACH_TRENCH,
            (SingleStates.ALIGN_TO_TRENCH, Events.NO_PATH): SingleStates.JUST_PLUNGE,
            (SingleStates.ALIGN_TO_TRENCH, Events.STALL): SingleStates.ALIGN_TO_TRENCH_STALL,
            (SingleStates.ALIGN_TO_TRENCH_STALL, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,

            (SingleStates.JUST_PLUNGE, Events.STALL): SingleStates.JUST_PLUNGE_STALL,
            (SingleStates.JUST_PLUNGE, Events.SUCCESS): SingleStates.ALIGN_TO_LINKUP,
            
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
            
            (SingleStates.RETREAT_TRENCH, Events.SUCCESS): SingleStates.TRAVERSE_TO_BERM,
            (SingleStates.RETREAT_TRENCH, Events.STALL): SingleStates.RETREAT_TRENCH_STALL,
            (SingleStates.RETREAT_TRENCH_STALL, Events.SUCCESS): SingleStates.RETREAT_TRENCH,

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
            
            (SingleStates.RETREAT_BERM, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,
            (SingleStates.RETREAT_BERM, Events.STALL): SingleStates.RETREAT_BERM_STALL,
            (SingleStates.RETREAT_BERM_STALL, Events.SUCCESS): SingleStates.RETREAT_BERM,

            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.SUCCESS): SingleStates.ALIGN_TO_TRENCH,
            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.STALL): SingleStates.TRAVERSE_TO_EXC_ZONE_STALL,
            (SingleStates.TRAVERSE_TO_EXC_ZONE, Events.NO_PATH): SingleStates.TRAVERSE_TO_EXC_ZONE_NO_PATH,
            (SingleStates.TRAVERSE_TO_EXC_ZONE_NO_PATH, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,
            (SingleStates.TRAVERSE_TO_EXC_ZONE_STALL, Events.SUCCESS): SingleStates.TRAVERSE_TO_EXC_ZONE,
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(SingleStates, SingleStates.STOP, Events, Event)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
