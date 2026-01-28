#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from enum import Enum, auto


class Events(Enum):
    SUCCESS = auto()
    STALL = auto()
    NO_PATH = auto()
    FAIL = auto()


class MainStates(Enum):
    IDLE = auto()

    INIT_TO_LINKUP = auto()
    INIT_TO_LINKUP_STALL = auto()
    INIT_TO_LINKUP_NO_PATH = auto()
    
    ALIGN_TO_TRENCH = auto()
    ALIGN_TO_TRENCH_STALL = auto()
    
    PLUNGE_ACT = auto()
    PLUNGE_ACT_STALL = auto()
    
    TRENCH = auto()
    TRENCH_STALL = auto()
    
    RAISE_ACT = auto()
    RAISE_ACT_STALL = auto()
    
    TRAVERSE_TO_LINKUP = auto()
    TRAVERSE_TO_LINKUP_STALL = auto()
    TRAVERSE_TO_LINKUP_NO_PATH = auto()

    WAIT_FOR_LINKUP = auto()

    TRAVERSE_TO_BERM = auto()
    TRAVERSE_TO_BERM_STALL = auto()
    TRAVERSE_TO_BERM_NO_PATH = auto()

    ALIGN_TO_BERM = auto()
    ALIGN_TO_BERM_STALL = auto()
    
    APPROACH_BERM = auto()
    APPROACH_BERM_STALL = auto()
    
    DEPOSIT = auto()
    DEPOSIT_STALL = auto()

    transitions = {
        (IDLE, Events.SUCCESS): INIT_TO_LINKUP,
        
        (INIT_TO_LINKUP, Events.SUCCESS): ALIGN_TO_TRENCH,
        (INIT_TO_LINKUP, Events.STALL): INIT_TO_LINKUP_STALL,
        (INIT_TO_LINKUP, Events.NO_PATH): INIT_TO_LINKUP_NO_PATH,
        (INIT_TO_LINKUP_NO_PATH, Events.SUCCESS): INIT_TO_LINKUP,
        (INIT_TO_LINKUP_STALL, Events.SUCCESS): INIT_TO_LINKUP,

        (ALIGN_TO_TRENCH, Events.SUCCESS): PLUNGE_ACT,
        (ALIGN_TO_TRENCH, Events.STALL): ALIGN_TO_TRENCH_STALL,
        (ALIGN_TO_TRENCH_STALL, Events.SUCCESS): ALIGN_TO_TRENCH,
        
        (PLUNGE_ACT, Events.SUCCESS): TRENCH,
        (PLUNGE_ACT, Events.STALL): PLUNGE_ACT_STALL,
        (PLUNGE_ACT_STALL, Events.SUCCESS): PLUNGE_ACT,
        
        (TRENCH, Events.SUCCESS): RAISE_ACT,
        (TRENCH, Events.STALL): TRENCH_STALL,
        (TRENCH_STALL, Events.SUCCESS): TRENCH,
        
        (RAISE_ACT, Events.SUCCESS): TRAVERSE_TO_LINKUP,
        (RAISE_ACT, Events.STALL): RAISE_ACT_STALL,
        (RAISE_ACT_STALL, Events.SUCCESS): RAISE_ACT,
        
        (TRAVERSE_TO_LINKUP, Events.SUCCESS): DEPOSIT,
        (TRAVERSE_TO_LINKUP, Events.STALL): TRAVERSE_TO_LINKUP_STALL,
        (TRAVERSE_TO_LINKUP, Events.NO_PATH): TRAVERSE_TO_LINKUP_NO_PATH,
        (TRAVERSE_TO_LINKUP_STALL, Events.SUCCESS): TRAVERSE_TO_LINKUP,
        (TRAVERSE_TO_LINKUP_NO_PATH, Events.SUCCESS): TRAVERSE_TO_LINKUP,

        (WAIT_FOR_LINKUP, Events.SUCCESS): DEPOSIT,
        (WAIT_FOR_LINKUP, Events.FAIL): TRAVERSE_TO_BERM,

        # in case minibot is indisposed and big bot has to make full cycles
        (TRAVERSE_TO_BERM, Events.SUCCESS): ALIGN_TO_BERM,
        (TRAVERSE_TO_BERM, Events.STALL): ALIGN_TO_BERM,
        (TRAVERSE_TO_BERM, Events.NO_PATH): TRAVERSE_TO_BERM_NO_PATH,
        (TRAVERSE_TO_BERM_STALL, Events.SUCCESS): TRAVERSE_TO_BERM,
        (TRAVERSE_TO_BERM_NO_PATH, Events.SUCCESS): TRAVERSE_TO_BERM,

        (ALIGN_TO_BERM, Events.SUCCESS): APPROACH_BERM,
        (APPROACH_BERM, Events.SUCCESS): DEPOSIT,
        
        (DEPOSIT, Events.SUCCESS): ALIGN_TO_TRENCH,
        (DEPOSIT, Events.STALL): DEPOSIT_STALL,
        (DEPOSIT_STALL, Events.SUCCESS): DEPOSIT
    }

    @staticmethod
    def get_transition(state, event: Events):
        return MainStates.transitions.get((state, event), None)


class MiniStates(Enum):
    IDLE = auto()

    FIND_LINKUP = auto()
    FIND_LINKUP_STALL = auto()
    FIND_LINKUP_NO_PATH = auto()

    MOVE_TO_BERM = auto()
    MOVE_TO_BERM_STALL = auto()
    MOVE_TO_BERM_NO_PATH = auto()

    DEPOSIT = auto()
    DEPOSIT_STALL = auto()

    MOVE_TO_STAGING = auto()
    MOVE_TO_STAGING_STALL = auto()
    MOVE_TO_STAGING_NO_PATH = auto()

    ALIGN_TO_MAIN = auto()
    ALIGN_TO_MAIN_STALL = auto()

    COLLECT_REGOLITH = auto()

    SEPARATE_FROM_MAIN = auto()
    SEPARATE_FROM_MAIN_STALL = auto()

    transitions = {
        (IDLE, Events.SUCCESS): FIND_LINKUP,

        (FIND_LINKUP, Events.SUCCESS): MOVE_TO_BERM,
        (FIND_LINKUP, Events.STALL): FIND_LINKUP_STALL,
        (FIND_LINKUP, Events.NO_PATH): FIND_LINKUP_NO_PATH,
        (FIND_LINKUP_STALL, Events.SUCCESS): FIND_LINKUP,
        (FIND_LINKUP_NO_PATH, Events.SUCCESS): FIND_LINKUP,

        (MOVE_TO_BERM, Events.SUCCESS): DEPOSIT,
        (MOVE_TO_BERM, Events.STALL): MOVE_TO_BERM_STALL,
        (MOVE_TO_BERM, Events.NO_PATH): MOVE_TO_BERM_NO_PATH,
        (MOVE_TO_BERM_STALL, Events.SUCCESS): MOVE_TO_BERM,
        (MOVE_TO_BERM_NO_PATH, Events.SUCCESS): MOVE_TO_BERM,

        (DEPOSIT, Events.SUCCESS): MOVE_TO_STAGING,
        (DEPOSIT, Events.STALL): DEPOSIT_STALL,
        (DEPOSIT_STALL, Events.STALL): DEPOSIT,

        (MOVE_TO_STAGING, Events.SUCCESS): ALIGN_TO_MAIN,
        (MOVE_TO_STAGING, Events.STALL): MOVE_TO_STAGING_STALL,
        (MOVE_TO_STAGING, Events.NO_PATH): MOVE_TO_STAGING_NO_PATH,
        (MOVE_TO_STAGING_STALL, Events.SUCCESS): MOVE_TO_STAGING,
        (MOVE_TO_STAGING_NO_PATH, Events.SUCCESS): MOVE_TO_STAGING,

        (ALIGN_TO_MAIN, Events.SUCCESS): COLLECT_REGOLITH,
        (ALIGN_TO_MAIN, Events.STALL): ALIGN_TO_MAIN_STALL,
        (ALIGN_TO_MAIN_STALL, Events.SUCCESS): ALIGN_TO_MAIN,

        (COLLECT_REGOLITH, Events.SUCCESS): SEPARATE_FROM_MAIN,

        (SEPARATE_FROM_MAIN, Events.SUCCESS): MOVE_TO_BERM,
        (SEPARATE_FROM_MAIN, Events.STALL): SEPARATE_FROM_MAIN_STALL,
        (SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): SEPARATE_FROM_MAIN
    }

    @staticmethod
    def get_transition(state, event: Events):
        if (MiniStates.ANY, event) in MiniStates.transitions:
            return MiniStates.transitions.get((MiniStates.ANY, event))

        return MiniStates.transitions.get((state, event), None)


class StateManager(Node):
    def __init__(self):
        super().__init__("state_manager")

        self.main_state = MainStates.IDLE
        self.main_return_state = None

        self.mini_state = MiniStates.IDLE
        self.mini_return_state = None

        # overall state control flags
        self.activate_main = False
        self.activate_mini = False

        self.linkup_coords = [None, None]

    def update_main_state(self, state, event):
        next_state = MainStates.get_transition(state, event)

        if next_state is not None:
            self.main_state = next_state

    def update_mini_state(self, state, event): # TODO: update
        next_state = MiniStates.get_transition(state, event)

        if next_state is not None:
            self.mini_state = next_state

    def main_state_periodic(self):
        if self.activate_main:
            event = None

            # define what constitutes success message and what actions to carry out for each state
            match self.main_state:
                case MainStates.IDLE:
                    pass

                case MainStates.INIT_TO_LINKUP:
                    pass

                case MainStates.INIT_TO_LINKUP_NO_PATH:
                    pass

                case MainStates.INIT_TO_LINKUP_STALL:
                    pass

                case MainStates.ALIGN_TO_TRENCH:
                    pass

                case MainStates.ALIGN_TO_TRENCH_STALL:
                    pass

                case MainStates.PLUNGE_ACT:
                    pass

                case MainStates.PLUNGE_ACT_STALL:
                    pass

                case MainStates.TRENCH:
                    pass

                case MainStates.TRENCH_STALL:
                    pass

                case MainStates.RAISE_ACT:
                    pass

                case MainStates.RAISE_ACT_STALL:
                    pass

                case MainStates.TRAVERSE_TO_LINKUP:
                    pass

                case MainStates.TRAVERSE_TO_LINKUP_NO_PATH:
                    pass

                case MainStates.TRAVERSE_TO_LINKUP_STALL:
                    pass

                case MainStates.DEPOSIT:
                    pass

            if event is not None:
                self.update_main_state(self.main_state, event)

    def mini_state_periodic(self):
        if self.activate_mini:
            event = None

            match self.main_state:
                case MiniStates.IDLE:
                    # stop all robot behaviors
                    pass

                case MiniStates.FIND_LINKUP:
                    # path to berm
                    # identify linkup spot as one that has a straight line of sight (of some length) to excavation zone
                    # keep track of that line - will be useful for angular lineup of main bot and determining of staging area for minibot
                    pass

                case MiniStates.FIND_LINKUP_STALL:
                    pass

                case MiniStates.FIND_LINKUP_NO_PATH:
                    pass

                case MiniStates.MOVE_TO_BERM:
                    # path to berm - store the final path
                    pass

                case MiniStates.MOVE_TO_BERM_STALL:
                    pass

                case MiniStates.MOVE_TO_BERM_NO_PATH:
                    pass

                case MiniStates.DEPOSIT:
                    pass

                case MiniStates.DEPOSIT_STALL:
                    pass

                case MiniStates.MOVE_TO_STAGING:
                    # move to area a certain distance away from linkup spot on the line with straight line of sight
                    pass

                case MiniStates.MOVE_TO_STAGING_STALL:
                    pass

                case MiniStates.MOVE_TO_STAGING_NO_PATH:
                    pass

                case MiniStates.ALIGN_TO_MAIN:
                    pass

                case MiniStates.ALIGN_TO_MAIN_STALL:
                    pass

                case MiniStates.COLLECT_REGOLITH:
                    pass

                case MiniStates.SEPARATE_FROM_MAIN:
                    pass

                case MiniStates.SEPARATE_FROM_MAIN_STALL:
                    pass

            if event is not None:
                self.update_mini_state(self.main_state, event)


def main(args=None):
    rclpy.init(args=args)

    state_manager = StateManager()

    rclpy.spin(state_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
