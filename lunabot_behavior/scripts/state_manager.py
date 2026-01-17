#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from enum import Enum, auto


class Events(Enum):
    SUCCESS = auto()
    STALL = auto()
    NO_PATH = auto()


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
    
    MOVE_TO_LINKUP = auto()
    MOVE_TO_LINKUP_STALL = auto()
    MOVE_TO_LINKUP_NO_PATH = auto()
    
    DEPOSIT = auto()
    DEPOSIT_STALL = auto()

    transitions = {
        (IDLE, Events.SUCCESS): INIT_TO_LINKUP,
        (INIT_TO_LINKUP, Events.SUCCESS): ALIGN_TO_TRENCH,
        (ALIGN_TO_TRENCH, Events.SUCCESS): PLUNGE_ACT,
        (PLUNGE_ACT, Events.SUCCESS): TRENCH,
        (TRENCH, Events.SUCCESS): RAISE_ACT,
        (RAISE_ACT, Events.SUCCESS): MOVE_TO_LINKUP,
        (MOVE_TO_LINKUP, Events.SUCCESS): DEPOSIT,
        (DEPOSIT, Events.SUCCESS): ALIGN_TO_TRENCH,
    }

    @staticmethod
    def get_transition(state, event: Events):
        return MainStates.transitions.get((state, event), None)



class MiniStates(Enum):
    IDLE = auto()
    FIND_LINKUP = auto()
    MOVE_TO_BERM = auto()
    DEPOSIT = auto()
    MOVE_TO_STAGING = auto()
    ALIGN_TO_MAIN = auto()
    COLLECT_REGOLITH = auto()
    SEPARATE_FROM_MAIN = auto()

    transitions = {
        (IDLE, Events.SUCCESS): FIND_LINKUP,
        (FIND_LINKUP, Events.SUCCESS): MOVE_TO_BERM,
        (MOVE_TO_BERM, Events.SUCCESS): DEPOSIT,
        (DEPOSIT, Events.SUCCESS): MOVE_TO_STAGING,
        (MOVE_TO_STAGING, Events.SUCCESS): ALIGN_TO_MAIN,
        (ALIGN_TO_MAIN, Events.SUCCESS): COLLECT_REGOLITH,
        (COLLECT_REGOLITH, Events.SUCCESS): SEPARATE_FROM_MAIN,
        (SEPARATE_FROM_MAIN, Events.SUCCESS): MOVE_TO_BERM,
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

    def update_main_state(self, state, event):
        # return to previous state if stall is handles
        if state == MainStates.STALL and event == Events.SUCCESS:
            self.main_state = self.main_return_state
            return

        next_state = MainStates.get_transition(self.main_state, event)

        if next_state is not None:
            # store current state if stalling
            if next_state == MainStates.STALL:
                self.main_return_state = self.main_state

            self.main_state = next_state

    def update_mini_state(self, state, event):
        # return to previous state if stall is handled
        if state == MiniStates.STALL and event == Events.SUCCESS:
            self.mini_state = self.mini_return_state
            return

        next_state = MiniStates.get_transition(self.mini_state, event)

        if next_state is not None:
            # store current state if stalling
            if next_state == MiniStates.STALL:
                self.mini_return_state = self.mini_state

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

                case MainStates.ALIGN_TO_TRENCH:
                    pass

                case MainStates.PLUNGE_ACT:
                    pass

                case MainStates.TRENCH:
                    pass

                case MainStates.RAISE_ACT:
                    pass

                case MainStates.MOVE_TO_LINKUP:
                    pass

                case MainStates.DEPOSIT:
                    pass

                case MainStates.STALL:
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

                case MiniStates.MOVE_TO_BERM:
                    # path to berm - store the final path
                    pass

                case MiniStates.DEPOSIT:
                    pass

                case MiniStates.MOVE_TO_STAGING:
                    # move to area a certain distance away from linkup spot on the line with straight line of sight
                    pass

                case MiniStates.ALIGN_TO_MAIN:
                    pass

                case MiniStates.COLLECT_REGOLITH:
                    pass

                case MiniStates.SEPARATE_FROM_MAIN:
                    pass

                case MiniStates.STALL:
                    pass

            if event is not None:
                self.update_mini_state(self.main_state, event)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
