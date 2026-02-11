from enum import Enum
from state import Events, State
import rclpy
from state_manager import StateManager

class MiniStates(Enum):
    FIND_LINKUP = State()
    FIND_LINKUP_STALL = State()
    FIND_LINKUP_NO_PATH = State()

    MOVE_TO_BERM = State()
    MOVE_TO_BERM_STALL = State()
    MOVE_TO_BERM_NO_PATH = State()

    DEPOSIT = State()
    DEPOSIT_STALL = State()

    MOVE_TO_STAGING = State()
    MOVE_TO_STAGING_STALL = State()
    MOVE_TO_STAGING_NO_PATH = State()

    ALIGN_TO_MAIN = State()
    ALIGN_TO_MAIN_STALL = State()

    COLLECT_REGOLITH = State()

    SEPARATE_FROM_MAIN = State()
    SEPARATE_FROM_MAIN_STALL = State()

    @staticmethod
    def get_transition(state, event: Events):
        transitions = {
            (MiniStates.FIND_LINKUP, Events.SUCCESS): MiniStates.MOVE_TO_BERM,
            (MiniStates.FIND_LINKUP, Events.STALL): MiniStates.FIND_LINKUP_STALL,
            (MiniStates.FIND_LINKUP, Events.NO_PATH): MiniStates.FIND_LINKUP_NO_PATH,
            (MiniStates.FIND_LINKUP_STALL, Events.SUCCESS): MiniStates.FIND_LINKUP,
            (MiniStates.FIND_LINKUP_NO_PATH, Events.SUCCESS): MiniStates.FIND_LINKUP,

            (MiniStates.MOVE_TO_BERM, Events.SUCCESS): MiniStates.DEPOSIT,
            (MiniStates.MOVE_TO_BERM, Events.STALL): MiniStates.MOVE_TO_BERM_STALL,
            (MiniStates.MOVE_TO_BERM, Events.NO_PATH): MiniStates.MOVE_TO_BERM_NO_PATH,
            (MiniStates.MOVE_TO_BERM_STALL, Events.SUCCESS): MiniStates.MOVE_TO_BERM,
            (MiniStates.MOVE_TO_BERM_NO_PATH, Events.SUCCESS): MiniStates.MOVE_TO_BERM,

            (MiniStates.DEPOSIT, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.DEPOSIT, Events.STALL): MiniStates.DEPOSIT_STALL,
            (MiniStates.DEPOSIT_STALL, Events.SUCCESS): MiniStates.DEPOSIT,

            (MiniStates.MOVE_TO_STAGING, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,
            (MiniStates.MOVE_TO_STAGING, Events.STALL): MiniStates.MOVE_TO_STAGING_STALL,
            (MiniStates.MOVE_TO_STAGING, Events.NO_PATH): MiniStates.MOVE_TO_STAGING_NO_PATH,
            (MiniStates.MOVE_TO_STAGING_STALL, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,
            (MiniStates.MOVE_TO_STAGING_NO_PATH, Events.SUCCESS): MiniStates.MOVE_TO_STAGING,

            (MiniStates.ALIGN_TO_MAIN, Events.SUCCESS): MiniStates.COLLECT_REGOLITH,
            (MiniStates.ALIGN_TO_MAIN, Events.STALL): MiniStates.ALIGN_TO_MAIN_STALL,
            (MiniStates.ALIGN_TO_MAIN_STALL, Events.SUCCESS): MiniStates.ALIGN_TO_MAIN,

            (MiniStates.COLLECT_REGOLITH, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN,

            (MiniStates.SEPARATE_FROM_MAIN, Events.SUCCESS): MiniStates.MOVE_TO_BERM,
            (MiniStates.SEPARATE_FROM_MAIN, Events.STALL): MiniStates.SEPARATE_FROM_MAIN_STALL,
            (MiniStates.SEPARATE_FROM_MAIN_STALL, Events.SUCCESS): MiniStates.SEPARATE_FROM_MAIN
        }

        return transitions.get((state, event), None)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = StateManager(MiniStates, MiniStates.FIND_LINKUP, Events)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
