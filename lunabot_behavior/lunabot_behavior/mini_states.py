from enum import Enum, auto
from state import Events

class MiniStates(Enum):
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
        return MiniStates.transitions.get((state, event), None)
