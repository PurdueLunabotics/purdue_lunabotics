from enum import Enum, auto
from state import Events

class MainStates(Enum):
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
