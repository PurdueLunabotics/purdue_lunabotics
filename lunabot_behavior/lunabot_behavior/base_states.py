class States:

    def __init__(self, state_manager):
        self.state_manager = state_manager

    def state(self) -> str:
        pass

    def periodic(self):
        pass
    def success(self) -> bool:
        pass

    def static(self) -> bool:
    
        return False

    def stuck(self) -> bool:
    
        return False

