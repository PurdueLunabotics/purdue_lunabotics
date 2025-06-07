class LaunchFile:
    def __init__(self, package: str, name: str, on_robot: bool, args=None):
        self.package = package
        self.name = name
        self.args = args
        self.on_robot = on_robot
        self.launched = False
        
    def launch(self):
        if self.launched == False:
            self.launched = True
            #start
        # If already going, don't do anything.
    
    def stop(self):
        if self.launched == True:
            self.launched = False
            #stop
        # if already stopped, don't do anything