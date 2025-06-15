class LaunchFile:
    def __init__(self, package: str, name: str, on_robot: bool, default_launch: bool, args=None):
        self.package = package
        self.name = name
        self.args = args
        self.on_robot = on_robot
        self.launched = False
        self.default_launch = default_launch
        
    def launch(self):
        if self.launched == False:
            self.launched = True
            #start
            print(f"Starting file {self.name}")
        # If already going, don't do anything.
    
    def stop(self):
        if self.launched == True:
            self.launched = False
            #stop
            print(f"Stopping file {self.name}")
        # if already stopped, don't do anything