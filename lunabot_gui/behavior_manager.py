import glob

class BehaviorManager:
    def __init__(self):
        self.behavior_path = "../lunabot_behavior/src/lunabot_behavior/"
        
        f = open("behavior_list.txt")
        self.files = []
        self.files.append("Stop scripts")
        for line in f:
            self.files.append(line)
        f.close()
        
        self.runningFile = -1
        
    # get the list of behavior files
    def get_list(self) -> list[str]:
        return self.files
    
    def stop(self, file_idx : int):
        if self.runningFile == -1:
            return # no file running
        
        print(f"Stopping behavior script {self.files[file_idx]}")
        # Stop the file here
        self.runningFile = -1
    
    def launch(self, file_idx : int):
        # Stop any currently running file before starting a new one
        if self.runningFile != -1:
            self.stop(self.runningFile) 
            
        if file_idx == 0:
            return # stop command
        
        print(f"Launching behavior script {self.files[file_idx]}")
        #launch the file here
        self.runningFile = file_idx
        
        
    