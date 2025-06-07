import launch
import csv
import ast

class LaunchManager:
    launch_files = {}
    
    
    def __init__(self):
        self.roscore_running = False
        
        with open('launch_file_list.csv', newline='') as csvfile:
            launchcsv = csv.reader(csvfile, delimiter=',')
            for row in launchcsv:
                self.launch_files[row[0]] = launch.LaunchFile(row[1], row[2], ast.literal_eval(row[3].capitalize()))
        
    def start_roscore(self):
        pass

    def stop_roscore(self):
        pass
    
    def default_start(self):
        self.start_roscore()
        self.launch_files["robot"].launch()
        self.launch_files["computer"].launch()
    
    def stop_all(self):
        for value in self.launch_files.values():
            value.stop()