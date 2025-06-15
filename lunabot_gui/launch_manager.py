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
                self.launch_files[row[0]] = launch.LaunchFile(row[1], 
                                                              row[2], 
                                                              ast.literal_eval(row[3].capitalize()), 
                                                              ast.literal_eval(row[4].capitalize()))
    
    def default_start(self):
        for value in self.launch_files.values():
            if value.default_launch:
                value.launch()
    
    def stop_all(self):
        for value in self.launch_files.values():
            value.stop()
        
    def launch_file(self, file_name):
        self.launch_files[file_name].launch()
        
    def stop_file(self, file_name):
        self.launch_files[file_name].stop()