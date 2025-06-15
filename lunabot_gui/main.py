import dearpygui.dearpygui as dpg
import launch_manager
import csv
import ast

lmgr = launch_manager.LaunchManager()

class LaunchBox:
    def __init__(self, name, default_value):
        self.name = name
        self.default = default_value
        self.button_id = 0
        
    def add_button_id(self, new_id):
        self.button_id = new_id
    
    def set_button_state(self, new_state):
        dpg.set_value(self.button_id, new_state)

def main():
    launch_list = []

    with open('launch_file_list.csv', newline='') as csvfile:
        launchcsv = csv.reader(csvfile, delimiter=',')
        for row in launchcsv:
            launch_list.append(LaunchBox(row[0], ast.literal_eval(row[4].capitalize())))
    

    
    dpg.create_context()
    dpg.set_global_font_scale(1.2)

    with dpg.window(label="Tutorial", tag="Primary Window"):
        with dpg.table(header_row=False):
            # use add_table_column to add columns to the table,
            # table columns use child slot 0
            dpg.add_table_column()
            dpg.add_table_column()
            dpg.add_table_column()

            # add_table_next_column will jump to the next row
            # once it reaches the end of the columns
            # table next column use slot 1
            for i in range(0, len(launch_list) + 2): # Rows
                with dpg.table_row():
                    for j in range(0, 3): # Cols
                        if j == 0: # FIRST COLUMN: LAUNCH SCRIPTS
                            if i >= len(launch_list):
                                new_i = i - len(launch_list)
                                
                                if new_i == 0:
                                    dpg.add_button(label="Default Start", 
                                                   callback=start_callback,
                                                   user_data=launch_list)
                                if new_i == 1:
                                    dpg.add_button(label="Stop All", 
                                                   callback=stop_callback,
                                                   user_data=launch_list)
                                    
                                continue
                            
                            check_id = dpg.add_checkbox(label=launch_list[i].name, 
                                             callback=launch_callback, 
                                             user_data=launch_list[i].name)
                            launch_list[i].add_button_id(check_id)
                        elif j == 1: # SECOND COLUMN
                            pass
                        else: #THIRD COLUMN
                            pass

    dpg.create_viewport(title='LunaGUI', width=1000, height=500)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    dpg.start_dearpygui()
    dpg.destroy_context()
    
def start_callback(sender, app_data, user_data : list[LaunchBox]):
    lmgr.default_start()
    for box in user_data:
        if box.default:
            box.set_button_state(True)
    
def stop_callback(sender, app_data, user_data : list[LaunchBox]):
    lmgr.stop_all()
    for box in user_data:
        box.set_button_state(False)
    
def launch_callback(sender, app_data, user_data):
    if app_data: # Button pressed
        lmgr.launch_file(user_data)
    else:
        lmgr.stop_file(user_data)
    
    
if __name__ == "__main__":
    main()
    