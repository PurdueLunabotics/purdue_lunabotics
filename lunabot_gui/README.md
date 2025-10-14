An (experimental) GUI to simplify robot operations.

# File Descriptions

## launch_file_list.csv

Contains all launch files used by the robot and some needed data about them. Read dynamically on GUI launch. 

Columns are, in order
- Name. This is displayed in the GUI.
- Package the file is located in
- Launch file name. This is the name of the file to be called
- Which device to launch it on. 0 = computer, 1 = main robot, 2 = minibot
- Whether or not to launch the file when the "default startup" button is pressed. True = yes, false = no