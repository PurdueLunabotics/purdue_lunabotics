This is the main code for the teensy system installed on the robot. 

## Folder Structure

"lib" contains the bulk of the code used by the robot.

### lunabot_includes

This contains all the libraries that we're using. Specifically: there is

- ADS1115 -> The library for the current sensors
- Encoder -> The library for the quadrature encoders
- HX711 -> The library for the load cells
- Sabertooth -> The library for the motor controllers
- pb -> all related to the communication between the teensy and the jetson
- Utility (folder) -> contains files needed for the Encoder library to work properly

### lunabot_drivers

This contains all of the code that is either written by us or frequenly maintained by us.

- RobotMsgs -> The actual messages being passed between the teensy and the jetson
- interfaces -> Deals primarily with interpreting sensor data and converting it into a useful form for us to use. May eventually be broken up into individual files
- robot -> Sits between teensy_main (which recieves data from the robot) and interfaces (which and recives values to the motor controllers and from the encoders)

### teensy_main.ino

The main file for the teensy. Setup runs once, loop runs repeatedly until stopped or reset. Gets the data from the robot sensors and sends it to the jetson, reads jetson effort values and sends them to the motors, and implements various other functionalities like overcurrent protection. 
