# NOT UPDATED FOR ROS2
## Run commands on the robot

### Setup Networking

1. Connect router to power

> Optionally to get wifi, connect "internet" port on router to an external ethernet port with wifi

2. Connect computer to ethernet port on the router
3. Turn on the jetson and wait 10-15 sec

### Start robot (default is manual)

1. Open a terminal, set the ROS IP info on the laptop and run the magic run command: 
```
source ~/set_ip.sh
roslaunch lunabot_bringup computer.launch
```

2. Open another terminal, ssh into the robot and run the robot
```
ssh <jetson-ip>
roslaunch lunabot_bringup robot.launch
```

#### Other cmdline options

1. On computer

- `autonomy:=true`: disables publishing of manual control

2. On jetson
- `exp_name:="mpc_test"`: experiment name
- `autonomy:=true`: enables slam, differential drive controller, mpc, global planner, 
- `record:=true`: enables recording to `lunabot_bringup/bags`, ensure the folder exists
- `cameras:=true`: enables cameras
- `slam:=true`: enables slam

## Manual control scheme

### Excavation/Deposition
- Right trigger- spin excavation forwards (pick up dirt)
- Left trigger- spin excavation backwards
- D-pad up/down- move linear actuators (excavation system) up/down
- Y button: latch/unlatch excavation speed. Excavation will remain at the speed it was latched at until the latch is released
- B button- spin auger for deposition

### Driving
- Left and Right joysticks (up/down): tank drive controls for drivetrain (left stick controls left wheel, right stick controls right wheel)
- X button: switch between driving forwards and driving backwards (forwards is in the direction of excavation)

### Other
- Start button- stop all motors on the robot
