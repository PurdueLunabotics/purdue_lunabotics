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

## Joystick commands

### General commands 
- cross key left/right: deposition (up/down)
- cross key up/down: lead screw (up/down
- B button: stop everything and latch excavation to zero
- X button: switch mode
- Y button: latch/unlatch excavation (only done in excavation mode

### Drive mode
- left and right joysticks (up/down): tank drive controls for drivetrain (left stick controls left wheel, right stick controls right wheel)

### Excavation mode
- left joystick (up/down): Excavation (forward/backward)
  - latch (Y button): will latch current value of excavation and ignore joystick values, press Y again to unlatch
- right joystick (up/down): Actuation (up/down)
