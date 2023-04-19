# purdue_lunabotics

This is the official git repo of the Purdue Lunabotics software team.

TODO: Use releases feature to simplify versioning

## Quick Start

1. [Install ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros)

This tutorial assumes you have your `catkin_ws` initialized in your home directory: `~/catkin_ws`

2. Clone the repository to `src` folder of ROS workspace

```
cd ~/catkin_ws/src
git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git
```
3. Install dependencies at the root of your catkin workspace
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

4. Build + source (Do this every time you download new packages)

```
catkin build
source ~/catkin_ws/devel/setup.bash # or .zsh if you use a zsh terminal
```
> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc (or .zshrc) so it sources automatically

### Run the robot

#### 1. Setup Networking

1. Connect router to power

> Optionally to get wifi, connect "internet" port on router to an external ethernet port with wifi

2. Connect computer to ethernet port on the router
3. Turn on the jetson and wait atleast 1.5 min

#### 2. Run roslaunch commands

1. Open a terminal, set the ROS IP info on the laptop and run the magic run command: 
```
set_ip
roslaunch lunabot_bringup robot.launch
```

That's it! Things should be running now.

### Manual Control Commands

##### General commands 
- cross key left/right: deposition (up/down)
- cross key up/down: lead screw (up/down
- B button: stop everything and latch excavation to zero
- X button: switch mode
- Y button: latch/unlatch excavation (only done in excavation mode

##### Drive mode
- left and right joysticks (up/down): tank drive controls for drivetrain (left stick controls left wheel, right stick controls right wheel)

##### Excavation mode
- left joystick (up/down): Excavation (forward/backward)
  - latch (Y button): will latch current value of excavation and ignore joystick values, press Y again to unlatch
- right joystick (up/down): Actuation (up/down)

## Contributing guidelines
See https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/contributing.md
