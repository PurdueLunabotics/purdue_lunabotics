# mining_arena_gazebo
Gazebo world environments for the NASA competition


## What's in here?
Lunabotics 2023-2024 competition arena assets and world files

## Models
- Apriltag assets
- arena assets with correct dimensions
## Worlds
- arena_nasa.world: Kennedy space center Arena (2024 competition)
- arena_ucf.world: UCF Arena (2024 competition)

## Running sim: 
1. Launch sim:
```
roslaunch lunabot_bringup sim.launch verbose:=true
```
> NOTE: if you get an error associated with a missing lib with the name skid_steer in it, uncomment the env export in gazebo.launch

Options:
- `KSC_arena:=<true/false>`: Use KSC or UCF arena
- `verbose:=<true/false>`: Runs with gazebo verbose
- `debug:=<true/false>`: Runs with gdb

Remove repeated warnings: append `2> >(grep -Ev "TF_REPEATED_DATA|buffer_core|at line|^$")` to the end of the command.

(This won't print any lines with the words `TF_REPEATED_DATA` or `buffer_core`)

![sim_nav_gazebo](https://user-images.githubusercontent.com/41026849/163585429-cf9080de-40e7-4be1-9648-d75ab31ae4af.png)

## Credits
Apriltag assets from https://github.com/koide3/gazebo_apriltag
