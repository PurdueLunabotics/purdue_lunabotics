# Python Launch Files

## What are Launch Files?
Launch Files help to start nodes automatically. We use them to reduce the amount of work we need to do to start the robot. They can also configure nodes and respond to arguments that you pass to them. Launch files can be written in 3 languages: XML, YAML, and Python. This file focuses on Python launch files. See [here](xml_launch.md) for XML launch file information.

## Why Python?
Python is one of the most popular languages used today. This makes it easier to write launch files for ROS users who may not be familiar with XML, or who are looking for a quick and effortless way to make a launch file without having to use XML. Launch files in ROS2 are pretty evenly distributed between Python and XML styles, so both are acceptable choices.

## Directory Organization
```
- ros_pkg/
  - launch/
    - launch_file.launch.py
  - src/
  - include/
  - ...
```
All launch files, regardless of whether they are Python or XML, should be located in the `launch` directory of your ROS package.

## Writing a Python Launch File
Every Python launch file needs a `generate_launch_description()` function. This is the function that is called to generate the launch command in ROS2, and is assumed by ROS to exist in your launch file when launching. The `generate_launch_description()` function returns a `launch.LaunchDescription` object that represents all nodes and/or other launch files to launch simultaneously by the launch file.

### Making a Launch Description
A `LaunchDescription` is instantiated as follows.
```
ld = LaunchDescription([node1, launch_file, node2, etc., ...])
```
After the `launch,LaunchDescription` is created inside the `generate_launch_description()` function, all you need to do is return it from the function to finish your launch file.

### Including Nodes
When including a Node in the launch description, you need to pass a `launch_ros.actions.Node` as one of the arguments in the `LaunchDescription`. Here's a general template for how you instantiate a `launch_ros.actions.Node` in a launch file:
```
node = Node(
  package='package_name',
  executable='node_name',
  arguments=[
    'arg1_name', 'value',
    'arg2_name', 'value',
    ...,
    'argn_name', 'value'
  ],
  output='screen'
)
```

## Conclusion
Using this template, you should now have the tools to create a basic launch file to launch the Python PID node you created. Below is an example of the general structure that your file should have. _Hint: the package name is 'lunabot_control', executable name is the node name you put into `lunabot_control`'s `setup.py`, and there shouldn't be any arguments._
```
from launch import LaunchDescription

def generate_launch_description():
  # define your node argument to pass into the launch file

  ld = LaunchDescription([
    node_launch_object
  ])
  return ld
```

## Additional Resources
You are always advised to look toward the ROS2 documentation when in doubt (even your leads find themselves on it quite often ;)). The documentation on how to make a simple launch file in ROS2 is [here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).
