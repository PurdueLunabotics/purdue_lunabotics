---
title: "Software Onboarding"
colorlinks: true
linkcolor: blue
urlcolor: blue
toc: true
linestretch: 1.3

geometry:
  - top=1in
  - bottom=1in
  - left=1in
  - right=1in

header-includes:
    - \renewcommand{\familydefault}{\sfdefault}
---

\newpage

## Onboarding Process

Welcome to Software! Before we begin working on our main projects for the season
we need to set up and learn to use the tools we use to program our robot. We’ll
go over the basics of ROS(Robot Operating System), talk about the general
structure of our software, then set up a Linux container to run the simulator
and other tests in. 

Throughout onboarding you will implement a couple of ROS nodes to allow you to
manually control the robot in the simulator. These nodes are very similar to the
ones that are currently used on the robot. 

This onboarding is intended to take the next four meetings, but you can complete
it as quickly or as slowly as you would like. All the work for this onboarding
is intended to be completed during normal club hours, which are 10am to 3pm on
Saturdays with a break for lunch, and 7pm to 9pm on Tuesdays. Please note that
only the first 2 hours of the first Saturday meeting on September 5th will be
used for onboarding. That means that onboarding should aim to be completed by
Tuesday, September 12th.

## Onboarding Timeline

### Meeting 1 - Saturday, Sep 5th

- Introduction to Lunabotics and its subteams
- Introduction to the structure of our code, ROS, and our development tools and
  workflow

### Meeting 2 - Tuesday, Sep 8th

- Start setting up development environments
- Create a Foxglove and GitHub account and join our organization on both (talk
  to me later or message me on Discord if you miss this day)

### Meeting 3 - Saturday, Sep 12th

- Write the effort factory node
- Start writing the drivetrain control node

### Meeting 4 - Tuesday, Sep 15th

- Finish writing both nodes and test in foxglove
- Run the simulator with navigation and play around with it

## Structure of our code

### Localization and Mapping

- The figures out where the robot is and what the environment around us looks
  like
- This outputs a 3d representation of the environment and where we are in it
- This also tells us where we are in relation to the arena
- We use RGBD SLAM (red-green-blue-depth \[cameras\] simultaneous localization and
  mapping) from the [rtabmap](https://github.com/introlab/rtabmap_ros) library
  - This is split into odometry, which looks at successive frames and can drift
    over time, and the slam itself which can recognize previous places and
    correct drift

### Perception

- This uses the data from mapping and raw data from sensors to determine where
  obstacles are
- We need to avoid walls, rocks, craters, and pillars
- We currently use another part of rtabmap to detect if a piece of ground is too
  sloped

### Navigation and Control

- This uses the data from mapping and perception to plan a path around obstacles
  to a target location and follows that path
- We use the [Navigation2](https://docs.nav2.org/) framework
- We currently use an STheta* planner
  - It is a variant of basic Theta*, which is a variant of A* (I will write docs
    on it, if you are reading this, yell at me)
- We currently use point to point to follow paths
  - Point at the next position in the path, drive to it, and repeat until you
    make it to the end

### Behavior

- This determines what the robot does and when
- Controls the cycles of the robot. Mine -> traverse to berm -> deposit ->
  traverse to excavation -> repeat
- This also handles failure conditions like motor stalls

### Embedded

- Communication with and code running on the teensy microcontroller
- Data is sent back and forth over USB
- This is a joint effort with Power and Hardware

## Frameworks, Libraries, and Tools

### What is ROS?

[ROS](https://docs.ros.org/en/jazzy/About-ROS.html) stands for Robot Operating
System, although it actually runs on Ubuntu Linux. ROS is a framework that
allows [nodes](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Nodes.html),
which are basically programs, to talk to other nodes through [topics, services,
and actions](https://docs.ros.org/en/jazzy/Concepts/Basic/Interfaces-Topics-Services-Actions.html).
Topics are channels where any number of publishers can send data to any number
of subscribers. Topics are identified by name, so you can have any number of
them. Services are like function calls. One node can call a service defined by
another node with a request and get a response. Services are also identified by
name, but not the node they are called on, so you cannot have two nodes that
define a service with the same name. Actions are built using a combination of
topics and services and are a long running version of services. An action is
called with a goal, which can be rejected by the action if, for example, the
node running the action is busy or the goal is invalid. The action then responds
with feedback, for example, the distance remaining to a goal. Finally, the
action produces a result. Additionally, the caller of the action can request
that the action is cancelled early.

ROS is intended to run on Ubuntu Linux. Each version of ROS is tied to
a specific version of Ubuntu, and new versions are released every year, with
long term versions being released every two years. This isn’t super important to
know but it will come up during the setup of the development environment.

ROS nodes are written in Python and C++. 3rd party libraries do exist for other
languages but we do not use them. The level of python that we use is relatively
basic, so knowledge in any language should be sufficient to write Python code.
C++ is  a little more complex but is only used when necessary for performance or
interoperability with libraries. If you have no coding experience, it is still
possible to help out through testing the robot or helping a group with research
for a project. I am unfortunately not able to teach an entire programming
language due to time and because I probably wouldn’t be very good at it.

### Why do we use ROS?

It allows the many components of our codebase to communicate easily
Nodes are in separate processes, so if one crashes, the rest don’t
There is a lot of tooling and libraries around ROS, so we don’t have to write as
much boilerplate code ROS communication works over networks so our mission
control laptop and robot(s) can easily communicate with each other.

### Development Environment and our Simulation

While ROS does technically support other operating systems, it is much easier to
standardize on one version of ROS, which is Jazzy in our case. In order to run
a specific version and configuration of ROS on every operating system we use OCI
containers, which is a generic term for Docker containers, as we don’t
necessarily use Docker on all platforms. We build a custom container image that
contains our code, some config, and our dependencies, which makes setup much
easier. This container image is hosted at
[ghcr.io/purduelunabotics/purdue_lunabotics](ghcr.io/purduelunabotics/purdue_lunabotics).
It is built for x64 and arm64. The most common setup is using devcontainers
through VS Code, or another [supported editor](https://containers.dev/supporting#editors)
(this list is not extensive, check your preferred editor). We provide a default
configuration for Windows, Linux (should work on all distros), and MacOS. 

Our full simulation is built with MuJoCo and does a full physics and visual
simulation of the arena with both robots. However, this simulation requires
quite a bit of resources, and cannot run on MacOS due to graphics limitations.
We also have a simplified sim which will be used during onboarding and whenever
the full sim is not needed. The simplified sim will be used for onboarding. 

In order to visualize things we use a combination of Foxglove, Rviz2, and RQt.
Rviz2 and RQt have historically been used for live visualization and Foxglove
for data analysis. However, Rviz and RQt rely on being able to create windows
from within docker, which does not work on MacOS, so we will use Foxglove for
onboarding and possibly as a full replacement of Rviz and Rqt.

### Git and GitHub

We use Git and GitHub to collaborate. Git is a version control system which
allows different developers to write and organize code in the same code base
then combine their code together when needed. The git tutorial is located
[here](https://git-scm.com/docs/gittutorial), and can also be opened in a Linux
terminal with `man gittutorial`. There is also a [list of everyday useful
commands](https://git-scm.com/docs/giteveryday) or `man giteveryday`. Git has
3 main areas, the working tree, the staging area, and the repository. The
working tree is the current state of the file system, the staging area is where
changes are moved to while they are being prepared for the repository, and the
repository is where changes are saved to by commits. The repository can then be
pushed to a host like GitHub. In order to get access to our repository, download
it using `git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git`.
Then make changes using your editor. When you are done making changes you can
move those changes into the staging area with `git add <the path to a file or directory
you want to stage>`. Finally commit with `git commit` which will open a window
that prompts for a commit message. Then to upload the code to GitHub, run `git
push`.  Code can be separated onto branches, so that commits onto a branch will
not appear on other branches. We use branches extensively so that different
groups don’t make conflicting changes. You can make a new branch with `git
branch <the name>` or switch to a branch with `git checkout <the name>`. GitHub
is where our repository is hosted ([link to
it](https://github.com/PurdueLunabotics/purdue_lunabotics)). Here you can make
a pull request to request that your branch is merged into the develop branch.
The develop branch is where code that is finished and works in simulation is,
and master is where code that works on the robot is.

### Starting the simulation

Once you have the repository cloned and the devcontainer running, you can build
the code with `colcon build --symlink-install` in the `/luna_ws` folder. Then
setup the install so ROS knows where to find the built code `source
/luna_ws/install/setup.bash`. This is done automatically when opening new
terminals after the first build. Then run the simulation with `ros2 launch
lunabot_bringup simple_sim.launch`. You can then open Foxglove and connect to
a live robot at the default URL.

## The Project

### Getting the robot to move in the sim

Follow the above steps to setup your development environment and switch to the
onboarding git branch. Then start the simple simulation and open Foxglove. Click
on open connection and connect with the default websocket url. Finally click on
the layout button in the top right and select onboarding under the organization
section. You should see the robot on a grid and two panels below it, if not,
please ask for help. The bottom left panel is a publish panel, which is set to
publish the robot’s effort. `/effort` is a topic that is read by the sim or the
microcontroller (through a bridge node) and it controls the robot's movement.
The most important fields we care about right now are the left drive, and right
drive. Try setting each of those fields in the publish panel to 1000 and
pressing publish. To stop the robot, set each field back to zero and publish
again. This is the core of how the robot moves, but there are some problems with
this approach. 

### Creating an effort factory

#### Creating a basic node

Anything that wants to control the robot needs to control everything, there is
no way to just set one motor. I will explain why this is such a problem later.
For now, we will make a node that reads from several topics and combines them
into one effort message. To do that we need to make a node. This node will be
written in python in the lunabot_onboarding package. Create
`lunabot_onboarding/lunabot_onboarding/effort_factory.py` in the
`purdue_lunabotics` folder. Nodes are created by inheriting from
`rclpy.node.Node`.

```python
from rclpy.node import Node

class EffortFactory(Node):
    # this is the constructor
    def __init__(self):
        # call the node constructor and set the nodes names
        super().__init__("effort_factory")
```

Now that we have created a very basic node, we need a way to run it. For that
we will add a main function to the effort factory file.

```python
import rclpy

def main():
    # setup the ros runtime
    rclpy.init()
    effort_factory = EffortFactory()

    # Tells the runtime to wait and process messages and timers for this node
    # until the node is stopped
    rclpy.spin(effort_factory)
    rclpy.shutdown()
```

#### Running the node

Now we need a way to actually run this file. For this we need to tell ros about
our executable. Open `lunabot_onboarding/setup.py` (it already exists). Find the
line at the bottom that has the list of entry points and add
`'effort_factory_node = lunabot_onboarding.effort_factory:main'` to the console
scripts list. The file should look something like this.

```python
from setuptools import find_packages, setup

package_name = 'lunabot_onboarding'

setup(
    # ... Leave all of these options the same.
    entry_points={
        'console_scripts': [
            'effort_factory_node = lunabot_onboarding.effort_factory:main',
        ],
    },
)
```

You can now run the node by calling `colcon build --symlink-install` again then
running `ros2 run lunabot_onboarding effort_factory`. We haven't actually made
the node do anything though, so nothing will happen. 

#### Creating the subscriptions

Now we need to add some subscribers so we have individual topics for both sides.
We'll make `/right_drive` and `/left_drive` which maps to the left drive and
right drive settings in the effort factory. In the node's init function, we will
call `self.create_subscription` with the type of the message, the topic name,
the callback for when a message is received and a quality of service. Quality of
service has several options about how ROS should send messages around, for now
we will set it to the default with a 10 message buffer. Add the following to `__init__`:

```python
self.left_drive_sub = self.create_subscription(Int32, "left_drive",
    self.left_drive_cb, 10)
```

We are using `Int32` as the type, so we must import it with `from std_msgs.msg import Int32`.
We also need to create the callback, so add the following into the class:

```python
def left_drive_cb(self, left_drive: Int32):
    # Int32 is not actually a number, so we must access the data inside
    self.left_drive = left_drive.data
```

We are just going to save the left drive into a variable. Make sure you
initialize the variable in the `__init__` function.

Your code should now look something like this:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class EffortFactory(Node):
    def __init__(self):
        super().__init__("effort_factory")

        self.left_drive = 0
        self.left_drive_sub = self.create_subscription(Int32, "left_drive",
            self.left_drive_cb, 10)

    def left_drive_cb(self, left_drive: Int32):
        self.left_drive = left_drive.data
        
def main():
    rclpy.init()
    effort_factory = EffortFactory()
    rclpy.spin(effort_factory)
    rclpy.shutdown()
```

Now repeat these steps for the right drive.

#### Creating the publisher

We now need to create an effort publisher. This is done with the
`self.create_publisher` function with the same arguments as creating
a subscription except without a callback. Add the following to `__init__`:

```python
self.effort_pub = self.create_publisher(RobotEffort, "effort", 10)
```

We also need to import our effort type from `lunabot_msgs`, the same way we
imported `Int32`.

#### Creating the loop

Now we actually need to use this publisher by publishing effort at a constant
rate. Almost everything on the robot is done in a loop, so this is a pattern you
will become very familiar with. Create a timer with `self.create_timer` with the
period and a callback as the arguments:

```python
self.timer = self.create_timer(0.1, self.loop) # 10 hertz
```

Now we need to make a callback that creates an effort message and publishes it:

```python
def loop(self):
    effort = RobotEffort() # create an empty message
    effort.left_drive = self.left_drive
    effort.right_drive = self.right_drive
    self.effort_pub.publish(effort)
```

Now when we run the node we can publish to left and right drive and the robot
moves! (Use the individual tab above the effort publisher)

### Creating a drivetrain controller

Right now we are publishing direct motor speeds to control our robots velocity,
but if we wanted a more generic way to control the robot, like drive forward at
1 m/s second we would have to write more code every time. A main benefit of ROS
is that we can use other libraries to make our code simpler, and other libraries
don't know our specific setup so they don't know how to control our robot. To
solve this issue, it is standard to have a topic called `cmd_vel` that is used
to control the movement of the robot. We are going to create a node that
subscribes to `cmd_vel` and publishes our left and right velocities. The
instructions for creating this node will be a lot less in depth, so refer to the
previous steps or ask for help if you need it.

#### What is command velocity?

Command velocity is a `geometry_msgs/msg/Twist`. A twist is used to represent
the velocity of something. You can view the structure of any message with the
command `ros2 interface show <the type>`. The output for a twist is shown below.

```
# This expresses velocity in free space broken into its linear and angular
    parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
```

Twists can represent the velocity of any object in 3d space, so it has
6 dimensions. However, our robot only has 2 dimensions of control, forward and
turn. The x axis is forward (as defined in the below standard of ROS standard
units), so we will use `linear.x` for our forward and back velocity. The z axis
is up, so we will use `angular.z` for our rotation.

Everything in ROS uses SI units ([ROS standard
units](https://www.ros.org/reps/rep-0103.html)), so the linear components are in
meters per second and the angular components are in radians per second.

#### Creating another node

Start by following the instructions in [Creating a basic node](#creating-a-basic-node) to make a node
that's called `DriveController`.

#### Create the publishers and subscriptions

Like in [Creating the subscriptions](#creating-the-subscriptions) and [Creating the publisher](#creating-the-publisher), we need to
subscribe to `cmd_vel` and publish to `left_drive` and `right_drive`. Just like
we did previously, store `cmd_vel` in a local variable.

#### Defining parameters

The first thing we want to figure out is the unit conversions. We want to be
able to transform meters per second across the ground to motor rpm.

$$
\frac{1\text{ rotation}}{1\text{ minute}} = \frac{1\text{ meter}}{1\text{ second}} * \frac{60\text{ seconds}}{1\text{ minute}} * \frac{1\text{ rotation}}{2 \pi r\text{ meters}} * \text{gear ratio}
$$

This means the conversion factor is

$$
\frac{60i}{2 \pi r}
$$

Where $r$ is the wheel radius and $i$ is the gear ratio. Lets make a simple
conversion function in our drive controller class to converts from meters per
second to rpm using this formula. The only problem we have now are these two
constants in our code. If we were to hard code these numbers, it would cause
a major issue for our two robots because they both use a drive controller node,
but they have different constants. ROS provides something called
[parameters](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html)
which makes dealing with constants like these in our code much easier.

To use a parameter we need to first declare it. This is usually done in the
class constructor with the `declare_parameter` function which takes the
parameter name and a default value (and optionally some other things we won't
use right now) and returns a `Parameter` object. To get the value we call
`get_parameter_value` on the returned object and get the field corresponding to
the type of parameter (eg `double_value` for doubles). Here are the 3 parameters
(we will use the chassis width later) we need with their default values:

```python
self.gear_ratio = self.declare_parameter("gear_ratio", 50.0)
    .get_parameter_value().double_value
self.wheel_radius = self.declare_parameter("wheel_radius", 0.2)
    .get_parameter_value().double_value
self.chassis_width = self.declare_parameter("chassis_width", 0.64)
    .get_parameter_value().double_value
```

We can now make our completed function to convert units. I called my function
`speed_to_rpm` but you can use something else if it makes more sense to you.

#### Implementing the logic

Like in [Creating the loop](#creating-the-loop), make another timer and loop callback. You can use
the same 10 hertz period. The logic in this node is a little more complicated
than the effort factory because we need to transform the linear x and angular
z commands into a left and right command. We can start by imagining the simplest
case, where linear x is non zero and angular z is zero. This means we just pass
the linear x value in to both the left and right speeds after using our
conversion function. For example:

```python
left_drive = self.cmd_vel.linear.x
right_drive = self.cmd_vel.linear.x

self.left_drive_pub.publish(Int32(data = int(self.speed_to_rpm(left_drive))))
self.right_drive_pub.publish(Int32(data = int(self.speed_to_rpm(right_drive))))
```

The angular rotation is slightly more complicated as we need a way to convert
between radians per second and meters per second, so we can use our conversion
function. If our input angular rotation was $2\pi$ radians per second, the robot
should complete one full rotation per second, which means the wheels will
complete a full circle every second. The circumference of the circle would be
$2\pi r$ where $r$ is half of the width of the chassis (kind of like the radius
of the chassis). $2\pi$ cancels out so we just multiply the angular velocity by
$\frac{\text{width}}{2}$ to get the linear velocity. A positive angular velocity
means the robot rotates counter-clockwise, so left should be negative, and right
is positive. For example:

```python
left_drive = -self.cmd_vel.angular.z * self.chassis_width / 2
right_drive = self.cmd_vel.angular.z * self.chassis_width / 2

self.left_drive_pub.publish(Int32(data = int(self.speed_to_rpm(left_drive))))
self.right_drive_pub.publish(Int32(data = int(self.speed_to_rpm(right_drive))))
```

Now to combine those two together, it's as simple as adding the two components
together. This gives us:

```python
left_drive = self.cmd_vel.linear.x
    - self.cmd_vel.angular.z * self.chassis_width / 2
right_drive = self.cmd_vel.linear.x
    + self.cmd_vel.angular.z * self.chassis_width / 2

self.left_drive_pub.publish(Int32(data = int(self.speed_to_rpm(left_drive))))
self.right_drive_pub.publish(Int32(data = int(self.speed_to_rpm(right_drive))))
```

We are now ready to test this node in the simulator! Run the simulator and both
nodes, then you can use the teleop controls in the bottom right to drive the
robot around using `cmd_vel`. Test your conversion factors by making sure that
if you drive forward at some speed for some time you travel the correct distance.

### Launch files

Both of the nodes should work at this point, but running them both separately is
a pain, so it would nice if we could run command to run both at the same time.
This can be accomplished with [launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).
You can write launch files in XML, Python, or YAML, but we only use XML and
Python. XML is simpler and shorter for easy stuff but Python is much more
powerful. We will cover the basics of both for this.

#### Create the launch file

Create the file `launch/onboarding.launch.py` in the `lunabot_onboarding`
directory. Then add the following empty launch description:

```python
import launch

def generate_launch_description():
    return launch.LaunchDescription([ ])
```

We also need to tell ROS where our launch files are, so add the following into
the data files section of `setup.py`:

```python
import os.path # add the imports to the beginning
from glob import glob

data_files=[
    # ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
],
```

This only needs to be done once and all the launch files will work. Try it by
rebuilding and running `ros2 launch lunabot_onboarding onboarding.launch.py` 

#### Adding our nodes

We need to actually add our nodes to our launch file, which is done with the
node launch action:

```python
import launch_ros # add this to the top

# add this to the launch description list
launch_ros.actions.Node(
    package='lunabot_onboarding',
    executable='effort_factory_node',
)
```

There are many more options available when launching nodes like display name
namespace, parameters, and topic remapping, for example. Now add the drive
controller to the launch file as well and test with the sim.


#### Combining launch files

We can also add the onboarding launch file to our simple sim launch file so we
only need to run one command. Edit the
`lunabot_bringup/launch/simple_sim.launch` file, which is in XML (a generic form
of HTML). Add the following line into the `launch` element:

```xml
<include file="$(find-pkg-share lunabot_onboarding)/launch/onboarding.launch.py" />
```

The `$(find-pkg-share lunabot_onboarding)` part calls the builtin find-pkg-share
function that finds where a given package is installed. Now we can start
everything just by running the sim!

### Why did we set it up this way?

There are a lot of ways we could have organized these nodes, so why did we do it
like this? Because these two nodes are critical for controlling any movement of
the robot, we want them to be both simple and able to handle any situation. Both
of the nodes are quite short, and easy to read. They are also very easy to
interact because they use standard topics that don't have extra data. They are
also very powerful as you can control the robot however you want and publish to
any part of effort you want. This is important because we don't want to have to
edit effort to get the behavior we want.
