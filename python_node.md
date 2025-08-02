# Python Node

## PID Controller

This project will have you make a PID Controller in a Python Node. 
PID is a control method that determines the speed of the robot using 3 formulas.

### Proportional
- The error is directly multiplied by a constant to get this portion of the output speed

### Integral
- The error is added up over time (integrated) and then multiplied by a constant

### Derivative
- The rate of change of the error is multiplied by a constant.

Add the 3 formulas together and you get the output speed

More information about PID can be found [here](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid)

## How do you code a Python ROS Node?

### First Steps
- To allow a python file to be executable, you must add `#!/usr/bin/env python3` as the first line.
- rclpy is the class that contains all ROS commands in python `import rclpy` 

### Creating the class
- Here is an example class definition.
  ```py
  class Name(InheritFromClass):
  ```
  - First you need the class keyword
  - Then the name of the class. This should be styled in TitleCase
  - In parentheses are any classes that this class inherits from
- All Python Nodes should inherit from the `rclpy.node.Node` class
- Every function within the class must have a first parameter of `self`.

### __init__ function
- This will be called when you first create the node.
- Since we are in a class, every function must have a first parameter of `self`.
- Next we also need to allow for variable length keyword arguments so add `**kwargs` as a parameter at the end.
  - This probably won't be used, but is best practice.
  - **kwargs must be at the end of the parameter list because it allows for "infinite" arguments, meaning anything after it would never get assigned.
- In the init function we need to tell the parent class (Node) to initialize and tell it what the name of our node should be. 
  - To achieve this we need to first get the parent object and then call it's `__init__` function
  - To find the parent object we will use the `super()` function. 
  - The two arguments you need to pass to the init function are the name of the node as a String and any keyword arguments that were passed into your node
- Now let's define the subscriptions that we will need.
  - We do this through calling `self.create_subscription()`
  - You need to pass 4 arguments to the function
    - The message type of the subscription (you will need to import the correct object for this)
    - The topic as a string
    - The callback function that will be called when a new message is available
    - Quality of Service - this defines the minimum quality of the subscription that the node is willing to deal with. 
      - You can either pass a QoS profile object or an int representing the queue size.
        - Since python nodes will only update callbacks when you allow them to, it is possible for many messages to build up in that time. If you just looked at them in the order they appear, you would end up with out of date data.
        - To fix this we define a queue size and only keep the last x number of messages.
      - Learn more [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html). 
  - For this node we need 2 subscriptions
    - Goal Location
      - Type: `geometry_msgs.msg.PoseStamped`
      - Topic: `/goal`
      - Callback Function: You define this (later)! 
      - QoS: 1 - we always want the most recent data
    - Current Location
      - Type: `nav_msgs.msg.Odometry`
      - Topic: `/odom`
      - Callback Function: You define this (later)! 
      - QoS: `1` - we always want the most recent data
- Next let's define the publisher we will output.
  - We can create this object by calling `self.create_publisher()`
  - You need to pass 3 arguments to the function
    - The message type of the subscription (you will need to import the correct object for this)
    - The topic as a string
    - Quality of Service - this defines the maximum quality of the publisher that the node can output. 
      - You can either pass a QoS profile object or an int representing the history depth.
        - Nodes can subscribe to a topic at any point, so sometimes you will have a node join the topic after you have started publishing to it, in that case the publisher will send the last x number of messages to that node based on the history depth.
        - Note: This is the maximum number that may be sent, if a subscriber asks for less than this only that number will be sent
      - Learn more [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html). 
  - For this node we need only 1 publisher
    - Commanded Velocity
      - Type: `geometry_msgs.msg.Twist`
      - Topic: `/cmd_vel`
      - QoS: `10` - This doesn't *really* matter as we will not be running this node until after the simulation fully starts, 10 is a standard value.
- That concludes your `__init__` function, you may want to add additional variables and constants here, but we will leave that up to you

### Callback Functions
- The name of callback functions should be `__name_callback` where the name is the thing that this callback is for. 
  - The two underscores at the beginning is conventionally used to signify this is a private function and shouldn't be called outside this class (though nothing stops someone from doing so).
- Callbacks should have two parameters
  - First is the self parameter needed for all members of a class
  - Second is the message that you should get from the subscription. You should define this to be the type you expect.
    - Defining types of parameters in python is done like so `msg: Type`
- Within the function you should save any information you need from the message as it will not be saved.

### Node Logic
- At this point you should write all the logic needed for your node.
- To publish you call `publisher.publish(msg)` where publisher is the object from the init function and msg is a message of the type defined above
- There are two options for style of Node.
  - The recommended option is to do all processing starting in callback functions and have no loop.
    - The idea behind this is that if nothing changes in your callbacks, the node doesn't need to be processing anything
  - Another option is to have a main loop that pauses occasionally to get callbacks. 
  - Either option works, it is mostly a matter of preference and code complexity.
- The goal and odom are 3D points, you only need to worry about a single dimension.
  - You need to figure out which dimension is forwards/backwards

#### Main Loop Only 
- This is only applicable if you chose to have a main loop.
- You should have a loop within your code that will run forever while the node is active. This will handle all the processing and publishing of outputs.
- To keep the node running while it is active, use the `rclpy.ok()` function.
  - This will return True while the node is active and false once it is killed.
- After you do your processsing, you should call `rclpy.spin_once(self, timeout_sec=0)`
  - This will call your callback functions

### Main Function
- This applies to either approach
- You need to define a function outside of your class that will be run when the node first starts.
- This function needs no parameters
- First you need to initialize ROS by calling `rclpy.init()`
- Next you need to create an object of your class.
- If you chose to do callback processing only
  - You can now run `rclpy.spin()`
    - This will allow ROS to respond to new messages by calling the callback functions and should run your node
    - Once you tell ROS to shutdown, it will stop looking for callbacks and go through any code after it (we won't have any so it'll exit the file)
- If you chose to have a main loop
  - You can call the main loop function of your class

### Setup.py
- To make your node able to be executed you must go into the setup.py file in the package you are in.
- At the end of the file there is `entry_points`.
- Within console scripts, you should add your node's name followed by the path to the main function of your node
  ```
  example_node = lunabot_package.example_code:main
  ```
- Now you can test your node by running ros2 run \<lunabot_package> \<example_node>