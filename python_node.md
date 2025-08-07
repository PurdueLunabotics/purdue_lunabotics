# Python Node

## PID Controller

This project will have you make a PID Controller in a Python Node. 
PID is a control method that determines the speed of the robot using 3 formulas.

### Proportional
- The error is directly multiplied by a constant to get this portion of the output speed
- This will get to the target quickly but will oscillate 

### Integral
- The error is added up over time (integrated) and then multiplied by a constant
- If you are close to the target, this will help get to the final value

### Derivative
- The rate of change of the error is multiplied by a constant.
- This will slow you down as you get to the target to reduce oscillations

Add the 3 formulas together and you get the output speed

More information about PID can be found [here](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid) and [here](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Fundamental_operation)

## How do you code a Python ROS Node?
The following steps will guide you through the creation of a python node in ROS2. If you find yourself needing more information/documentation, [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) is the ROS2 documentation for writing a Python node.

### First Steps
- Open pid_control.py in lunabot_control/src/lunabot_control
- To allow a python file to be executable, you must add `#!/usr/bin/env python3` as the first line.
- rclpy is the class that contains all ROS commands in python, add `import rclpy` to the code.
- You will need to import more things as we add more code, you should keep all imports in the this spot 

### Creating the class
- A python class is a collection of variables and functions
- Here is an example class definition.
  ```py
  class Name(InheritFromClass):
  ```
  - First you need the class keyword
  - Then the name of the class. This should be styled in TitleCase
  - In parentheses are any classes that this class inherits from
- All Python Nodes should inherit from the `rclpy.node.Node` class
  - Inheriting is a way for us to add functionality to our class without rewriting the code.
  - You can import the Node class by using the from keyword. `from rclpy.node import Node` 
- Every function within the class must have a first parameter of `self`.
  - This allows us to use the self object later on to create variables that don't get removed at the end of a function and can be accessed from anywhere in the code.

### Creating functions
- Functions in python are defined with 4 parts
  - The `def` keyword indicates that you are writing a new function rather than calling an existing one
  - Next we have the name of the function.
    - Functions with two underscores at the beginning is conventionally used to signify this is a private function and shouldn't be called outside this class.
    - Functions with two underscores on both sides of the name are generally system functions that are called automatically, such as `__init__` and `__str__` which initalize an object and convert it to a string respectively.
  - Then we have the parameters of the function within parentheses.
    - These are the inputs to the function.
    - Parameters can be named anything, but are typically using snake_case and cannot start with a number or special character
    - If we want to indicate that a parameter should be a specific type we can do that by putting a colon and then the type
    - If we want to have optional parameters with a default value, we can do that by putting an equals sign and the default value.
      - Note: optional parameters must be after all required parameters
    - If we want to allow a variable length of arguments, we can put an asterisk before the parameter name
      - This will create a list of the arguments that are passed into the method
    - If we want people to be able to define any keyword arguments, we can put two asterisks before that parameter
      - This will create a dictionary of the keyword arguments that are passed into the method.
  - Finally we have a colon to show that the following indented code is a part of the function
  ```py
  def example_function(required_param, string_param: String, optional_param=None, *variable_args, **keyword_args):
  ```

### \_\_init\_\_ function
- This will be called when you first create the node.
- The first parameter will be `self`.
- Next we also need to allow for variable length keyword arguments so add `**kwargs` as a parameter at the end.
  - This probably won't be used, but is best practice.
- Here is an example init function definition.
  ```py
  def __init__(self, **kwargs):
  ```
- In the init function we need to tell the parent class (Node) to initialize and tell it what the name of our node should be. 
  - To achieve this we need to first get the parent object and then call it's `__init__` function
    - To find the parent object we will use the `super()` function.
  - The two arguments you need to pass to the init function are the name of the node as a String and any keyword arguments that were passed into your node
  ```py
  super().__init__("example_node",**kwargs)
  ```
- Now let's define the subscriptions that we will need.
  - We do this through calling `self.create_subscription()`
  - You need to pass 4 arguments to the function
    - The message type of the subscription (you will need to import this)
    - The topic as a string
    - The callback function that ROS will call when a new message is available
    - Quality of Service - this defines the minimum quality of the subscription that the node is willing to deal with. 
      - You can either pass a QoS profile object or an int representing the queue size.
        - Since python nodes will only update callbacks when you allow them to, it is possible for many messages to build up in that time. If you just looked at them in the order they appear, you would end up with out of date data.
        - To fix this we define a queue size and only keep the last x number of messages.
      - Learn more [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html). 
  - For this node we need 2 subscriptions
    - Goal Location
      - Type: `geometry_msgs.msg.PoseStamped`
      - Topic: `/goal`
      - Callback Function: Leave blank for now 
      - QoS: 1 - we always want the most recent data
    - Current Location
      - Type: `nav_msgs.msg.Odometry`
      - Topic: `/odom`
      - Callback Function: Leave blank for now 
      - QoS: `1` - we always want the most recent data
  - Here is an example subscription:
  ```py
  self.create_subscription(Type, "topic", self.__example_callback, 1)
  ```
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
  - Here is an example subscription:
  ```py
  self.example_publisher = self.create_publisher(Type, "topic", 10)
  ```

That concludes your `__init__` function, you may want to add additional variables and constants here, but we will leave that up to you

### Callback Functions
- The name of callback functions should be `__name_callback` where the name is the thing that this callback is for. 
- Callbacks should have two parameters
  - First is the self parameter needed for all members of a class
  - Second is the message that you should get from the subscription. You should define this to be the type you expect.
    - Defining types of parameters in python is done like so `msg: Type`. 
    - This will indicate to your code editor (and other members) that the variable is a specific type and specific methods can be called on it.
- Within the function, you should save any information you need from the message. Once the function ends, any unsaved information will go away
- Make sure to add this to your create_subscription in init.

### Main Function
- There are two options for style of Node.
  - The recommended option is to do all processing starting in callback functions and have no loop.
    - The idea behind this is that if nothing changes in your callbacks, the node doesn't need to be processing anything
  - Another option is to have a main loop that will continuously process and publish output at a specified rate. 
  - Either option works, it is mostly a matter of preference and code complexity.
- You need to define a function outside of your class that will be run when the node first starts.
- This function needs no parameters
- First you need to initialize ROS by calling `rclpy.init()`
- Next you need to create an object of your class.
- If you chose to do callback processing only
  - You can now run `rclpy.spin()`
    - This will allow ROS to respond to new messages by calling the callback functions and should run your node
    - Once you tell ROS to shutdown, it will stop looking for callbacks and go through any code after it (we won't have any so it'll exit the file)
- If you chose to have a main loop
  - You need to tell your node to execute callbacks in the background. To do this you need to start another thread for the callbacks
  - First we will make a function that will tell ros to respond to callbacks.
    - In this function we need to get the global executor, which deals with executing the code by running `rclpy.get_global_executor()`
    - Then we need to run that object's `spin()` function.
    - When the program exits, this function will return an error so we should encase it within a try/except block.
      - Try Excepts will try to run the first section of the code and if there's an error, go to the second section.
      ```py
      try:
        function_might_return_error()
        print("no error")
      except:
        print("error happpened")
      ```
      - Since we don't want anything to happen when the error occurs, we can just put the keyword `pass`, which python will ignore, in the except block.
    - Now going back to the main function, we need to start another thread. This can be done by using the `threading.Thread` class.
      - You need to pass the function you just created into the constructor
      ```py 
      thread = threading.Thread(target=cool_multi_threaded_function)
      ```
      - Next you have to run the `start()` function on that thread.
    - Finally, we need to tell our node to also run using the global executor.
      - Back in the `__init__` function, below the call to `super()`, we need to add the following line
      ```py
      rclpy.get_global_executor().add_node(self)
      ```
      - Since the class we are making inherits from Node, we can tell it to add itself as a node to the executor.
    - Now we will be able to get callbacks while also running our processing.
  - Finally, you need to call your class's main loop function using the object you created before.



### Node Logic
- At this point you should write all the logic needed for your node within this class.
- To publish you call `publisher.publish(msg)` where publisher is the object from the init function and msg is a message of the type defined above
- The goal and odom are 3D points, but you only need to worry about a single dimension.
  - You need to figure out which dimension is forwards/backwards
- If you want a hint on how to write this code [here is some pseudocode](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Pseudocode)

#### Message Types
- PoseStamped
  - Message Definition [here](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html)
  - You only need to care about msg.pose.position
- Odometry
  - Message Definition [here](https://docs.ros2.org/foxy/api/nav_msgs/html/msg/Odometry.html)
  - You only need to care about msg.pose.pose.position
- Twist
  - Message Definition [here](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)
  - You only need to care about msg.linear

#### Main Loop Only 
- This is only applicable if you chose to have a main loop.
- You should have a loop within your code that will run forever while the node is active. This will handle all the processing and publishing of outputs.
- To keep the node running while it is active, use the `rclpy.ok()` function.
  - This will return True while the node is active and false once it is killed.
- In the begining of this function you should define a rate object. This will ensure that your loop runs at a maximum of the specified times per second.
  - If you do not do this, it will attempt to run as fast as possible, which may waste resources
  - If the code takes a long time to run, you may run slower than the specified rate.
  - You can create your rate object by using a function that was inherited from the Node class
    ```py
    rate = self.create_rate(self.FREQUENCY, self.get_clock())
    ```
- After you do your processsing, you should call `rate.sleep()`
  - This will pause your program to maintain the rate.


### Setup.py
- To make your node able to be executed you must go into the setup.py file in the package you are in.
- At the end of the file there is `entry_points`.
- Within console scripts, you should add your node's name followed by the path to the main function of your node
  ```
  example_node = lunabot_package.example_code_file:main
  ```
- Now you can test your node by running `ros2 run <lunabot_package> <example_node>`