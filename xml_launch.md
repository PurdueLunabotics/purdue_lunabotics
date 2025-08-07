# XML Launch Files

## What are Launch Files?

Launch Files help to start nodes automatically. We use them to reduce the amount of work we need to do to start the robot. They can also configure nodes and respond to arguments that you pass to them. Launch files can be written in 3 languages: XML, YAML, and Python. This file focuses on XML launch files. See [here](python_launch.md) for python launch file information.

## Why XML?

XML is used because it is a fairly user friendly and simple language to write in. This is the least verbose option, but might not have as many features as python as it is not a scripting language.

## Components of an XML file

XML consists of a variety of tags that are used to define what should be run.

Tags consist of 4 elements
- Open angle bracket indicates the start of a tag
- Text inside defines what the tag is 
- Keyword Attributes add further specifics about that tag
- Close angle bracket indicates the end of the tag
```xml
<tag atrribute="value" speed="fast"/>
```

Tags must be opened and closed to define a region that is enclosed by the tag.
```xml
<tag>

</tag>
```

They can also be self-closing if they do not need any other tags within them
```xml
<tag/>
```
### Substitutions

Sometimes you want to use variables and file paths in your launch files. To do this you will need to use substitutions. They are defined by using the following notation: `"$(keyword name)"`. Within the parentheses, you put the keyword followed by the name. There are two main keywords that are used:
- var: This is for a variable. These are defined with `<arg>` and `<let>` tags. 
- find-pkg-share: This is for finding the directory of a package. Mostly used for finding launch files or parameter files.

### Important Tags

#### You will not need all of this information to write the onboarding launch file, this is a reference for future work as well.

The most basic tag is `<launch>`. This is simply a marker that the file is a launch file and should surround the entire file.

Next is `<node>`. This defines a node that should be launched by your file. 

Important Attributes: __(*=required)__
- pkg*: The package that the node is in
- exec*: The name of the node that should be run
- respawn: Should the node start again if it crashes
- output: where should console output go from this node
  - "screen": goes to the terminal
  - "log": goes to a log file
- name: If you want to change the name of the node from exec
- namespace: acts as folders for the nodes, every node needs to have a unique path
  - This allows you to have multiple nodes with the same name as long as they are in different namespaces

Within a node you can have two tags, first is `<param>`. This defines a parameter that should be sent to the node.

Important Attributes: Either need name and value or from 
- name: The name of the parameter, must match with the name within node's code
- value: the value of the parameter, must have the same type as the node expects. See more info about types [here](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html#type-inference-rules)
- from: A file to import parameters from. 

The other tag you can use within a node is `<remap>`. This will change the topic that the node is looking for or outputting to.

Important Attributes:
- from: The name of the topic that the node expects.
- to: The name of the topic that you want it to use instead.

Another thing you will need to do is combine multiple launch files to have them all start at the same time.
You can do this by using `<include>`.

Important Attributes:
- file: the file path to the launch file to be included
  - see above for information on substitutions.

## Writing your launch file.

- Go to `pid_control.launch` in `lunabot_control/launch`. 
- The first thing we need to do is to define this file as an xml file. This is done through the following tag.
```xml
<?xml version="1.0" encoding="UTF-8"?>
```
- Next let's put an open and closed launch tag, We will put all of our stuff inside them.
```xml
<launch>
  CODE HERE
</launch>
```
  - Every time we put a tag within another one, we are going to indent.
- Now we can make our node, put a self-closing node tag and add the `name`, `exec`, and `pkg` attributes as you see fit.
- That is all we need to do in this file, let's go to `sim.launch` in `lunabot_bringup/launch`. 
  - This file is the main launch file for the simulation, so we need to include the launch file we just made, so that it will also run.
- At the bottom before the close launch tag, we will add an `include` tag.
  - Add the `file` attribute, making sure to use the `$(find-pkg-share)` substitution.
- That's all you have to do, test it out!