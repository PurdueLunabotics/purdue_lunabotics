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
<tag atrribute="value" speed="fast">
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

### Important Tags

The most basic tag is `<launch>`. This is simply a marker that the file is a launch file and should surround the entire file.

Next is `<node>`. This defines a node that should be launched by your file. 