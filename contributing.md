# Contributing!

### Branch Naming Conventions

#### Permanent Branches
- `master`
  - Represents current working state of the repo
- `develop`
  - Used for integration of features and individual contrbutions

#### Temporary Branches

- `feature/feature-name`
  - Use this for general (long-term) features that **multiple people** will touch (i.e `feature/graphslam`)
- `user/your-gh-username/<contribution-name>`
  - Use this for small bug fixes and **individual** contributions (i.e `user/johncena/quick-encoder-fix`)

For all temporary branches:
- Open PR to `develop` or respective `feature` branch, not `master`

### General Individual Development Workflow
1. Create new branch from:
   - `feature` branch that you're working on (optional, create new `feature` branch if adding to new feature)
   - `develop` if quick fix or individual contribution
2. Develop the feature/change
3. Open Pull Request on Github merging to `feature` branch or `develop`
4. Request review on Github from leads
5. Make any needed changes (The code should completely work)
6. Request final review on Github
7. Merge the pull request on Github

# Coding Standards

Keep a consistent standard within the same file, and ideally, within the same package with files of the same type. Hopefully don't write code that is hard to read.

#### Python Source
Try to adhere to [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)

TLDR:
- Use type hinting for every function and class variable

#### C++ Source
Similar to python except [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)

## ROS 2 Tips

### Workspace Layout
```
- ros2_ws
  - src
    - purdue_lunabotics
    - any other ROS2 projects
  - install (automatically generated on build)
  - log (automatically generated on build)
  - build (automatically generated on build)
```
When building the packages in your workspace, always call the build command from the `ros2_ws` directory. This will prevent the install, build, and log directories from being generated in arbitrary places in your project.

### Package Layout
```
- lunabot_PACKAGE
  - include
     - lunabot_PACKAGE
        - *.h header files
  - scripts
     - Python ROS Node executables (use *.py extensions)
  - lunabot_PACKAGE
     - __init__.py (empty)
     - python source files (.py), imported by executables in "scripts"
  - src
     - *.cpp (source files and nodes)
  - package.xml
  - CMakelists.txt
```
Use `ros2 pkg create <name>` to create a new package. 

### Adding a C++ node:
- Place cpp and h files as specified in the package.
- Make sure `package.xml` contains 
``` 
<depend>rclcpp</depend>

<buildtool_depend>ament_cmake</buildtool_depend>
```
as well as any other dependencies for your C++ code.
- In CMakeLists, make sure it contains
```
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
```
and any other dependencies.
Define your executable (node) with 
```
add_executable(<name> src/<x>.cpp, src/<y>.cpp ...)
ament_target_dependencies(<name> <d1> <d2> ...)
```
where *name* is the name of the node, *x*, *y*, etc are the source C++ files to compile, and *d1*, *d2*, etc are the dependencies.
- Install the executable with 
```
install(TARGETS 
        <name>
        DESTINATION lib/${PROJECT_NAME})
```

### Adding a python node:
- Place a .py file in the scripts directory. This should have a defined `main` function and have the `#!/usr/bin/env python3` line at the top.
- Make sure `package.xml` contains 
``` 
<depend>rclpy</depend>

<buildtool_depend>ament_cmake_python</buildtool_depend>
```
as well as any other dependencies for the python code.
- In CMakeLists, make sure it contains
```
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
```
and any other dependencies.
- Set up a python package with 
```
ament_python_install_package(${PROJECT_NAME})
```
- Install the executable with
```
install(PROGRAMS
        scripts/<name>.py
        DESTINATION lib/${PROJECT_NAME})
```

### Adding new messages:
- Add a new `.msg` file to `lunabot_msgs/msg` with the datatypes/names (see current examples).
- Add any dependencies (for example, on other complicated messages types) to `package.xml`, inside a `<depend>  </depend>` block.
- Add these dependencies to `CMakeLists.txt` using `find_package(<dependency> REQUIRED)`
- Add your new message to the 
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotEffort.msg"
  "msg/RobotSensors.msg"
  <etc...>
  DEPENDENCIES std_msgs
)
```
block. Put any dependencies next to `DEPENDENCIES`.
