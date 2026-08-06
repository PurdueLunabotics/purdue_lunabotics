cmake_minimum_required(VERSION 3.8)
project(lunabot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simulator src/simulator.cpp)
add_executable(simple_simulator src/simple_simulator.cpp)
target_link_libraries(simulator mujoco::mujoco glfw)

ament_target_dependencies(simulator rclcpp glfw3 lunabot_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(simple_simulator rclcpp lunabot_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mujoco")

set(GENERATED_ALL_SCENE "${CMAKE_CURRENT_BINARY_DIR}/scene_all.xml")
set(GENERATED_JUST_MAIN_SCENE "${CMAKE_CURRENT_BINARY_DIR}/scene_just_main.xml")

add_custom_command(
  OUTPUT "${GENERATED_ALL_SCENE}"
  COMMAND xacro "${MUJOCO_DIR}/scene.xacro.xml" two_bots:=true > "${GENERATED_ALL_SCENE}"
  DEPENDS "${MUJOCO_DIR}/scene.xacro.xml" "${MUJOCO_DIR}/main_bot.xacro.xml" "${MUJOCO_DIR}/mini_bot.xacro.xml"
)
add_custom_command(
  OUTPUT "${GENERATED_JUST_MAIN_SCENE}"
  COMMAND xacro "${MUJOCO_DIR}/scene.xacro.xml" two_bots:=false > "${GENERATED_JUST_MAIN_SCENE}"
  DEPENDS "${MUJOCO_DIR}/scene.xacro.xml" "${MUJOCO_DIR}/main_bot.xacro.xml"
)

add_custom_target(scenes ALL DEPENDS "${GENERATED_ALL_SCENE}" "${GENERATED_JUST_MAIN_SCENE}")

# Install launch files
install(DIRECTORY
        mujoco
        DESTINATION share/${PROJECT_NAME}/)

install(FILES
  ${GENERATED_ALL_SCENE}
  ${GENERATED_JUST_MAIN_SCENE}
  DESTINATION share/${PROJECT_NAME}/mujoco)

install(TARGETS
  simulator
  simple_simulator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
