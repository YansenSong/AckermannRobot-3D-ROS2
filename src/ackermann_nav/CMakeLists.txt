cmake_minimum_required(VERSION 3.8)
project(ackermann_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(ackermann_goal_orientation_bt_node SHARED
  src/use_path_approach_orientation_bt_node.cpp
)
ament_target_dependencies(
  ackermann_goal_orientation_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
)
target_compile_definitions(
  ackermann_goal_orientation_bt_node PRIVATE BT_PLUGIN_EXPORT
)

install(
  TARGETS ackermann_goal_orientation_bt_node
  DESTINATION lib
)

install(
  PROGRAMS
    scripts/cmd_bridge.py
    scripts/nav2_startup_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
