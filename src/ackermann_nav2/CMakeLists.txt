cmake_minimum_required(VERSION 3.10)
project(ackermann_nav2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/nav2_cmd_adapter.py
  scripts/nav2_localization_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
