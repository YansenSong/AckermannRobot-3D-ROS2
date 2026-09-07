cmake_minimum_required(VERSION 3.8)
project(ackermann_simulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gazebo_lidar_adapter.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY gazebo_model launch meshes models rviz worlds xacro
  DESTINATION share/${PROJECT_NAME})

ament_package()
