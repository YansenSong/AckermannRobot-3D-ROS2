cmake_minimum_required(VERSION 3.8)
project(ackermann_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz xacro
  DESTINATION share/${PROJECT_NAME})

ament_package()
