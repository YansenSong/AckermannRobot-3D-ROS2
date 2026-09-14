cmake_minimum_required(VERSION 3.8)
project(ackermann_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
