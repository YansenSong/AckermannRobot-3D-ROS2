cmake_minimum_required(VERSION 3.8)
project(ackermann_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS arrow_key_control.py cmd_vel_mux.py neupan_ackermann_adapter.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
