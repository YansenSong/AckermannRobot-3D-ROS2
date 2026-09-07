cmake_minimum_required(VERSION 3.8)
project(ackermann_gazebo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS gazebo_lidar_adapter.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch models DESTINATION share/${PROJECT_NAME})
# Gazebo model lookup expects model.config at the package-share root.
install(FILES models/model.config DESTINATION share/${PROJECT_NAME})

ament_package()
