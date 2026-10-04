cmake_minimum_required(VERSION 3.8)
project(nav_neupan_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
