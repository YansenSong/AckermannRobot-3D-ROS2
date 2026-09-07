cmake_minimum_required(VERSION 3.8)
project(ackermann_simulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS gazebo/scripts/gazebo_lidar_adapter.py DESTINATION lib/${PROJECT_NAME})
# Keep launch files in the conventional installed share/<package>/launch path,
# while the source tree groups them with the Gazebo resources.
install(DIRECTORY gazebo/launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY robot DESTINATION share/${PROJECT_NAME})
install(DIRECTORY gazebo/worlds DESTINATION share/${PROJECT_NAME}/gazebo)

ament_package()
