cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
