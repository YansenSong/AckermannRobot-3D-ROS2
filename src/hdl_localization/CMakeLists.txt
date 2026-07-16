cmake_minimum_required(VERSION 3.10)
project(hdl_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# SIMD optimization
add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# PCL (system dependency)
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ndt_omp (local package)
find_package(ndt_omp REQUIRED)

include_directories(include)

# ---- hdl_localization_node ----
add_executable(hdl_localization_node
  src/hdl_localization_node.cpp
  src/pose_estimator.cpp
)

target_include_directories(hdl_localization_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hdl_localization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
)

target_link_libraries(hdl_localization_node
  ndt_omp::ndt_omp
  ${PCL_LIBRARIES}
)

# ---- globalmap_server_node ----
add_executable(globalmap_server_node
  src/globalmap_server_node.cpp
)

target_include_directories(globalmap_server_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(globalmap_server_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(globalmap_server_node
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

# ---- Install ----
install(TARGETS hdl_localization_node globalmap_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
