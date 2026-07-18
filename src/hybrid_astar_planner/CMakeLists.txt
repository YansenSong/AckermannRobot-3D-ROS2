cmake_minimum_required(VERSION 3.5)
project(hybrid_astar_planner)

# C++14 必须在 find_package 之前设置 (某些依赖如 spdlog 需要)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ---- ament 构建 ----
find_package(ament_cmake REQUIRED)

# ---- ROS2 依赖 ----
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---- 系统依赖 ----
find_package(ompl REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Ceres REQUIRED)

# ---- 包含路径 ----
# core/          — 核心算法模板 (a_star.hpp, node_se2.hpp, ...)
# footprint-collision-checker/ — git submodule (header-only)
# standalone_planner/include/  — PGM map adapter
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  ${CMAKE_CURRENT_SOURCE_DIR}/standalone_planner/include
  ${EIGEN3_INCLUDE_DIR}
)

# ================================================================
# 独立 Hybrid A* 规划器节点 (备用)
# ================================================================
add_executable(hybrid_astar_planner_node
  core/node_se2.cpp
  standalone_planner/src/planner_node.cpp
)

ament_target_dependencies(hybrid_astar_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
)

target_link_libraries(hybrid_astar_planner_node
  ompl::ompl
  Eigen3::Eigen
  ceres
)

# ---- 安装 ----
install(TARGETS hybrid_astar_planner_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY standalone_planner/config standalone_planner/launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
