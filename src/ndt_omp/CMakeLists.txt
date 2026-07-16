cmake_minimum_required(VERSION 3.10)
project(ndt_omp)

# C++17 for ROS 2 Humble compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# SIMD optimization
if (BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
else()
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")

if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Build shared library
include_directories(include)

add_library(ndt_omp SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

target_include_directories(ndt_omp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp OpenMP::OpenMP_CXX)
endif()

# Install
install(TARGETS ndt_omp
  EXPORT ndt_omp_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/pclomp
  DESTINATION include
)

install(EXPORT ndt_omp_targets
  DESTINATION share/ndt_omp/cmake
)

ament_export_targets(ndt_omp_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
