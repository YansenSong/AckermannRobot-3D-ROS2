cmake_minimum_required(VERSION 3.8)
project(lpms_ig1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(lpms_ig1_node
  src/lpms_ig1_node.cpp
)

add_executable(lpms_ig2_node
  src/lpms_ig2_node.cpp
)

add_executable(lpms_curs3_node
  src/lpms_curs3_node.cpp
)

add_executable(lpms_be1_node
  src/lpms_be1_node.cpp
)
add_executable(lpms_nav3_node
  src/lpms_nav3_node.cpp
)
add_executable(lpms_ig1_rs485_node
  src/lpms_ig1_rs485_node.cpp
)
add_executable(lpms_ig1_rs485_client
  src/lpms_ig1_rs485_client.cpp
)
add_executable(lpms_si1_node
  src/lpms_si1_node.cpp
)
add_executable(quat_to_euler_node
  src/quat_to_euler_node.cpp
)
add_executable(imudata_rad_to_deg_node
  src/imudata_rad_to_deg_node.cpp
)

add_executable(lpms_can_bridge_node src/lpms_can_bridge_node.cpp)

target_link_libraries(lpms_ig1_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_ig2_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_curs3_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_be1_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_nav3_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_ig1_rs485_node LpmsIG1_OpenSourceLib)
target_link_libraries(lpms_si1_node LpmsIG1_OpenSourceLib)

# SocketCAN needs the pthread that std::thread pulls in
find_package(Threads REQUIRED)
target_link_libraries(lpms_can_bridge_node Threads::Threads)

ament_target_dependencies(lpms_ig1_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_ig2_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_curs3_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_be1_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_nav3_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_ig1_rs485_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(lpms_ig1_rs485_client rclcpp std_srvs)
ament_target_dependencies(lpms_si1_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(quat_to_euler_node rclcpp sensor_msgs tf2)
ament_target_dependencies(imudata_rad_to_deg_node rclcpp sensor_msgs tf2)

ament_target_dependencies(lpms_can_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS lpms_ig1_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_ig2_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_curs3_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_be1_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_nav3_node
  DESTINATION lib/${PROJECT_NAME})
 install(TARGETS lpms_ig1_rs485_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_ig1_rs485_client
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lpms_si1_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS quat_to_euler_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS imudata_rad_to_deg_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lpms_can_bridge_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
