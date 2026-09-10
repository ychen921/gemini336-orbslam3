cmake_minimum_required(VERSION 3.8)
project(gemini336_slam_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(timestamp_diagnostic_node
  src/timestamp_diagnostic_node.cpp
)

ament_target_dependencies(timestamp_diagnostic_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS timestamp_diagnostic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
