cmake_minimum_required(VERSION 3.5)
project(global_motion)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(global_motion_node
  src/global_motion_node.cpp
)

ament_target_dependencies(global_motion_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  
)

install(TARGETS
  global_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
