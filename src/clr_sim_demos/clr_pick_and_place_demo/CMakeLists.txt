cmake_minimum_required(VERSION 3.8)
project(clr_pick_and_place_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(color_tools_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  color_tools_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
  rclcpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(run_demo src/run_demo.cpp)
ament_target_dependencies(run_demo ${dependencies})

# Install launch files.
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS run_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
