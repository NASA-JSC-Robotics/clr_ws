cmake_minimum_required(VERSION 3.22)
project(clr_behavior_demos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
