cmake_minimum_required(VERSION 3.22)
project(clr_behavior_demos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/roboplan_planning_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
