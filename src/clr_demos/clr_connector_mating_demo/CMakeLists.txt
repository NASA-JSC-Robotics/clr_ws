cmake_minimum_required(VERSION 3.22)
project(clr_connector_mating_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_connector_mating_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()