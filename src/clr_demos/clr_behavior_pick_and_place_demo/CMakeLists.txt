cmake_minimum_required(VERSION 3.8)
project(clr_behavior_pick_and_place_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies_pkgs
    ament_cmake
    rclcpp
    drt_behavior
    moveit_behavior
    color_tools_behavior
    geometry_behavior
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

install(DIRECTORY
    config
    launch
    behavior_trees_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
