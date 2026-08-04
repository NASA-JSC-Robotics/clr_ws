cmake_minimum_required(VERSION 3.22)
project(clr_roboplan_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_oink REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(roboplan_toppra REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/chonkur_oink_streaming_node.py
        scripts/plan_and_execute_node.py
        scripts/viser_ik_rrt.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
