cmake_minimum_required(VERSION 3.16)

project(clr_functional_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/clr_launch_test.py
    TIMEOUT 75
  )

endif()

ament_package()
