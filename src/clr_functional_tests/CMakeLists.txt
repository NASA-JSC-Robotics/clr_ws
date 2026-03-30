cmake_minimum_required(VERSION 3.16)

project(clr_functional_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  add_launch_test(test/clr_kinematic_sim_test.py
    TIMEOUT 75
  )

  add_launch_test(test/clr_mujoco_sim_test.py
    TIMEOUT 75
  )

endif()

ament_package()
