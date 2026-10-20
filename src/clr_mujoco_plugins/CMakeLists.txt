cmake_minimum_required(VERSION 3.16)
project(clr_mujoco_plugins)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_limit_monitor_plugin.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugins derive from a base class owned by mujoco_ros2_control_plugins, so register them there.
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins clr_mujoco_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_limit_monitor_plugin
    test/test_joint_limit_monitor_plugin.cpp
  )
  target_include_directories(test_joint_limit_monitor_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_joint_limit_monitor_plugin
    ${PROJECT_NAME}
  )
  # MuJoCo statically links TinyXML2, which conflicts with the system TinyXML2 used by FastDDS.
  # Mirror mujoco_ros2_control_plugins and force the system one to load first.
  # https://github.com/pal-robotics/mujoco_vendor/issues/2
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  target_link_options(test_joint_limit_monitor_plugin PUBLIC
    "LINKER:--push-state,--no-as-needed"
    "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
    "LINKER:--pop-state"
  )
endif()

ament_package()
