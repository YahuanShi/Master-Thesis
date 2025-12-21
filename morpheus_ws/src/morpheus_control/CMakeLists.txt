cmake_minimum_required(VERSION 3.8)
project(morpheus_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

# --- C++ ros2_control controller plugin ---
add_library(morpheus_drive_controller SHARED
  src/morpheus_drive_controller.cpp
)
target_include_directories(morpheus_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(morpheus_drive_controller
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)
pluginlib_export_plugin_description_file(controller_interface morpheus_drive_controller.xml)

install(TARGETS morpheus_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# --- Python scripts and launch files ---
install(DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/morpheus_control.py scripts/kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS launch/morpheus_control.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_kinematics test/test_kinematics.py)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_control_node.py)
endif()

ament_export_libraries(morpheus_drive_controller)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)

ament_package()
