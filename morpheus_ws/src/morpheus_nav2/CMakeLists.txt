cmake_minimum_required(VERSION 3.8)
project(morpheus_nav2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

# --- BT condition plugin: IsArucoDetected ---
add_library(morpheus_is_aruco_detected_bt_node SHARED
  src/is_aruco_detected_condition.cpp
)
target_include_directories(morpheus_is_aruco_detected_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(morpheus_is_aruco_detected_bt_node
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  ros2_aruco_interfaces
)

install(TARGETS morpheus_is_aruco_detected_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# --- Config, launch, behavior trees ---
install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/patrol_mission.py
  scripts/ground_segmentation.py
  scripts/aruco_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
