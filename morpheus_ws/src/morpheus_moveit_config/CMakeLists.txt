cmake_minimum_required(VERSION 3.5)
project(morpheus_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/arm_cartesian_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
