cmake_minimum_required(VERSION 3.8)
project(morpheus_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/patrol_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
