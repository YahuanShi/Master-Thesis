cmake_minimum_required(VERSION 3.5)
project(morpheus_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#xacro_add_files(
#  robots/head_in.urdf.xacro
#    INSTALL DESTINATION robots
#)

# add_custom_target(urdf ALL COMMAND "ros2" "launch" ${PROJECT_NAME} "wamv_description.launch.py")

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY user_interface/
  DESTINATION share/${PROJECT_NAME}/user_interface)

# install(DIRECTORY 02_gazebo/
#   DESTINATION share/${PROJECT_NAME}/02_gazebo)

# install(DIRECTORY 03_ros2_control/
#   DESTINATION share/${PROJECT_NAME}/03_ros2_control)

# install(DIRECTORY 03_ros2_control/01_config/
#   DESTINATION share/${PROJECT_NAME}/03_ros2_control/01_config)

# install(DIRECTORY 01_urdf/01_rover/meshes/
#   DESTINATION share/${PROJECT_NAME}/01_urdf/01_rover/meshes)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
