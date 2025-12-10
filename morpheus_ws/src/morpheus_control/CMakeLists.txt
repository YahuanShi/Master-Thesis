cmake_minimum_required(VERSION 3.5)
project(morpheus_control)

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
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/morpheus_control.py scripts/kinematics.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS launch/morpheus_control.launch.py
  DESTINATION lib/${PROJECT_NAME})

#file (COPY ${CMAKE_BINARY_DIR}/tmp/somescript.sh 
#  DESTINATION ${CMAKE_BINARY_DIR} FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)

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

ament_package()
