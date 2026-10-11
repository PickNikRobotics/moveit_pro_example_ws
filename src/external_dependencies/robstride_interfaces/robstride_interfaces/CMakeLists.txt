################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robstride_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Interface files
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobstrideState.msg"
  "srv/GetDataFromRobstride.srv"
  "srv/SetDataToRobstride.srv"
  "srv/SetZeroRobstride.srv"
  "srv/RebootRobstride.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
