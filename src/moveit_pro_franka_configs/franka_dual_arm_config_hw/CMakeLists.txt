cmake_minimum_required(VERSION 3.22)
project(franka_dual_arm_config_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    waypoints
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
