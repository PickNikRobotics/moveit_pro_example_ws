cmake_minimum_required(VERSION 3.22)
project(pushing_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
