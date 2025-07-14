cmake_minimum_required(VERSION 3.22)
project(fanuc_config)

find_package(ament_cmake REQUIRED)

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
  # ...
endif()

ament_package()
