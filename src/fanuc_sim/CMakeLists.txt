cmake_minimum_required(VERSION 3.22)
project(fanuc_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

install_picknik_accessories(
    DESTINATION "share/${PROJECT_NAME}/description"
)
        
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
