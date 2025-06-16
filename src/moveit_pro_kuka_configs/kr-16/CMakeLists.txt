cmake_minimum_required(VERSION 3.22)
project(moveit_pro_kr16)

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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV STUDIO_CONFIG_PACKAGE=lab_sim STUDIO_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR})
endif()

ament_package()
