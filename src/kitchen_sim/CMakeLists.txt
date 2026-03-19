cmake_minimum_required(VERSION 3.22)
project(kitchen_sim)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    models
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
          ENV MOVEIT_CONFIG_PACKAGE=kitchen_sim MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR})
endif()

ament_package()
