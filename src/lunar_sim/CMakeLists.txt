cmake_minimum_required(VERSION 3.22)
project(lunar_sim)

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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          husky_mujoco_geometry_test test/test_husky_mujoco_geometry.py
          TIMEOUT 30)
endif()

ament_package()
