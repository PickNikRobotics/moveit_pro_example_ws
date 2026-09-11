cmake_minimum_required(VERSION 3.22)
project(so101_base_config)

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
          so101_hardware_plugin_test test/test_hardware_plugin.py
          TIMEOUT 60)
endif()

ament_package()
