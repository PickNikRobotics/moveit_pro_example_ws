cmake_minimum_required(VERSION 3.22)
project(so101_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          so101_sim_inherits_base_test test/test_config_inheritance.py
          TIMEOUT 60)
endif()

ament_package()
