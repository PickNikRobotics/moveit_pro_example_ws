cmake_minimum_required(VERSION 3.22)
project(franka_base_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
