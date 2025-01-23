cmake_minimum_required(VERSION 3.22)
project(ridgeback_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    scripts
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
