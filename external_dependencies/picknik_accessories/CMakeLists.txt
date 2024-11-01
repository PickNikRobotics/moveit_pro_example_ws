cmake_minimum_required(VERSION 3.22)
project(picknik_accessories)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    descriptions
    macros
  DESTINATION
    share/${PROJECT_NAME}
)
# TODO: is this lint process hanging somehow?
# if(BUILD_TESTING)
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
# endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()
