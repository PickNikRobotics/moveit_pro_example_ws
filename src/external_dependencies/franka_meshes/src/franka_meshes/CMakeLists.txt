cmake_minimum_required(VERSION 3.22)
project(franka_meshes)

find_package(ament_cmake REQUIRED)

# The whole tree, so package://franka_meshes/meshes/... resolves at the source-tree path.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)
install(FILES NOTICE DESTINATION share/${PROJECT_NAME}/)

# script/ is deliberately not installed: it is a dev tool run by hand from the source tree,
# and it needs mesh libraries that are not workspace dependencies.

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_meshes test/test_meshes.py)
endif()

ament_package()
