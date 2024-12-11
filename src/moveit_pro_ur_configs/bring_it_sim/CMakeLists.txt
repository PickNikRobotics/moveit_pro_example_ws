cmake_minimum_required(VERSION 3.22)
project(bring_it_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)


# Install all XML files in directory
set(UR5E_RIDGEBACK_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../ur5e_ridgeback_mujoco_config/share/ur5e_ridgeback_mujoco_config/assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

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
