cmake_minimum_required(VERSION 3.22)
project(space_satellite_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    meshes
    objectives
    rviz
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

# Install AR Markers from picknik_accessories into this project's description directory.
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/mujoco")
install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
