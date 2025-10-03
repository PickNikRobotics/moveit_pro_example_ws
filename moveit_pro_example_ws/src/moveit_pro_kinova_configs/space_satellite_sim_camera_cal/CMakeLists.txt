cmake_minimum_required(VERSION 3.22)
project(space_satellite_sim_camera_cal)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config description launch objectives waypoints
        DESTINATION share/${PROJECT_NAME})

# Install description/mujoco/assets from space_satellite_sim into this project.
set(SPACE_SATELLITE_SIM_ASSETS_SHARE_DIR
    "${CMAKE_INSTALL_PREFIX}/../space_satellite_sim/share/space_satellite_sim/description/mujoco/assets/"
)
set(DEST_DIR
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/mujoco/assets")
install(
  DIRECTORY "${SPACE_SATELLITE_SIM_ASSETS_SHARE_DIR}"
  DESTINATION "${DEST_DIR}"
  FILES_MATCHING
  PATTERN "*")

# Install meshes from space_satellite_sim into this project.
set(SPACE_SATELLITE_SIM_MESHES_SHARE_DIR
    "${CMAKE_INSTALL_PREFIX}/../space_satellite_sim/share/space_satellite_sim/meshes/"
)
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes")
install(
  DIRECTORY "${SPACE_SATELLITE_SIM_MESHES_SHARE_DIR}"
  DESTINATION "${DEST_DIR}"
  FILES_MATCHING
  PATTERN "*")

# Install AR Markers from picknik_accessories into this project's description
# directory.
set(PICKNIK_ACCESSORIES_SHARE_DIR
    "${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/mujoco")
install(
  DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
  DESTINATION "${DEST_DIR}"
  FILES_MATCHING
  PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
