cmake_minimum_required(VERSION 3.22)
project(lunar_sim)

find_package(ament_cmake REQUIRED)
find_package(phoebe_sim REQUIRED)

install(
  DIRECTORY
    config
    mjcf
  DESTINATION
    share/${PROJECT_NAME}
)

# Mujoco models in this package use relative paths to assets such as meshes and textures.
# To simplify building relative paths to assets in other packages, we will copy those assets
# into this project's file structure.

# Paths we will copy from - the robot MJCF files and mesh assets from phoebe_sim
set(PHOEBE_SIM_DESCRIPTION_PATH "${phoebe_sim_DIR}/../description/")

# Path we will copy to - the folder where this robot config's Mujoco model is located
set(MUJOCO_MODEL_DIR "share/${PROJECT_NAME}/mjcf/")

# Copy all contents into the installed mjcf folder
install(DIRECTORY "${PHOEBE_SIM_DESCRIPTION_PATH}"
        DESTINATION "${MUJOCO_MODEL_DIR}"
        FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
