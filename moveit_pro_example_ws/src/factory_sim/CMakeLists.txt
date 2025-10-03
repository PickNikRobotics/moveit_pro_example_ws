cmake_minimum_required(VERSION 3.22)
project(factory_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

install(DIRECTORY config description launch objectives waypoints
        DESTINATION share/${PROJECT_NAME})

# Mujoco models in this package use relative paths to assets such as meshes and
# textures To simplify building relative paths to assets in other packages, we
# will copy those assets into this project's file structure

# Paths we will copy from - the assets we want to easily reference in our Mujoco
# model
set(PICKNIK_ACCESSORIES_PATH "${picknik_accessories_DIR}/../mujoco_assets/")

# Path we will copy to - the folder where this robot config's Mujoco model is
# located
set(MUJOCO_MODEL_DIR "share/${PROJECT_NAME}/description/")

# Copy all contents into the installed description folder
install(
  DIRECTORY "${PICKNIK_ACCESSORIES_PATH}"
  DESTINATION "${MUJOCO_MODEL_DIR}"
  FILES_MATCHING
  PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
