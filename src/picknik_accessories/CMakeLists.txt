cmake_minimum_required(VERSION 3.22)
project(picknik_accessories)

find_package(ament_cmake REQUIRED)
# LOCAL SPIKE CHANGE -- NOT FOR COMMIT (example_ws here is a disposable materialization).
# fanuc_lrmate200id_support is not available in this workspace and has no apt package,
# so this hard requirement blocks building picknik_accessories and therefore lab_sim,
# which uses no Fanuc assets at all. Made optional so the Nyx sidecar can be tested
# against a live lab_sim stack. factory_sim would lose its Fanuc meshes.
find_package(fanuc_lrmate200id_support QUIET)

install(
  DIRECTORY
    descriptions
    macros
    mujoco_assets
  DESTINATION
    share/${PROJECT_NAME}
)

# Mujoco models in this package use relative paths to assets such as meshes and textures
# To simplify building relative paths to assets in other packages, we will copy those assets into this project's file structure

# Paths we will copy from - the assets we want to easily reference in our Mujoco model
set(FANUC_LRMATE200ID_SUPPORT_PATH "${fanuc_lrmate200id_support_DIR}/../meshes")
if(NOT fanuc_lrmate200id_support_FOUND)
  message(WARNING "fanuc_lrmate200id_support not found; skipping its mesh install (local spike workaround)")
endif()

# Path we will copy to - the folder where the other Mujoco assets are located
set(DEST_DIR "share/${PROJECT_NAME}/mujoco_assets/")

# Copy all contents into the installed description folder
if(fanuc_lrmate200id_support_FOUND)
  install(DIRECTORY "${FANUC_LRMATE200ID_SUPPORT_PATH}"
          DESTINATION "${DEST_DIR}"
          FILES_MATCHING PATTERN "*")
endif()

# TODO: is this lint process hanging somehow?
# if(BUILD_TESTING)
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
# endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()
