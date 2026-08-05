cmake_minimum_required(VERSION 3.22)
project(factory_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    models
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

# Mujoco models in this package use relative paths to assets such as meshes and textures
# To simplify building relative paths to assets in other packages, we will copy those assets into this project's file structure

# Paths we will copy from - the assets we want to easily reference in our Mujoco model
set(PICKNIK_ACCESSORIES_PATH "${picknik_accessories_DIR}/../mujoco_assets/")

# Path we will copy to - the folder where this robot config's Mujoco model is located
set(MUJOCO_MODEL_DIR "share/${PROJECT_NAME}/description/")

# Copy all contents into the installed description folder
install(DIRECTORY "${PICKNIK_ACCESSORIES_PATH}"
        DESTINATION "${MUJOCO_MODEL_DIR}"
        FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. The default ~/.ros/log/ lives on the doomed
  # container filesystem and never gets uploaded, making post-mortem of
  # objective failures impossible.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=factory_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
