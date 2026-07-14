cmake_minimum_required(VERSION 3.22)
project(lab_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)


# Install all XML files in directory
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")

# Install individual XML files
# set(EXTERNAL_XML_FILES
#   "${PICKNIK_ACCESSORIES_SHARE_DIR}/ur5e/ur5e_globals.xml"
#   "${PICKNIK_ACCESSORIES_SHARE_DIR}/ur5e/ur5e.xml"
#   # Add additional files here if desired
# )
# foreach(xml_file IN LISTS EXTERNAL_XML_FILES)
#   install(FILES "${xml_file}"
#           DESTINATION "${DEST_DIR}"
#   )
# endforeach()

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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. Default would be ~/.ros/log/, which lives
  # on the doomed container filesystem and never gets uploaded -- making
  # post-mortem of objective failures impossible.
  set(LAB_SIM_INTEGRATION_TEST_ENV
      MOVEIT_CONFIG_PACKAGE=lab_sim
      MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
  # Point-cloud conversion for the 1280x720 wrist camera can starve MuJoCo's
  # single render executor on a loaded GitHub Actions runner. Reduce only the
  # integration-test rate; normal and local-test launches retain the 10 Hz
  # Xacro default so moving-camera point clouds keep their temporal fidelity.
  if("$ENV{GITHUB_ACTIONS}" STREQUAL "true")
    list(APPEND LAB_SIM_INTEGRATION_TEST_ENV MOVEIT_PRO_POINT_CLOUD_PUBLISH_RATE=1)
  endif()
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV ${LAB_SIM_INTEGRATION_TEST_ENV})
endif()

ament_package()
