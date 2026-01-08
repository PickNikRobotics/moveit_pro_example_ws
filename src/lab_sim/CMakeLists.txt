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
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=lab_sim MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR})
endif()

ament_package()
