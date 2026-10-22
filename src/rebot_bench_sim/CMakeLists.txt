cmake_minimum_required(VERSION 3.22)
project(rebot_bench_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
  DESTINATION
    share/${PROJECT_NAME}
  PATTERN "scene.xml.xacro" EXCLUDE
  PATTERN "bench_layout.yaml" EXCLUDE
)

# The MuJoCo scene is generated from a build-time snapshot of
# config/bench_layout.yaml, installed beside it for the robot description to
# read, so a layout edit reaches both together on rebuild and never just one.
set(SCENE_XML ${CMAKE_CURRENT_BINARY_DIR}/scene.xml)
set(LAYOUT_SNAPSHOT ${CMAKE_CURRENT_BINARY_DIR}/bench_layout.yaml)
add_custom_command(
  OUTPUT ${SCENE_XML} ${LAYOUT_SNAPSHOT}
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_SOURCE_DIR}/config/bench_layout.yaml ${LAYOUT_SNAPSHOT}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${SCENE_XML}
  DEPENDS description/scene.xml.xacro config/bench_layout.yaml
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL DEPENDS ${SCENE_XML} ${LAYOUT_SNAPSHOT})
install(FILES ${SCENE_XML} ${LAYOUT_SNAPSHOT} DESTINATION share/${PROJECT_NAME}/description)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          rebot_bench_sim_inherits_base_test test/test_config_inheritance.py
          TIMEOUT 60)
  ament_add_pytest_test(
          rebot_bench_sim_scene_model_test test/test_scene_model.py
          TIMEOUT 120)
  # Runs every Objective against the MuJoCo twin, like lab_sim's suite.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=rebot_bench_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
