cmake_minimum_required(VERSION 3.22)
project(rebot_bench_sim)

find_package(ament_cmake REQUIRED)
find_package(rebot_bench_base_config REQUIRED)

install(
  DIRECTORY
    config
    description
    objectives
    launch
  DESTINATION
    share/${PROJECT_NAME}
  PATTERN "*.xml.xacro" EXCLUDE
  PATTERN "bench_layout.yaml" EXCLUDE
)

# The MuJoCo scene and arm are generated from a build-time snapshot of
# config/bench_layout.yaml, installed beside them for the robot description to
# read, so a layout edit reaches both together on rebuild and never just one.
# The snapshot adds entries derived from the layout (the wrist-camera mount
# joint, scripts/layout_snapshot.py).
set(BASE_XACRO ${rebot_bench_base_config_DIR}/../description/rebot.urdf.xacro)
set(SCENE_XML ${CMAKE_CURRENT_BINARY_DIR}/scene.xml)
set(ARM_XML ${CMAKE_CURRENT_BINARY_DIR}/rebot_arm.xml)
set(LAYOUT_SNAPSHOT ${CMAKE_CURRENT_BINARY_DIR}/bench_layout.yaml)
add_custom_command(
  OUTPUT ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/layout_snapshot.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/bench_layout.yaml ${BASE_XACRO} ${LAYOUT_SNAPSHOT}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${SCENE_XML}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot_arm.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${ARM_XML}
  DEPENDS description/scene.xml.xacro description/rebot_arm.xml.xacro
          config/bench_layout.yaml scripts/layout_snapshot.py ${BASE_XACRO}
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL DEPENDS ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT})
install(FILES ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT} DESTINATION share/${PROJECT_NAME}/description)

install(PROGRAMS scripts/record_stack_evidence.py scripts/sim_compressed_cameras.py scripts/sim_grasp_proxy.py
        DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(
          rebot_bench_sim_grasp_proxy_test test/test_sim_grasp_proxy.py
          TIMEOUT 60)
  ament_add_pytest_test(
          rebot_bench_sim_compressed_cameras_test test/test_sim_compressed_cameras.py
          TIMEOUT 60)
  ament_add_pytest_test(
          rebot_bench_sim_recorder_test test/test_record_stack_evidence.py
          TIMEOUT 60)
  ament_add_pytest_test(rebot_leader_startup_test test/test_leader_startup.py TIMEOUT 60)
  # Runs every Objective against the MuJoCo twin, like lab_sim's suite.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=rebot_bench_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
