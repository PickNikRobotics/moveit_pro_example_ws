cmake_minimum_required(VERSION 3.22)
project(rebot_bench_sim)

find_package(ament_cmake REQUIRED)
find_package(rebot_bench_behaviors REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
  PATTERN "*.xml.xacro" EXCLUDE
  PATTERN "bench_layout.yaml" EXCLUDE
)
install(FILES README.md LICENSE thumbnail.png DESTINATION share/${PROJECT_NAME})

# The MuJoCo scene and arm are generated from a build-time snapshot of
# config/bench_layout.yaml, installed beside them for the robot description to
# read, so a layout edit reaches both together on rebuild and never just one.
# The snapshot adds entries derived from the layout (the wrist-camera mount
# joint, scripts/layout_snapshot.py).
set(ROBOT_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot.urdf.xacro)
set(SCENE_XML ${CMAKE_CURRENT_BINARY_DIR}/scene.xml)
set(ARM_XML ${CMAKE_CURRENT_BINARY_DIR}/rebot_arm.xml)
set(LAYOUT_SNAPSHOT ${CMAKE_CURRENT_BINARY_DIR}/bench_layout.yaml)
add_custom_command(
  OUTPUT ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/layout_snapshot.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/bench_layout.yaml ${ROBOT_XACRO} ${LAYOUT_SNAPSHOT}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${SCENE_XML}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot_arm.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${ARM_XML}
  DEPENDS description/scene.xml.xacro description/rebot_arm.xml.xacro
          config/bench_layout.yaml scripts/layout_snapshot.py ${ROBOT_XACRO}
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL DEPENDS ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT})
install(FILES ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT} DESTINATION share/${PROJECT_NAME}/description)

install(PROGRAMS scripts/record_stack_evidence.py scripts/sim_compressed_cameras.py scripts/sim_grasp_proxy.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(behaviortree_cpp_picknik REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(moveit_pro_base REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  ament_add_gtest(test_stack_objectives test/test_stack_objectives.cpp)
  ament_target_dependencies(test_stack_objectives behaviortree_cpp_picknik)
  target_compile_features(test_stack_objectives PRIVATE cxx_std_20)
  # Header-only motion settings parser; the Behavior library itself is not linked.
  target_include_directories(test_stack_objectives PRIVATE
    $<TARGET_PROPERTY:rebot_bench_behaviors::rebot_bench_behaviors,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(test_stack_objectives tinyxml2::tinyxml2 yaml-cpp::yaml-cpp)
  target_compile_definitions(test_stack_objectives PRIVATE PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/test_rebot.urdf)
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND xacro ${ROBOT_XACRO} -o ${TEST_URDF}
    DEPENDS ${ROBOT_XACRO}
    VERBATIM)
  add_custom_target(test_rebot_urdf DEPENDS ${TEST_URDF})
  ament_add_gtest(test_joint_limits test/test_joint_limits.cpp)
  add_dependencies(test_joint_limits test_rebot_urdf)
  ament_target_dependencies(test_joint_limits moveit_pro_base urdf srdfdom)
  target_link_libraries(test_joint_limits yaml-cpp::yaml-cpp)
  target_compile_definitions(test_joint_limits PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_gtest(test_jaw_legs test/test_jaw_legs.cpp)
  add_dependencies(test_jaw_legs test_rebot_urdf)
  target_compile_features(test_jaw_legs PRIVATE cxx_std_20)
  ament_target_dependencies(test_jaw_legs moveit_pro_base urdf srdfdom)
  target_link_libraries(test_jaw_legs rebot_bench_behaviors::rebot_bench_behaviors)
  target_compile_definitions(test_jaw_legs PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # pytest's launch_testing hook imports objectives_integration_test.py while collecting any test
  # in this directory, so every test needs its config selection and workspace root.
  set(REBOT_BENCH_SIM_TEST_ENV
      MOVEIT_CONFIG_PACKAGE=rebot_bench_sim
      USER_WS=${CMAKE_CURRENT_SOURCE_DIR}/../..)
  ament_add_pytest_test(
          test_rebot_model test/test_model.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          test_stack_legs test/test_stack_legs.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          rebot_bench_sim_config_test test/test_config.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          rebot_bench_sim_scene_model_test test/test_scene_model.py
          TIMEOUT 120
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          rebot_bench_sim_grasp_proxy_test test/test_sim_grasp_proxy.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          rebot_bench_sim_compressed_cameras_test test/test_sim_compressed_cameras.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  ament_add_pytest_test(
          rebot_bench_sim_recorder_test test/test_record_stack_evidence.py
          TIMEOUT 60
          ENV ${REBOT_BENCH_SIM_TEST_ENV})
  # Runs every Objective against the MuJoCo twin, like lab_sim's suite.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=rebot_bench_sim
              MOVEIT_PRO_CAMERA_RENDERER=opengl
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
