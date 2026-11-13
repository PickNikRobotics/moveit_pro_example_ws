cmake_minimum_required(VERSION 3.22)
project(lunar_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    maps
    objectives
    params
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_map_server REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_add_gtest(keepout_collision_test test/test_keepout_collision.cpp)
  target_compile_definitions(keepout_collision_test PRIVATE
    LUNAR_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_target_dependencies(keepout_collision_test
    nav2_costmap_2d nav2_map_server nav2_util pluginlib tf2_ros)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Pytest discovers the Objective integration module from this directory too. Give the
  # geometry and base-jog tests the same config selection and workspace root as that module.
  set(LUNAR_SIM_UNIT_TEST_ENV
      MOVEIT_CONFIG_PACKAGE=lunar_sim
      USER_WS=${CMAKE_CURRENT_SOURCE_DIR}/../..)
  ament_add_pytest_test(
          husky_mujoco_geometry_test test/test_husky_mujoco_geometry.py
          TIMEOUT 30
          ENV ${LUNAR_SIM_UNIT_TEST_ENV})
  ament_add_pytest_test(
          base_jog_config_test test/test_base_jog_config.py
          TIMEOUT 30
          ENV ${LUNAR_SIM_UNIT_TEST_ENV})
  ament_add_pytest_test(
          keepout_mask_test test/test_keepout_mask.py
          TIMEOUT 30
          ENV ${LUNAR_SIM_UNIT_TEST_ENV})
  # Redirect ROS node logs into the test_results tree so the test-results CI artifact ships them back too.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 900
          ENV MOVEIT_CONFIG_PACKAGE=lunar_sim
              MOVEIT_PRO_CAMERA_RENDERER=opengl
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
