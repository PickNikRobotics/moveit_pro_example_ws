cmake_minimum_required(VERSION 3.22)
project(hangar_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

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

install(PROGRAMS
  script/forward_stereo_publisher.py
  script/odometry_joint_state_publisher.py
  script/odom_qos_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          forward_stereo_calibration_test test/test_forward_stereo_calibration.py
          TIMEOUT 30)
  ament_add_pytest_test(
          forward_stereo_publisher_test test/test_forward_stereo_publisher.py
          TIMEOUT 60)
  # Static checks over the copied Request Teleoperation Objective. Needs no
  # running stack: the regression it guards aborts the command before any
  # motion, and Teleoperate cannot run headless (see the module docstring).
  ament_add_pytest_test(
          teleoperation_commands_smoke_test test/teleoperation_commands_smoke_test.py
          TIMEOUT 30)
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. Default would be ~/.ros/log/, which lives
  # on the doomed container filesystem and never gets uploaded -- making
  # post-mortem of objective failures impossible.
  # 900 s: the suite runs ~7 min on the CI runner, and the surface-following
  # objectives carry 180 s execute budgets (EXECUTE_TIMEOUT_OVERRIDES_S in the
  # test) — two worst-case executions alone could exceed the previous 600 s.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 900
          ENV MOVEIT_CONFIG_PACKAGE=hangar_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
