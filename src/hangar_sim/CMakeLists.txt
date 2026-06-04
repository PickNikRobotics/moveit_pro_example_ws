cmake_minimum_required(VERSION 3.22)
project(hangar_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_studio_plugins REQUIRED)

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

pluginlib_export_plugin_description_file(
        moveit_ros_control_interface moveit_mecanum_drive_controller_plugins.xml)

install(PROGRAMS
  script/odometry_joint_state_publisher.py
  script/odom_qos_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(IMPORTED_RUNTIME_ARTIFACTS moveit_studio_plugins::dummy_controller_handle_plugin LIBRARY DESTINATION
        lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # End-to-end objectives integration test. Mirrors lab_sim: launches the
  # hangar_sim backend once and runs every runnable objective against it.
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them too -- the default ~/.ros/log/ lives on the doomed
  # container filesystem and never gets uploaded. TIMEOUT and the heartbeat
  # override are larger than lab_sim's because hangar_sim boots a heavier
  # stack (nav2 / slam_toolbox / fuse) before the first objective can run.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 900
          ENV MOVEIT_CONFIG_PACKAGE=hangar_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              MOVEIT_PRO_TEST_HEARTBEAT_TIMEOUT_S=300
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
