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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. Default would be ~/.ros/log/, which lives
  # on the doomed container filesystem and never gets uploaded -- making
  # post-mortem of objective failures impossible.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=hangar_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
