cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_sim_config)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

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
  script/odometry_joint_state_publisher.py
  script/odom_qos_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Register a dummy controller-handle allocator so MoveIt's controller manager
# can claim/release the clearpath_mecanum_drive_controller.
pluginlib_export_plugin_description_file(
  moveit_ros_control_interface moveit_mecanum_drive_controller_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
