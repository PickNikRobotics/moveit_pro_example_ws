cmake_minimum_required(VERSION 3.22)
project(hangar_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(slip_aware_odom src/slip_aware_odom.cpp)
ament_target_dependencies(slip_aware_odom rclcpp nav_msgs)

add_executable(amcl_odom_gate src/amcl_odom_gate.cpp src/amcl_odom_gate_logic.cpp)
target_include_directories(amcl_odom_gate PRIVATE include)
target_compile_features(amcl_odom_gate PRIVATE cxx_std_17)
ament_target_dependencies(amcl_odom_gate rclcpp geometry_msgs tf2_ros)

install(TARGETS slip_aware_odom amcl_odom_gate DESTINATION lib/${PROJECT_NAME})

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
  script/odom_world_drift.py
  DESTINATION lib/${PROJECT_NAME}
)

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

  # Unit tests for the pure amcl_odom_gate decision logic (no ROS/TF).
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_amcl_odom_gate test/test_amcl_odom_gate.cpp src/amcl_odom_gate_logic.cpp)
  target_include_directories(test_amcl_odom_gate PRIVATE include)
  target_compile_features(test_amcl_odom_gate PRIVATE cxx_std_17)
endif()

ament_package()
