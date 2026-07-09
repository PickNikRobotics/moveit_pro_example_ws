cmake_minimum_required(VERSION 3.22)
project(kitchen_sim)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. The default ~/.ros/log/ lives on the doomed
  # container filesystem and never gets uploaded, making post-mortem of
  # objective failures impossible.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=kitchen_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
