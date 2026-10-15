cmake_minimum_required(VERSION 3.22)
project(market_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    maps
    mjcf
    objectives
    params
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/base_twist_to_planar.py
    scripts/lidar_flattener.py
    scripts/odometry_joint_state_publisher.py
    scripts/reset_relocalizer.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Nav2's stock navigate-to-pose tree with SmacLattice as its planner, for Drive Base using Nav2.
# The build stops if the stock tree no longer selects GridBased by default.

# Not find_package: nav2_bt_navigator's CMake config needs stock BT.CPP, absent from the image.
ament_index_has_resource(NAV2_BT_NAVIGATOR_PREFIX packages nav2_bt_navigator)
if(NOT NAV2_BT_NAVIGATOR_PREFIX)
  message(FATAL_ERROR "nav2_bt_navigator is not installed")
endif()
set(STOCK_NAV_TREE "${NAV2_BT_NAVIGATOR_PREFIX}/share/nav2_bt_navigator/behavior_trees/\
navigate_to_pose_w_replanning_and_recovery.xml")
file(READ "${STOCK_NAV_TREE}" NAV_TREE)
string(FIND "${NAV_TREE}" "default_planner=\"GridBased\"" NAVFN_SELECTOR)
if(NAVFN_SELECTOR EQUAL -1)
  message(FATAL_ERROR "${STOCK_NAV_TREE} no longer selects GridBased by default")
endif()
string(REPLACE "default_planner=\"GridBased\"" "default_planner=\"SmacLattice\""
  NAV_TREE "${NAV_TREE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/behavior_trees/navigate_to_pose_SmacLattice.xml"
  "${NAV_TREE}")
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/behavior_trees" DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          reset_relocalizer_test test/test_reset_relocalizer.py
          TIMEOUT 30)
  # The generated scene, map and Objectives against the generator, and the compiled MuJoCo model.
  ament_add_pytest_test(
          market_scene_test test/test_market_scene.py
          TIMEOUT 120)
  # The Nav2 footprint of both costmaps, and the aisle goals against the map.
  ament_add_pytest_test(
          nav2_footprint_test test/test_nav2_footprint.py
          TIMEOUT 60)
  # Drive Base using Nav2's tree files against the launch file and the installed copies.
  ament_add_pytest_test(
          nav_tree_files_test test/test_nav_tree_files.py
          TIMEOUT 30)
  ament_add_pytest_test(
          base_twist_to_planar_test test/test_base_twist_to_planar.py
          TIMEOUT 30)
  ament_add_pytest_test(
          odometry_joint_state_publisher_test test/test_odometry_joint_state_publisher.py
          TIMEOUT 30)
  # Every runnable Objective against the running simulator. ROS node logs go into the test
  # results, so the CI artifact carries them. The three store drives set the timeout.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 1800
          ENV MOVEIT_CONFIG_PACKAGE=market_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
