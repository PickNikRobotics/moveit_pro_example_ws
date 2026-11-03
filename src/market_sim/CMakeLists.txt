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

# market_assets is a build-time input to scripts/generate_market.py, but three consumers also
# reach into it at RUN time: scene.xml includes the cart materials, the generated store visuals
# include the cart body, and description/market_racks.urdf names the shelf meshes through a
# package:// URI. Install the whole directory rather than named subdirectories, so a new asset
# cannot be referenced at runtime and silently missing from the install tree.
install(
  DIRECTORY
    market_assets
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
  # Every runtime file reference against the installed tree, not the source tree.
  ament_add_pytest_test(
          installed_references_test test/test_installed_references.py
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
