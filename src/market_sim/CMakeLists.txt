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
    scripts/navigation_scan_relay.py
    scripts/odometry_joint_state_publisher.py
    scripts/reset_controller_guard.py
    scripts/reset_relocalizer.py
    scripts/scan_merger.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(behaviortree_cpp_picknik REQUIRED)
  find_package(moveit_pro_behavior_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  add_executable(market_sim_test_run_objective_tree test/run_objective_tree.cpp)
  target_compile_features(market_sim_test_run_objective_tree PRIVATE cxx_std_20)
  ament_target_dependencies(
    market_sim_test_run_objective_tree
    behaviortree_cpp_picknik
    moveit_pro_behavior_interface
    pluginlib
    rclcpp
  )
  install(TARGETS market_sim_test_run_objective_tree DESTINATION lib/${PROJECT_NAME})
  ament_add_pytest_test(
          reset_objective_test test/test_reset_objective.py
          TIMEOUT 60)
  ament_add_pytest_test(
          undock_objective_test test/test_undock_objective.py
          TIMEOUT 180)
  add_executable(market_sim_test_behavior_manifest test/export_behavior_manifest.cpp)
  target_compile_features(market_sim_test_behavior_manifest PRIVATE cxx_std_20)
  ament_target_dependencies(
    market_sim_test_behavior_manifest
    behaviortree_cpp_picknik
    moveit_pro_behavior_interface
    pluginlib
  )
  install(TARGETS market_sim_test_behavior_manifest DESTINATION lib/${PROJECT_NAME})
  # Registered node IDs come from the installed loaders, including built-in control nodes.
  ament_add_pytest_test(
          objective_behavior_manifest_test test/test_objective_behavior_manifest.py
          TIMEOUT 60)
  # The recovery cases exercise wall-clock continuity windows and bounded reset timeouts.
  ament_add_pytest_test(
          reset_relocalizer_test test/test_reset_relocalizer.py
          TIMEOUT 60)
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
  ament_add_pytest_test(
          scan_merger_test test/test_scan_merger.py
          TIMEOUT 30)
  ament_add_pytest_test(
          camera_renderer_test test/test_camera_renderer.py
          TIMEOUT 30)
  ament_add_pytest_test(
          ground_truth_navigation_test test/test_ground_truth_navigation.py
          TIMEOUT 30)
  ament_add_pytest_test(
          duo_model_test test/test_duo_model.py
          TIMEOUT 60)
  ament_add_pytest_test(
          packaging_test test/test_packaging.py
          TIMEOUT 30)
  # The Objectives the integration test runs, against its budgets and the start pose.
  ament_add_pytest_test(
          objective_ci_policy_test test/test_objective_ci_policy.py
          TIMEOUT 30
          ENV MOVEIT_CONFIG_PACKAGE=market_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR})
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
