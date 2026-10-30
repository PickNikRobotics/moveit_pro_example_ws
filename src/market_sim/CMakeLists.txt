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
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/navigation_scan_relay.py
    scripts/stamp_nav_velocity.py
    scripts/lidar_flattener.py
    scripts/reset_relocalizer.py
    scripts/scan_merger.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
          reset_relocalizer_test test/test_reset_relocalizer.py
          TIMEOUT 30)
  ament_add_pytest_test(
          camera_renderer_test test/test_camera_renderer.py
          TIMEOUT 30)
  ament_add_pytest_test(
          ground_truth_navigation_test test/test_ground_truth_navigation.py
          TIMEOUT 30)
  ament_add_pytest_test(
          scan_merger_test test/test_scan_merger.py
          TIMEOUT 30)
  # The generated scene, map and Objectives against the generator, and the compiled MuJoCo model.
  ament_add_pytest_test(
          market_scene_test test/test_market_scene.py
          TIMEOUT 120)
  # The Nav2 footprint of both costmaps, and the aisle goals against the map.
  ament_add_pytest_test(
          nav2_footprint_test test/test_nav2_footprint.py
          TIMEOUT 60)
endif()

ament_package()
