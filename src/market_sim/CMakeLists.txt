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
    scripts/scan_merger.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
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
  ament_add_pytest_test(
          base_twist_to_planar_test test/test_base_twist_to_planar.py
          TIMEOUT 30)
  ament_add_pytest_test(
          odometry_joint_state_publisher_test test/test_odometry_joint_state_publisher.py
          TIMEOUT 30)
endif()

ament_package()
