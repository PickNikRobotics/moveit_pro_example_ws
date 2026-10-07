cmake_minimum_required(VERSION 3.22)
project(market_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    maps
    mjcf
    objectives
    params
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The generated scene, map and Objectives against the generator, and the compiled MuJoCo model.
  ament_add_pytest_test(
          market_scene_test test/test_market_scene.py
          TIMEOUT 120)
endif()

ament_package()
