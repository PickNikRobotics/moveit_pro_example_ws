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

install(PROGRAMS scripts/reset_relocalizer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
          reset_relocalizer_test test/test_reset_relocalizer.py
          TIMEOUT 30)
  # The generated scene, map and Objectives against the generator, and the compiled MuJoCo model.
  ament_add_pytest_test(
          market_scene_test test/test_market_scene.py
          TIMEOUT 120)
endif()

ament_package()
