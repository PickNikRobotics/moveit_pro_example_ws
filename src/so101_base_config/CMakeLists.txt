cmake_minimum_required(VERSION 3.22)
project(so101_base_config)

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

install(PROGRAMS
  script/calibrate_so101.py
  script/so101_arm_bridge.py
  script/verify_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          so101_hardware_plugin_test test/test_hardware_plugin.py
          TIMEOUT 60)
  ament_add_pytest_test(
          so101_arm_bridge_test test/test_so101_arm_bridge.py
          TIMEOUT 60)
  ament_add_pytest_test(
          so101_arm_bridge_launch_test test/test_so101_arm_bridge_launch.py
          TIMEOUT 60)
  ament_add_pytest_test(
          so101_drivers_launch_test test/test_so101_drivers_launch.py
          TIMEOUT 60)
  ament_add_pytest_test(
          so101_verify_calibration_test test/test_verify_calibration.py
          TIMEOUT 60)
  ament_add_pytest_test(
          so101_calibrate_so101_test test/test_calibrate_so101.py
          TIMEOUT 60)
endif()

ament_package()
