cmake_minimum_required(VERSION 3.22)
project(so101_moveit_hardware_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch objectives waypoints DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/external_driver_contract.py
    scripts/external_driver_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_external_driver_contract
    test/test_external_driver_contract.py
  )
  ament_add_pytest_test(
    test_teleoperation_contract
    test/test_teleoperation_contract.py
  )
  ament_add_pytest_test(
    test_hardware_motion_contract
    test/test_hardware_motion_contract.py
  )
endif()

ament_package()
