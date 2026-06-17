cmake_minimum_required(VERSION 3.22)
project(lab_sim_collision_monitor)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Parallel-monitor Layer-2 integration test: brings up the lab_sim MoveIt Pro
  # backend AND the licensed collision_monitor node together, then asserts both
  # (a) no false trips while a valid objective runs, and (b) a deliberate
  # self-collision pose DOES trip.
  #
  # Requires a valid MOVEIT_LICENSE_KEY in the environment (the monitor engine
  # ctor is license-gated). Long timeout: sim bring-up + objective execution.
  ament_add_pytest_test(
    collision_monitor_integration_test
    test/collision_monitor_integration_test.py
    TIMEOUT 900
    ENV
      MOVEIT_CONFIG_PACKAGE=lab_sim
      MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR})
endif()

ament_package()
