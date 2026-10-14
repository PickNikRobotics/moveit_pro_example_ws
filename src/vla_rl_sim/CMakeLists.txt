cmake_minimum_required(VERSION 3.22)
project(vla_rl_sim)

find_package(ament_cmake REQUIRED)

# Inherits the description, control, and MoveIt config from vla_sim through
# based_on_package; installs only what this package adds.
install(
  DIRECTORY
    config
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          vla_rl_sim_spec_matches_runtime_test test/test_rl_spec_matches_runtime.py
          TIMEOUT 60)
endif()

ament_package()
