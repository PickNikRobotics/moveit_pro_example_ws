cmake_minimum_required(VERSION 3.22)
project(vla_sim)

find_package(ament_cmake REQUIRED)

# A self-contained MoveIt Pro config: MuJoCo description (scene + arm/gripper),
# MoveIt/control config, objectives, waypoints, and the agent-bridge launch.
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
  script/get_action_chunk_adapter.py
  script/joint_command_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # docker/test_vla_inference_server.py is not wired here: it needs
  # torch/lerobot and runs inside the inference_server container instead
  # (see docker/README.md).
  ament_add_pytest_test(test_get_action_chunk_adapter
    test/test_get_action_chunk_adapter.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_joint_command_bridge
    test/test_joint_command_bridge.py
    TIMEOUT 60
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
