cmake_minimum_required(VERSION 3.22)
project(vo_vslam_evaluation_harness)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/vo_vslam_evaluation_node
  scripts/tartanair_stereo_playback
  scripts/smoke_runtime_adapter.py
  scripts/run_sequence_smoke.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(${PROJECT_NAME}_tests test TIMEOUT 120)
  add_launch_test(launch_test/test_ros_node.py TIMEOUT 30)
endif()

ament_package()
