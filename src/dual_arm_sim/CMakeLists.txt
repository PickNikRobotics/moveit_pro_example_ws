cmake_minimum_required(VERSION 3.22)
project(dual_arm_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    maps
    objectives
    params
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/base_twist_to_planar.py
    scripts/odometry_joint_state_publisher.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(behaviortree_cpp_picknik REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  ament_add_gtest(test_sort_blocks test/test_sort_blocks.cpp)
  ament_target_dependencies(test_sort_blocks behaviortree_cpp_picknik)
  target_link_libraries(test_sort_blocks tinyxml2::tinyxml2)
  target_compile_definitions(test_sort_blocks PRIVATE
    SORT_BLOCKS_XML="${CMAKE_CURRENT_SOURCE_DIR}/objectives/sort_blocks.xml")
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          scene_test test/test_scene.py
          TIMEOUT 120)
  ament_add_pytest_test(
          base_twist_to_planar_test test/test_base_twist_to_planar.py
          TIMEOUT 30)
  ament_add_pytest_test(
          odometry_joint_state_publisher_test test/test_odometry_joint_state_publisher.py
          TIMEOUT 30)
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. The default ~/.ros/log/ lives on the doomed
  # container filesystem and never gets uploaded, making post-mortem of
  # objective failures impossible.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=dual_arm_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
