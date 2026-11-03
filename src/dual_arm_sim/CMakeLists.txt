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
  find_package(moveit_pro_behavior_interface REQUIRED)
  find_package(moveit_pro_base REQUIRED)
  find_package(moveit_pro_task_constructor_core REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io)
  ament_add_gtest(test_block_surface test/test_block_surface.cpp)
  ament_target_dependencies(test_block_surface
    behaviortree_cpp_picknik moveit_pro_behavior_interface pluginlib
    sensor_msgs geometry_msgs rclcpp pcl_conversions)
  target_link_libraries(test_block_surface tinyxml2::tinyxml2 ${PCL_LIBRARIES})
  target_compile_features(test_block_surface PRIVATE cxx_std_17)
  target_compile_definitions(test_block_surface PRIVATE
    FIND_WITH_PROMPT_XML="${CMAKE_CURRENT_SOURCE_DIR}/objectives/find_with_prompt.xml"
    BLOCK_CLOUD_DATA="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  add_executable(block_transfer_plan_probe test/block_transfer_plan_probe.cpp)
  target_compile_features(block_transfer_plan_probe PRIVATE cxx_std_20)
  target_link_libraries(block_transfer_plan_probe PRIVATE
    moveit_pro_behavior_interface::behavior_context
    moveit_pro_base::robot_model_loader
    moveit_pro_base::moveit_pro_base_planning_scene
    moveit_pro_task_constructor_core::moveit_pro_task_constructor_core
    pluginlib::pluginlib nlohmann_json::nlohmann_json tinyxml2::tinyxml2)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          scene_test test/test_scene.py
          TIMEOUT 120)
  ament_add_pytest_test(
          grasp_contacts_test test/test_grasp_contacts.py
          TIMEOUT 120)
  ament_add_pytest_test(
          block_transfer_test test/test_block_transfer.py
          TIMEOUT 210
          ENV BLOCK_TRANSFER_PROBE=$<TARGET_FILE:block_transfer_plan_probe>)
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
              MOVEIT_PRO_CAMERA_RENDERER=opengl
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
