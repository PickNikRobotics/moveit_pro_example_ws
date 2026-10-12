cmake_minimum_required(VERSION 3.22)
project(factory_sim)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(behaviortree_cpp_picknik REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(factory_pick_behaviors SHARED src/is_bracket_pick_start_current.cpp
            src/get_joint_state_from_topic.cpp
            src/pad_planning_scene_for_motion.cpp
            src/is_lookahead_enabled.cpp
            src/register_pick_behaviors.cpp)
target_compile_features(factory_pick_behaviors PUBLIC cxx_std_20)
target_include_directories(
  factory_pick_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(factory_pick_behaviors moveit_pro_behavior_interface
                          pluginlib moveit_task_constructor_msgs moveit_msgs
                          behaviortree_cpp_picknik sensor_msgs)
pluginlib_export_plugin_description_file(moveit_pro_behavior_interface
                                        factory_pick_behaviors.xml)
install(TARGETS factory_pick_behaviors LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY
    config
    description
    launch
    models
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

# Mujoco models in this package use relative paths to assets such as meshes and textures
# To simplify building relative paths to assets in other packages, we will copy those assets into this project's file structure

# Paths we will copy from - the assets we want to easily reference in our Mujoco model
set(PICKNIK_ACCESSORIES_PATH "${picknik_accessories_DIR}/../mujoco_assets/")

# Path we will copy to - the folder where this robot config's Mujoco model is located
set(MUJOCO_MODEL_DIR "share/${PROJECT_NAME}/description/")

# Copy all contents into the installed description folder
install(DIRECTORY "${PICKNIK_ACCESSORIES_PATH}"
        DESTINATION "${MUJOCO_MODEL_DIR}"
        FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(moveit_pro_behavior REQUIRED)
  find_package(moveit_pro_behavior_interface REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(test_bin_configuration test/test_bin_configuration.cpp)
  ament_add_gtest(test_parallel_pick test/test_parallel_pick.cpp)
  target_compile_features(test_parallel_pick PRIVATE cxx_std_20)
  target_compile_definitions(test_parallel_pick PRIVATE
    FACTORY_OBJECTIVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/objectives")
  target_link_libraries(test_parallel_pick factory_pick_behaviors
                        moveit_pro_behavior::setup_mtc_from_solution
                        moveit_pro_behavior::repeat_unless_failure_each_tick
                        moveit_pro_test_utils::test_utils)
  ament_add_pytest_test(parallel_pick_benchmark_test
                        test/test_parallel_pick_benchmark.py)
  target_compile_features(test_bin_configuration PRIVATE cxx_std_20)
  target_compile_definitions(test_bin_configuration PRIVATE
    FACTORY_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    FACTORY_OBJECTIVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/objectives")
  target_link_libraries(test_bin_configuration
    moveit_pro_behavior::add_collision_box
    moveit_pro_behavior::transform_pose
    moveit_pro_behavior::get_element_of_vector
    moveit_pro_behavior::get_size_of_vector
    moveit_pro_behavior::log_message
    moveit_pro_test_utils::test_utils
    yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_bin_configuration moveit_pro_behavior_interface ament_index_cpp pluginlib)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Redirect ROS node logs into the test_results tree so the test-results CI
  # artifact ships them back too. The default ~/.ros/log/ lives on the doomed
  # container filesystem and never gets uploaded, making post-mortem of
  # objective failures impossible.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=factory_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
