cmake_minimum_required(VERSION 3.22)
project(vla_sim_behaviors)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  geometry_msgs
  moveit_pro_base
  moveit_pro_behavior
  moveit_pro_behavior_interface
  moveit_studio_internal_msgs
  nlohmann_json
  pluginlib
  rclcpp_action
  sensor_msgs
  tf2_eigen
  trajectory_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  vla_sim_behaviors
  SHARED
  src/compute_top_down_keyposes.cpp
  src/plan_joint_spline_through_poses.cpp
  src/send_gripper_command.cpp
  src/wait_for_episode_start.cpp
  src/register_behaviors.cpp)
target_include_directories(
  vla_sim_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vla_sim_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS vla_sim_behaviors
  EXPORT vla_sim_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  moveit_pro_behavior_test(vla_sim_behaviors)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface vla_sim_behaviors_plugin_description.xml)

ament_export_targets(vla_sim_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
