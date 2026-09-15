cmake_minimum_required(VERSION 3.22)
project(hangar_sim_behaviors)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_pro_behavior
  moveit_pro_behavior_interface
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  hangar_sim_behaviors
  SHARED
  src/call_empty_service.cpp
  src/localization_gates.cpp
  src/project_pose_to_plane.cpp
  src/reinterpret_pose_frame.cpp
  src/scan_match_residual.cpp
  src/seed_localization_at_pose.cpp
  src/register_behaviors.cpp)
target_include_directories(
  hangar_sim_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hangar_sim_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS hangar_sim_behaviors
  EXPORT hangar_sim_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)


if(BUILD_TESTING)
  moveit_pro_behavior_test(hangar_sim_behaviors)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface hangar_sim_behaviors_plugin_description.xml)

ament_export_targets(hangar_sim_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
