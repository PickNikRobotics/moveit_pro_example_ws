cmake_minimum_required(VERSION 3.22)
project(pi0_behaviors)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_pro_behavior
  moveit_pro_behavior_interface
  pluginlib
  rclcpp
  trajectory_msgs)

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(pi0_behaviors SHARED
  src/get_pi0_trajectory.cpp
  src/register_behaviors.cpp)

target_include_directories(pi0_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pi0_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS pi0_behaviors EXPORT pi0_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface pi0_behaviors_plugin_description.xml)

ament_export_targets(pi0_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
