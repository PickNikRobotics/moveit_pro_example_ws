cmake_minimum_required(VERSION 3.22)
project(lab_sim_behaviors CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_pro_behavior
  moveit_pro_behavior_interface
  moveit_studio_vision
  moveit_studio_vision_msgs
  pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  lab_sim_behaviors
  SHARED
  src/compute_tray_place_positions_using_apriltags.cpp
  src/register_behaviors.cpp)
target_include_directories(
  lab_sim_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lab_sim_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS lab_sim_behaviors
  EXPORT lab_sim_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(lab_sim_behaviors)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface lab_sim_behaviors_plugin_description.xml)

ament_export_targets(lab_sim_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
