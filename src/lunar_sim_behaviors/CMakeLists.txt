cmake_minimum_required(VERSION 3.22)
project(lunar_sim_behaviors)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

# moveit_pro_behavior's exported CMake target links moveit_studio_internal_msgs without declaring
# it as a find_package dependency of its own Config.cmake, so it must already be in scope here.
find_package(moveit_studio_internal_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_pro_behavior
  moveit_pro_behavior_interface
  pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  lunar_sim_behaviors
  SHARED
  src/publish_twist_stamped.cpp
  src/register_behaviors.cpp)
target_include_directories(
  lunar_sim_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lunar_sim_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS lunar_sim_behaviors
  EXPORT lunar_sim_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(lunar_sim_behaviors)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface lunar_sim_behaviors_plugin_description.xml)

ament_export_targets(lunar_sim_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
