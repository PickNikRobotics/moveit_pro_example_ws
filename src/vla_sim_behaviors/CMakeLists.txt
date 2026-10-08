cmake_minimum_required(VERSION 3.22)
project(vla_sim_behaviors)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS geometry_msgs moveit_pro_behavior_interface
                                 pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(fmt REQUIRED)

add_library(
  vla_sim_behaviors
  SHARED
  src/cube_stack_trial.cpp
  src/sample_cube_stack_trial.cpp
  src/register_behaviors.cpp)
target_include_directories(
  vla_sim_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vla_sim_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(vla_sim_behaviors fmt::fmt)

install(
  TARGETS vla_sim_behaviors
  EXPORT vla_sim_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(vla_sim_behaviors)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface vla_sim_behaviors_plugin_description.xml)

ament_export_targets(vla_sim_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
