cmake_minimum_required(VERSION 3.22)
project(franka_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
find_package(franka_msgs REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_pro_behavior moveit_pro_behavior_interface pluginlib 
franka_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  franka_behaviors
  SHARED 
  src/franka_grasp_action.cpp
  src/register_behaviors.cpp)
target_include_directories(
  franka_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(franka_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS franka_behaviors
  EXPORT franka_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(franka_behaviors)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface franka_behaviors_plugin_description.xml)

ament_export_targets(franka_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
