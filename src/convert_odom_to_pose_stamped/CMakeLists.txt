cmake_minimum_required(VERSION 3.22)
project(convert_odom_to_pose_stamped CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_pro_behavior_interface pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  convert_odom_to_pose_stamped
  SHARED
  src/convert_odom_to_pose_stamped.cpp
  src/register_behaviors.cpp)
target_include_directories(
  convert_odom_to_pose_stamped
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(convert_odom_to_pose_stamped
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS convert_odom_to_pose_stamped
  EXPORT convert_odom_to_pose_stampedTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(convert_odom_to_pose_stamped)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface convert_odom_to_pose_stamped_plugin_description.xml)

ament_export_targets(convert_odom_to_pose_stampedTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
