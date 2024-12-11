cmake_minimum_required(VERSION 3.22)
project(graspnet_example_behavior CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  graspnet_example_behavior
  SHARED
  src/graspnet_example_behavior.cpp
  src/register_behaviors.cpp)
target_include_directories(
  graspnet_example_behavior
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(graspnet_example_behavior
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS graspnet_example_behavior
  EXPORT graspnet_example_behaviorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(graspnet_example_behavior)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface graspnet_example_behavior_plugin_description.xml)

ament_export_targets(graspnet_example_behaviorTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
