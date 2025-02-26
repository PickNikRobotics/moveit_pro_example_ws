cmake_minimum_required(VERSION 3.22)
project(visualization_behaviors CXX)

find_package(moveit_studio_common REQUIRED)
find_package(example_interfaces REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
        moveit_studio_behavior
        moveit_studio_behavior_interface
        geometry_msgs
        pluginlib
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        visualization_msgs
        example_interfaces)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  visualization_behaviors
  SHARED
  src/example_publish_text_marker.cpp
  src/example_publish_arrow_marker.cpp
  src/register_behaviors.cpp)
target_include_directories(
        visualization_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(visualization_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS visualization_behaviors
  EXPORT visualization_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface example_behaviors_plugin_description.xml)

ament_export_targets(visualization_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
