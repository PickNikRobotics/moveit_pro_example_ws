cmake_minimum_required(VERSION 3.22)
project(pushing_behaviors CXX)

find_package(moveit_studio_common REQUIRED)
find_package(example_interfaces REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        moveit_studio_behavior
        moveit_studio_behavior_interface
        geometry_msgs
        pluginlib
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        fmt
        visualization_msgs
        example_interfaces)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(OsqpEigen REQUIRED)

add_library(
  pushing_behaviors
  SHARED
  src/T_pushing.cpp
  src/register_behaviors.cpp)
target_include_directories(
        pushing_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pushing_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(pushing_behaviors fmt::fmt OsqpEigen::OsqpEigen)

# Install Libraries
install(
  TARGETS pushing_behaviors
  EXPORT pushing_behaviorsTargets
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

ament_export_targets(pushing_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
