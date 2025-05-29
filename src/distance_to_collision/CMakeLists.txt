cmake_minimum_required(VERSION 3.22)
project(distance_to_collision CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)


add_library(
  distance_to_collision
  SHARED
  src/distance_to_collision.cpp
  src/register_behaviors.cpp)
target_include_directories(
  distance_to_collision
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(distance_to_collision
                          ${THIS_PACKAGE_INCLUDE_DEPENDS}
                          PCL
                          pcl_conversions)

# Install Libraries
install(
  TARGETS distance_to_collision
  EXPORT distance_to_collisionTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(distance_to_collision)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface distance_to_collision_plugin_description.xml)

ament_export_targets(distance_to_collisionTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
