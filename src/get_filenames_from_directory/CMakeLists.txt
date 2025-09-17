cmake_minimum_required(VERSION 3.22)
project(get_filenames_from_directory CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface pluginlib)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  get_filenames_from_directory
  SHARED
  src/get_filenames_from_directory.cpp
  src/register_behaviors.cpp
  src/push_back_vector.cpp
  src/string_to_int.cpp)
target_include_directories(
  get_filenames_from_directory
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(get_filenames_from_directory
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS get_filenames_from_directory
  EXPORT get_filenames_from_directoryTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(get_filenames_from_directory)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface get_filenames_from_directory_plugin_description.xml)

ament_export_targets(get_filenames_from_directoryTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
