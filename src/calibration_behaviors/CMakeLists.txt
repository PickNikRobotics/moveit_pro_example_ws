cmake_minimum_required(VERSION 3.22)
project(calibration_behaviors CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface moveit_studio_common pluginlib geometry_msgs tf2_geometry_msgs Eigen3 tf2 tf2_eigen yaml-cpp)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  calibration_behaviors
  SHARED
  src/average_pose_stamped_vector.cpp
  src/calibrate_camera_pose.cpp
  src/save_calibration_pose_yaml.cpp
  src/register_behaviors.cpp)
target_include_directories(
  calibration_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(calibration_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS calibration_behaviors
  EXPORT calibration_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(calibration_behaviors)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface calibration_behaviors_plugin_description.xml)

ament_export_targets(calibration_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
