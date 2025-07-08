cmake_minimum_required(VERSION 3.22)
project(detect_aruco_tag CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior_interface pluginlib cv_bridge)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core)

add_library(
  detect_aruco_tag
  SHARED
  src/detect_aruco_tag.cpp
  src/register_behaviors.cpp)
target_include_directories(
  detect_aruco_tag
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(detect_aruco_tag
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(detect_aruco_tag Eigen3::Eigen ${OpenCV_LIBS})

# Install Libraries
install(
  TARGETS detect_aruco_tag
  EXPORT detect_aruco_tagTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(detect_aruco_tag)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface detect_aruco_tag_plugin_description.xml)

ament_export_targets(detect_aruco_tagTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
