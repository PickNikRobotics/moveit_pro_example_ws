cmake_minimum_required(VERSION 3.22)
project(example_behaviors CXX)

find_package(moveit_studio_common REQUIRED)
find_package(example_interfaces REQUIRED)
moveit_studio_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_studio_behavior moveit_studio_behavior_interface pluginlib 
moveit_studio_vision
moveit_studio_vision_msgs
PCL
pcl_conversions
pcl_ros
example_interfaces)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  example_behaviors
  SHARED 
  src/add_two_ints_service_client.cpp
  src/convert_mtc_solution_to_joint_trajectory.cpp
  src/delayed_message.cpp
  src/get_string_from_topic.cpp
  src/fibonacci_action_client.cpp
  src/hello_world.cpp
  src/publish_color_rgba.cpp
  src/setup_mtc_pick_from_pose.cpp
  src/setup_mtc_place_from_pose.cpp
  src/setup_mtc_wave_hand.cpp
  src/ndt_registration.cpp
  src/ransac_registration.cpp
  src/register_behaviors.cpp)
target_include_directories(
  example_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(example_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS example_behaviors
  EXPORT example_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(example_behaviors)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface example_behaviors_plugin_description.xml)

ament_export_targets(example_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
