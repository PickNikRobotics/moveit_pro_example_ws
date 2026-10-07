cmake_minimum_required(VERSION 3.22)
project(franka_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_pro_behavior_interface pluginlib geometry_msgs moveit_msgs
                                 tf2_geometry_msgs tf2_ros)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

# The spine Behaviors. The real-hardware Behaviors in hardware/ need franka_msgs and are not built.
add_library(franka_behaviors SHARED src/create_spine_state.cpp src/get_spine_state_for_pose_height.cpp
                                    src/register_behaviors.cpp)
target_include_directories(franka_behaviors PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(franka_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS franka_behaviors
  EXPORT franka_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(test_spine_height test/test_spine_height.cpp)
  target_include_directories(test_spine_height PRIVATE include)
  ament_add_ros_isolated_gtest(test_behavior_plugins test/test_behavior_plugins.cpp)
  ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})
  ament_add_ros_isolated_gtest(test_create_spine_state test/test_create_spine_state.cpp)
  ament_target_dependencies(test_create_spine_state ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

# Lets MoveIt Pro's Behavior loaders find FrankaBehaviorsLoader.
pluginlib_export_plugin_description_file(moveit_pro_behavior_interface franka_behaviors_plugin_description.xml)

ament_export_targets(franka_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
