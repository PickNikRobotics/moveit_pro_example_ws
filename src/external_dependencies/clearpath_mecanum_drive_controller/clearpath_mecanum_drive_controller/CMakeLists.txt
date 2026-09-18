# Modified by PickNik Inc., 2026.
cmake_minimum_required(VERSION 3.16)
project(clearpath_mecanum_drive_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Detect ROS distro for API compatibility
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    set(ROS_DISTRO_COMPILE_DEFINITION ROS_DISTRO_JAZZY)
  elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ROS_DISTRO_COMPILE_DEFINITION ROS_DISTRO_HUMBLE)
  endif()
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  generate_parameter_library
  lifecycle_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(clearpath_mecanum_drive_controller_parameters
  src/clearpath_mecanum_drive_controller.yaml
  include/clearpath_mecanum_drive_controller/parameter_validators.hpp
)

add_library(
  clearpath_mecanum_drive_controller
  SHARED
  src/clearpath_mecanum_drive_controller.cpp
  src/odometry.cpp
)
target_compile_features(clearpath_mecanum_drive_controller PUBLIC cxx_std_17)
target_compile_definitions(
  clearpath_mecanum_drive_controller PUBLIC ${ROS_DISTRO_COMPILE_DEFINITION})
target_include_directories(clearpath_mecanum_drive_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(clearpath_mecanum_drive_controller PUBLIC
  clearpath_mecanum_drive_controller_parameters)
ament_target_dependencies(clearpath_mecanum_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(
  clearpath_mecanum_drive_controller PRIVATE "ACKERMANN_STEERING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface clearpath_mecanum_drive_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/clearpath_mecanum_drive_controller
)

install(
  TARGETS clearpath_mecanum_drive_controller clearpath_mecanum_drive_controller_parameters
  EXPORT export_clearpath_mecanum_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_clearpath_mecanum_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_load_mecanum_drive_controller test/test_load_mecanum_drive_controller.cpp)
  target_include_directories(test_load_mecanum_drive_controller PRIVATE include)
  target_link_libraries(test_load_mecanum_drive_controller clearpath_mecanum_drive_controller)
  ament_target_dependencies(
    test_load_mecanum_drive_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gtest(reference_validation_test test/reference_validation_test.cpp)
  target_include_directories(reference_validation_test PRIVATE include)

  add_test(
    NAME installed_header_consumer
    COMMAND
      "${CMAKE_COMMAND}"
      "-DCONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/installed_header_consumer"
      "-DCONSUMER_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/installed_header_consumer"
      "-DCMAKE_EXECUTABLE=${CMAKE_COMMAND}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/run_installed_header_consumer.cmake"
  )
endif()

ament_package()
