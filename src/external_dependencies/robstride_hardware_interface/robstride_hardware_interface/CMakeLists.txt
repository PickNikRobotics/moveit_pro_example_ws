################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robstride_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(robstride_sdk REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robstride_interfaces REQUIRED)

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME}
  SHARED
  src/robstride_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${hardware_interface_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${robstride_sdk_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${hardware_interface_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${robstride_sdk_TARGETS}
  ${std_srvs_TARGETS}
  ${robstride_interfaces_TARGETS}
  ${realtime_tools_TARGETS}
)

pluginlib_export_plugin_description_file(hardware_interface robstride_hardware_interface_plugin.xml)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/setup_can.sh
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Drives the real lifecycle against a fake motor on a virtual CAN interface.
  # Skips itself when no vcan interface exists; see the test's header comment.
  ament_add_gtest(test_deactivate_torque test/test_deactivate_torque.cpp)
  target_include_directories(test_deactivate_torque PRIVATE test)
  target_link_libraries(test_deactivate_torque ${PROJECT_NAME})

  ament_add_gtest(test_fault_bits test/test_fault_bits.cpp)
  target_link_libraries(test_fault_bits ${PROJECT_NAME})

  ament_add_gtest(test_feedforward test/test_feedforward.cpp)
  target_link_libraries(test_feedforward ${PROJECT_NAME})
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  robstride_sdk
  robstride_interfaces
  realtime_tools
)

ament_package()
