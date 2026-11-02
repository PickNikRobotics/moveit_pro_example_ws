cmake_minimum_required(VERSION 3.22)
project(kdl_gravity_compensation_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface 4.45.2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gravity_model.cpp
                                   src/gravity_compensation_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC controller_interface::controller_interface
         hardware_interface::hardware_interface
         kdl_parser::kdl_parser
         orocos-kdl
         pluginlib::pluginlib
         ${rcl_interfaces_TARGETS}
         rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle
         realtime_tools::realtime_tools
         ${sensor_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_gravity_model test/test_gravity_model.cpp)
  target_link_libraries(test_gravity_model ${PROJECT_NAME})
  ament_add_gtest(test_gravity_compensation_controller
                  test/test_gravity_compensation_controller.cpp TIMEOUT 60)
  target_link_libraries(test_gravity_compensation_controller ${PROJECT_NAME}
                        ${lifecycle_msgs_TARGETS})
  ament_add_gtest(test_controller_chaining test/test_controller_chaining.cpp TIMEOUT 60)
  target_link_libraries(test_controller_chaining ${PROJECT_NAME}
                        controller_manager::controller_manager
                        ${controller_manager_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface kdl_parser orocos_kdl_vendor
                          orocos_kdl pluginlib rcl_interfaces rclcpp rclcpp_lifecycle realtime_tools sensor_msgs)
ament_package()
