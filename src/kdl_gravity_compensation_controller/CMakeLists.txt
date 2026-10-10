cmake_minimum_required(VERSION 3.22)
project(kdl_gravity_compensation_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gravity_model.cpp
                                   src/gravity_compensation_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC controller_interface::controller_interface
         kdl_parser::kdl_parser
         orocos-kdl
         pluginlib::pluginlib
         rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gravity_compensation_controller
                  test/test_gravity_compensation_controller.cpp)
  target_link_libraries(test_gravity_compensation_controller ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface kdl_parser orocos_kdl pluginlib
                          rclcpp_lifecycle)
ament_package()
