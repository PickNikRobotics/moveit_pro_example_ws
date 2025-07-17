cmake_minimum_required(VERSION 3.8)
project(slider_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qt_gui REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# Add executable
add_executable(slider_gui_node src/slider_gui_node.cpp)
ament_target_dependencies(slider_gui_node rclcpp std_msgs sensor_msgs)
target_link_libraries(slider_gui_node Qt5::Widgets Qt5::Core)

# Install executable
install(TARGETS
  slider_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
