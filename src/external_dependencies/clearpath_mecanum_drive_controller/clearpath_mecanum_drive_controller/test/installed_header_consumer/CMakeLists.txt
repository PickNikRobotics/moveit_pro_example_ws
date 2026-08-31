# Modified by PickNik Inc., 2026.
cmake_minimum_required(VERSION 3.16)
project(clearpath_mecanum_drive_controller_installed_header_consumer LANGUAGES CXX)

find_package(clearpath_mecanum_drive_controller REQUIRED)

add_executable(installed_header_consumer main.cpp)
target_link_libraries(
  installed_header_consumer
  PRIVATE clearpath_mecanum_drive_controller::clearpath_mecanum_drive_controller
)
