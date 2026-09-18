cmake_minimum_required(VERSION 3.3)

project(fanuc_resources)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
