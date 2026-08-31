cmake_minimum_required(VERSION 3.3)

project(fanuc_lrmate200id_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
