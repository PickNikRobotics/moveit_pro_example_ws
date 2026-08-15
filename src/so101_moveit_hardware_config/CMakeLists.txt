cmake_minimum_required(VERSION 3.22)
project(so101_moveit_hardware_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch objectives DESTINATION share/${PROJECT_NAME})

ament_package()
