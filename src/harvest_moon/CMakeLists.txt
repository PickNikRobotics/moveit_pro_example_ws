cmake_minimum_required(VERSION 3.22)
project(harvest_moon)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config launch objectives DESTINATION share/${PROJECT_NAME}/)

ament_package()
