cmake_minimum_required(VERSION 3.22)
project(fr3_duo_mock)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config description launch objectives waypoints DESTINATION share/${PROJECT_NAME}/)

ament_package()
