cmake_minimum_required(VERSION 3.22)
project(so101_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config description launch objectives simulation_objectives waypoints
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
