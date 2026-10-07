cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_sim)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config description launch mjcf params DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS script/lidar_flattener.py script/odometry_joint_state_publisher.py script/scan_merger.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_odometry_joint_state_publisher test/test_odometry_joint_state_publisher.py)
endif()

ament_package()
