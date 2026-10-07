cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_mock)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config description launch maps objectives params waypoints DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS script/base_twist_to_planar.py script/joint_state_odometry.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_base_twist_to_planar test/test_base_twist_to_planar.py)
  ament_add_pytest_test(test_joint_state_odometry test/test_joint_state_odometry.py)
endif()

ament_package()
