#!/bin/bash

git subtree pull --prefix src/example_behaviors https://github.com/PickNikRobotics/example_behaviors main --squash
git subtree pull --prefix src/lab_sim https://github.com/PickNikRobotics/lab_sim main --squash
git subtree pull --prefix src/moveit_pro_ur_configs https://github.com/PickNikRobotics/moveit_pro_ur_configs main --squash
git subtree pull --prefix src/moveit_pro_kinova_configs https://github.com/PickNikRobotics/moveit_pro_kinova_configs main --squash
git subtree pull --prefix src/moveit_pro_mobile_manipulation https://github.com/PickNikRobotics/moveit_pro_mobile_manipulation main --squash
git subtree pull --prefix src/fanuc_sim https://github.com/PickNikRobotics/fanuc_sim main --squash
git subtree pull --prefix src/external_dependencies/ridgeback https://github.com/sjahr/ridgeback ros2 --squash
git subtree pull --prefix src/external_dependencies/ros2_robotiq_gripper https://github.com/PickNikRobotics/ros2_robotiq_gripper main --squash
git subtree pull --prefix src/external_dependencies/serial https://github.com/tylerjw/serial.git ros2 --squash
git submodule update --recursive --init
