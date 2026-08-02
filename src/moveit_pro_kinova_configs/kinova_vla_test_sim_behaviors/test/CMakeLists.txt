find_package(ament_cmake_gtest REQUIRED)

# rclcpp::init wedges on discovery when it shares a ROS domain with a live backend,
# so every test that brings up a node gets a domain of its own -- one each, since
# colcon runs them in parallel and they would otherwise discover each other. The
# domain is read while the rmw library loads, too early for the test's own main().
ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp ENV "ROS_DOMAIN_ID=89")
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gtest(test_compute_top_down_keyposes test_compute_top_down_keyposes.cpp)
target_link_libraries(test_compute_top_down_keyposes kinova_vla_test_sim_behaviors)
ament_target_dependencies(test_compute_top_down_keyposes ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gtest(test_plan_joint_spline_through_poses test_plan_joint_spline_through_poses.cpp)
target_link_libraries(test_plan_joint_spline_through_poses kinova_vla_test_sim_behaviors)
ament_target_dependencies(test_plan_joint_spline_through_poses ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gtest(test_send_gripper_command test_send_gripper_command.cpp ENV "ROS_DOMAIN_ID=90")
target_link_libraries(test_send_gripper_command kinova_vla_test_sim_behaviors)
ament_target_dependencies(test_send_gripper_command ${THIS_PACKAGE_INCLUDE_DEPENDS})
