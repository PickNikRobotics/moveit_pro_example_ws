// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_studio_kinova_pstop_manager/mock_kinova_client.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<moveit_studio::kinova_pstop_manager::MockKinovaClient>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  rclcpp::shutdown();
}
