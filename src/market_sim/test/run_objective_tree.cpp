// Copyright 2026 PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 3)
  {
    std::cerr << "Usage: market_sim_test_run_objective_tree OBJECTIVE_XML [STOP_AFTER_SEC]\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("market_sim_objective_test");
  const auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, /*tf2_spin_thread=*/false,
                                                                                /*load_robot_model=*/false);
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  BT::BehaviorTreeFactory factory;
  for (const std::string plugin_name :
       { "moveit_pro::behaviors::CoreBehaviorsLoader", "moveit_pro::behaviors::VisionBehaviorsLoader",
         "moveit_pro::behaviors::MujocoBehaviorsLoader", "moveit_pro::behaviors::ConverterBehaviorsLoader" })
  {
    auto plugin = loader.createUniqueInstance(plugin_name);
    plugin->registerBehaviors(factory, context);
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::jthread callbacks([&executor]() { executor.spin(); });
  // Each process constructs a fresh tree, matching a new Objective-server execution after failure.
  BT::Tree tree = factory.createTreeFromFile(argv[1]);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  if (argc == 2)
  {
    status = tree.tickWhileRunning();
  }
  else
  {
    // Halting the running tree matches an Objective stop request.
    const auto stop_at =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                               std::chrono::duration<double>(std::stod(argv[2])));
    status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < stop_at)
    {
      tree.sleep(std::chrono::milliseconds(10));
      status = tree.tickOnce();
    }
  }
  tree.haltTree();
  executor.cancel();
  callbacks.join();
  rclcpp::shutdown();
  std::cout << "OBJECTIVE_RESULT=" << (status == BT::NodeStatus::RUNNING ? "STOPPED" : BT::toStr(status)) << '\n';
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
