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

#include <fstream>
#include <iostream>
#include <memory>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: market_sim_test_behavior_manifest OUTPUT_XML LOADER [LOADER ...]\n";
    return 2;
  }

  // Registration reads the installed plugins' manifests without constructing or ticking motion nodes.
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  BT::BehaviorTreeFactory factory;
  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  for (int index = 2; index < argc; ++index)
  {
    auto plugin = loader.createUniqueInstance(argv[index]);
    plugin->registerBehaviors(factory, context);
  }

  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(argv[1]);
  output << BT::writeTreeNodesModelXML(factory, true);
  return 0;
}
