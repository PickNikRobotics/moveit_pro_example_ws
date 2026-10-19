// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct Scenario
{
  std::deque<bool> choose_right{ true };
  std::deque<bool> red;
  std::deque<bool> green;
  std::string fail_at;
  int fail_on_call = 1;
  std::string hold_at;
  std::map<std::string, int> calls;
  std::vector<std::string> motions;
  int halts = 0;
};

bool consume(std::deque<bool>& results)
{
  if (results.empty())
  {
    return false;
  }
  const bool result = results.front();
  results.pop_front();
  return result;
}

class Operation : public BT::StatefulActionNode
{
public:
  Operation(const std::string& name, const BT::NodeConfig& config, Scenario* scenario)
    : BT::StatefulActionNode(name, config), scenario_(*scenario)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("operation"), BT::InputPort<std::string>("arm") };
  }

private:
  BT::NodeStatus onStart() override
  {
    const auto operation = getInput<std::string>("operation").value();
    const auto key = operation + ":" + getInput<std::string>("arm").value();
    const int call = ++scenario_.calls[key];
    if (operation == "BiasedCoinFlip")
    {
      return consume(scenario_.choose_right) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (operation == "Find Red Block" || operation == "Find Green Block")
    {
      return consume(operation == "Find Red Block" ? scenario_.red : scenario_.green) ? BT::NodeStatus::SUCCESS :
                                                                                        BT::NodeStatus::FAILURE;
    }
    if (operation == "Execute MTC Solution" || operation == "Move to Waypoint" ||
        operation.find("Gripper") != std::string::npos)
    {
      scenario_.motions.push_back(key);
    }
    if (key == scenario_.fail_at && call == scenario_.fail_on_call)
    {
      return BT::NodeStatus::FAILURE;
    }
    return key == scenario_.hold_at ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning() override
  {
    const auto key = getInput<std::string>("operation").value() + ":" + getInput<std::string>("arm").value();
    return key == scenario_.fail_at ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    ++scenario_.halts;
  }

  Scenario& scenario_;
};

// Keep the shipped tree's controls, decorators, scripts and pre/postconditions.
// Substitute only the hardware/perception boundaries, avoiding SAM and robot dependencies.
void substituteOperations(tinyxml2::XMLElement* element, std::string arm = "")
{
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string id = child->Attribute("ID") ? child->Attribute("ID") : "";
    if (id == "Find Red Block" || id == "Find Green Block")
    {
      arm = id == "Find Red Block" ? "right" : "left";
    }
  }
  const std::string tag = element->Name();
  const std::string id = element->Attribute("ID") ? element->Attribute("ID") : "";
  if (tag == "SubTree" || (tag == "Action" && id != "Script"))
  {
    std::vector<std::string> attributes;
    for (auto* attribute = element->FirstAttribute(); attribute; attribute = attribute->Next())
    {
      if (attribute->Name()[0] != '_')
      {
        attributes.emplace_back(attribute->Name());
      }
    }
    for (const auto& attribute : attributes)
    {
      element->DeleteAttribute(attribute.c_str());
    }
    element->SetName("Action");
    element->SetAttribute("ID", "Operation");
    element->SetAttribute("operation", id.c_str());
    element->SetAttribute("arm", arm.c_str());
  }
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    substituteOperations(child, arm);
  }
}

class SortBlocks : public ::testing::Test
{
public:
  BT::Tree makeTree()
  {
    tinyxml2::XMLDocument source;
    if (source.LoadFile(SORT_BLOCKS_XML) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("Cannot load Sort Blocks XML");
    }
    source.RootElement()->DeleteChild(source.RootElement()->FirstChildElement("TreeNodesModel"));
    substituteOperations(source.RootElement()->FirstChildElement("BehaviorTree"));
    tinyxml2::XMLPrinter printer;
    source.Print(&printer);
    factory.registerNodeType<Operation>("Operation", &scenario);
    return factory.createTreeFromText(printer.CStr());
  }

  static BT::NodeStatus finish(BT::Tree& tree)
  {
    for (int tick = 0; tick < 100; ++tick)
    {
      const auto status = tree.tickExactlyOnce();
      if (status != BT::NodeStatus::RUNNING)
      {
        return status;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  Scenario scenario;
  BT::BehaviorTreeFactory factory;
};

TEST_F(SortBlocks, EmptyTableSucceedsWithoutMotion)
{
  auto tree = makeTree();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(scenario.motions.empty());
}

TEST_F(SortBlocks, BothArmsCompleteBeforeExhaustionSucceeds)
{
  scenario.choose_right = { true, false, true };
  scenario.red = { true, false };
  scenario.green = { true, false };
  auto tree = makeTree();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:right"], 2);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:left"], 2);
  EXPECT_EQ(scenario.calls["Move to Waypoint:right"], 1);
  EXPECT_EQ(scenario.calls["Move to Waypoint:left"], 1);
}

class SortBlocksMotionFailure : public SortBlocks, public ::testing::WithParamInterface<std::pair<std::string, int>>
{
};

TEST_P(SortBlocksMotionFailure, AbortFailsImmediatelyWithoutTryingTheOtherArm)
{
  scenario.red = { true };
  scenario.green = { true };
  scenario.fail_at = GetParam().first;
  scenario.fail_on_call = GetParam().second;
  scenario.choose_right = { scenario.fail_at.find(":right") != std::string::npos };
  auto tree = makeTree();
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  ASSERT_FALSE(scenario.motions.empty());
  EXPECT_EQ(scenario.motions.back(), scenario.fail_at);
  const std::string other_arm = scenario.fail_at.find(":right") != std::string::npos ? "left" : "right";
  EXPECT_EQ(scenario.calls["Execute MTC Solution:" + other_arm], 0);

  // Re-running this same tree must not retain the previous motion failure.
  scenario.red.clear();
  scenario.green.clear();
  scenario.fail_at.clear();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
}

INSTANTIATE_TEST_SUITE_P(
    GraspPlaceGripperAndHome, SortBlocksMotionFailure,
    ::testing::Values(std::make_pair("Open Right Gripper:right", 1), std::make_pair("Open Right Gripper:right", 2),
                      std::make_pair("Close Right Gripper:right", 1), std::make_pair("Execute MTC Solution:right", 1),
                      std::make_pair("Execute MTC Solution:right", 2), std::make_pair("Move to Waypoint:right", 1),
                      std::make_pair("Open Left Gripper:left", 1), std::make_pair("Open Left Gripper:left", 2),
                      std::make_pair("Close Left Gripper:left", 1), std::make_pair("Execute MTC Solution:left", 1),
                      std::make_pair("Execute MTC Solution:left", 2), std::make_pair("Move to Waypoint:left", 1)));

TEST_F(SortBlocks, ControllerAbortAfterRunningFailsWithoutMovingOtherArm)
{
  scenario.red = { true };
  scenario.green = { true };
  scenario.hold_at = "Execute MTC Solution:right";
  auto tree = makeTree();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  scenario.fail_at = scenario.hold_at;
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:left"], 0);
  EXPECT_EQ(scenario.motions.back(), scenario.fail_at);
}

TEST_F(SortBlocks, CancellationHaltsMotion)
{
  scenario.red = { true };
  scenario.hold_at = "Execute MTC Solution:right";
  auto tree = makeTree();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(scenario.halts, 1);
  scenario.hold_at.clear();
  scenario.red.clear();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
}
}  // namespace
