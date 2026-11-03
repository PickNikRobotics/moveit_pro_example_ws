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
  std::deque<bool> red;
  std::deque<bool> green;
  std::string fail_at;
  int fail_on_call = 1;
  std::string hold_at;
  int hold_on_call = 1;
  std::map<std::string, int> calls;
  std::vector<std::string> motions;
  std::vector<std::string> transfer_steps;
  std::vector<double> carry_speeds;
  std::vector<double> carry_accelerations;
  int halts = 0;
};

[[nodiscard]] bool consume(std::deque<bool>& results)
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

  [[nodiscard]] static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("operation"),
             BT::InputPort<std::string>("arm"),
             BT::InputPort<double>("carry_speed"),
             BT::InputPort<double>("carry_acceleration"),
             BT::InputPort<std::string>("solution"),
             BT::OutputPort<std::string>("pickup_solution"),
             BT::OutputPort<std::string>("carry_solution"),
             BT::OutputPort<bool>("detections_empty", "{detections_empty}",
                                  "Successful segmentation found no objects.") };
  }

private:
  [[nodiscard]] BT::NodeStatus onStart() override
  {
    const std::string operation = getInput<std::string>("operation").value();
    const std::string key = operation + ":" + getInput<std::string>("arm").value();
    const int call = ++scenario_.calls[key];
    if (const auto speed = getInput<double>("carry_speed"); speed.has_value())
    {
      scenario_.carry_speeds.push_back(speed.value());
      scenario_.carry_accelerations.push_back(getInput<double>("carry_acceleration").value());
    }
    if (operation == "Find Red Block" || operation == "Find Green Block")
    {
      const bool failed = key == scenario_.fail_at && call == scenario_.fail_on_call;
      const bool found = !failed && consume(operation == "Find Red Block" ? scenario_.red : scenario_.green);
      // A failed segmentation/fit is not evidence that the table is empty.
      setOutput("detections_empty", !failed && !found);
      return found ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    if (operation == "Plan Block Transfer")
    {
      const std::string arm = getInput<std::string>("arm").value();
      setOutput("pickup_solution", arm + " pickup");
      setOutput("carry_solution", arm + " carry");
      scenario_.transfer_steps.push_back(key);
    }
    else if (operation == "Execute MTC Solution")
    {
      scenario_.transfer_steps.push_back(getInput<std::string>("solution").value());
    }
    else if (operation.find("Gripper") != std::string::npos)
    {
      scenario_.transfer_steps.push_back(key);
    }
    return key == scenario_.hold_at && call == scenario_.hold_on_call ? BT::NodeStatus::RUNNING :
                                                                        BT::NodeStatus::SUCCESS;
  }

  [[nodiscard]] BT::NodeStatus onRunning() override
  {
    const std::string key = getInput<std::string>("operation").value() + ":" + getInput<std::string>("arm").value();
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
    const std::string carry_speed = element->Attribute("carry_speed") ? element->Attribute("carry_speed") : "";
    const std::string carry_acceleration =
        element->Attribute("carry_acceleration") ? element->Attribute("carry_acceleration") : "";
    const std::string detections_empty =
        element->Attribute("detections_empty") ? element->Attribute("detections_empty") : "";
    std::map<std::string, std::string> solution_ports;
    for (const auto* port : { "solution", "pickup_solution", "carry_solution" })
    {
      if (const auto* value = element->Attribute(port))
      {
        solution_ports.emplace(port, value);
      }
    }
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
    for (const auto& [port, value] : solution_ports)
    {
      element->SetAttribute(port.c_str(), value.c_str());
    }
    if (!carry_speed.empty())
    {
      element->SetAttribute("carry_speed", carry_speed.c_str());
      element->SetAttribute("carry_acceleration", carry_acceleration.c_str());
    }
    if (!detections_empty.empty())
    {
      element->SetAttribute("detections_empty", detections_empty.c_str());
    }
  }
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    substituteOperations(child, arm);
  }
}

class SortBlocks : public ::testing::Test
{
public:
  [[nodiscard]] BT::Tree makeTree()
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

  [[nodiscard]] static BT::NodeStatus finish(BT::Tree& tree)
  {
    for (int tick = 0; tick < 100; ++tick)
    {
      const BT::NodeStatus status = tree.tickExactlyOnce();
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
  // GIVEN neither color has detections.
  auto tree = makeTree();
  // WHEN the sort loop checks both colors.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN success requires no motion.
  EXPECT_TRUE(scenario.motions.empty());
}

TEST_F(SortBlocks, BothArmsCompleteBeforeExhaustionSucceeds)
{
  // GIVEN one detected block of each color.
  scenario.red = { true, false };
  scenario.green = { true, false };
  auto tree = makeTree();
  // WHEN the sort loop completes.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN both arms pick, place and return home before exhaustion.
  EXPECT_EQ(scenario.calls["Execute MTC Solution:right"], 2);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:left"], 2);
  EXPECT_EQ(scenario.calls["Move to Waypoint:right"], 1);
  EXPECT_EQ(scenario.calls["Move to Waypoint:left"], 1);
}

TEST_F(SortBlocks, RegressionEmptyGreenDetectionsDoNotLeaveRedBlocksUnprocessed)
{
  // GIVEN three red blocks and no green blocks.
  scenario.red = { true, true, true, false };
  auto tree = makeTree();

  // WHEN the actual Sort Blocks control flow reaches terminal success.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);

  // THEN every red block must be picked and placed, and both colors must have been checked for exhaustion.
  EXPECT_EQ(scenario.calls["Execute MTC Solution:right"], 6);
  EXPECT_EQ(scenario.calls["Find Red Block:right"], 4);
  EXPECT_GE(scenario.calls["Find Green Block:left"], 1);
  EXPECT_TRUE(scenario.red.empty());
}

TEST_F(SortBlocks, RegressionBothColorsAreExhaustedBeforeSuccess)
{
  // GIVEN three blocks of each color.
  scenario.red = { true, true, true, false };
  scenario.green = { true, true, true, false };
  auto tree = makeTree();

  // WHEN the actual Sort Blocks control flow finishes.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);

  // THEN all six picks and placements occurred, and neither detection queue retains a block.
  EXPECT_EQ(scenario.calls["Execute MTC Solution:right"], 6);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:left"], 6);
  EXPECT_TRUE(scenario.red.empty());
  EXPECT_TRUE(scenario.green.empty());
  EXPECT_EQ(scenario.calls["Plan Block Transfer:right"], 3);
  EXPECT_EQ(scenario.calls["Plan Block Transfer:left"], 3);
  ASSERT_EQ(scenario.carry_speeds.size(), 6);
  for (const double speed : scenario.carry_speeds)
  {
    EXPECT_DOUBLE_EQ(speed, 0.25);
  }
  ASSERT_EQ(scenario.carry_accelerations.size(), 6);
  for (const double acceleration : scenario.carry_accelerations)
  {
    EXPECT_DOUBLE_EQ(acceleration, 0.0625);
  }
}

class SortBlocksMotionFailure : public SortBlocks, public ::testing::WithParamInterface<std::pair<std::string, int>>
{
};

TEST_P(SortBlocksMotionFailure, AbortFailsImmediatelyWithoutTryingTheOtherArm)
{
  // GIVEN a motion operation aborts for the selected arm.
  scenario.red = { true };
  scenario.green = { true };
  scenario.fail_at = GetParam().first;
  scenario.fail_on_call = GetParam().second;
  if (scenario.fail_at.find(":left") != std::string::npos)
  {
    scenario.red.clear();
  }
  auto tree = makeTree();
  // WHEN the abort reaches the sort loop.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN no later motion or other-arm attempt is allowed.
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
  // GIVEN the right controller is executing a trajectory.
  scenario.red = { true };
  scenario.green = { true };
  scenario.hold_at = "Execute MTC Solution:right";
  auto tree = makeTree();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // WHEN it aborts after returning RUNNING.
  scenario.fail_at = scenario.hold_at;
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN the left arm remains idle.
  EXPECT_EQ(scenario.calls["Execute MTC Solution:left"], 0);
  EXPECT_EQ(scenario.motions.back(), scenario.fail_at);
}

class SortBlocksPerceptionFailure : public SortBlocks, public ::testing::WithParamInterface<std::string>
{
};

TEST_P(SortBlocksPerceptionFailure, RegressionPerceptionErrorCannotSucceedAsExhaustion)
{
  // GIVEN a perception error for either color, not a valid empty detection result.
  scenario.red = { true };
  scenario.green = { true };
  scenario.fail_at = GetParam();
  if (scenario.fail_at == "Find Green Block:left")
  {
    scenario.red.clear();
  }
  auto tree = makeTree();

  // WHEN the Find subtree fails.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

  // THEN no arm may move, and a red perception error must not fall through to green.
  EXPECT_TRUE(scenario.motions.empty());
  if (scenario.fail_at == "Find Red Block:right")
  {
    EXPECT_EQ(scenario.calls["Find Green Block:left"], 0);
  }

  // A later successful empty detection must not retain this failure.
  scenario.fail_at.clear();
  scenario.red.clear();
  scenario.green.clear();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
}

INSTANTIATE_TEST_SUITE_P(BothColors, SortBlocksPerceptionFailure,
                         ::testing::Values("Find Red Block:right", "Find Green Block:left"),
                         [](const ::testing::TestParamInfo<std::string>& info) {
                           return info.param == "Find Red Block:right" ? "Red" : "Green";
                         });

TEST_F(SortBlocks, RegressionPoseTransformFailureCannotSucceedAsExhaustion)
{
  // GIVEN a detected red block whose pose cannot be transformed.
  scenario.red = { true };
  scenario.green = { true };
  scenario.fail_at = "TransformPose:right";
  auto tree = makeTree();

  // WHEN the pose conversion fails before grasp planning.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

  // THEN neither stale-pose motion nor the other color's branch is allowed.
  EXPECT_TRUE(scenario.motions.empty());
  EXPECT_EQ(scenario.calls["Find Green Block:left"], 0);
}

class SortBlocksPlanFailure : public SortBlocks, public ::testing::WithParamInterface<std::string>
{
};

TEST_P(SortBlocksPlanFailure, RegressionTransferMustPlanBeforeGraspExecution)
{
  // GIVEN one block and an infeasible continuation after its grasp.
  const std::string arm = GetParam();
  scenario.red = arm == "right" ? std::deque<bool>{ true } : std::deque<bool>{};
  scenario.green = { true };
  scenario.fail_at = "Plan Block Transfer:" + arm;
  auto tree = makeTree();

  // WHEN complete transfer planning fails.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

  // THEN the open gripper never approaches or closes on a block with no carry path.
  EXPECT_EQ(scenario.calls[scenario.fail_at], 1);
  EXPECT_EQ(scenario.calls["Execute MTC Solution:" + arm], 0);
  EXPECT_EQ(scenario.calls["Close Right Gripper:right"], 0);
  EXPECT_EQ(scenario.calls["Close Left Gripper:left"], 0);
  if (arm == "right")
  {
    EXPECT_EQ(scenario.calls["Find Green Block:left"], 0);
  }
}

INSTANTIATE_TEST_SUITE_P(BothArms, SortBlocksPlanFailure, ::testing::Values("right", "left"),
                         [](const ::testing::TestParamInfo<std::string>& info) { return info.param; });

TEST_F(SortBlocks, RegressionGrippersCloseBetweenPickupAndCarrySolutions)
{
  // GIVEN one block for each arm and distinguishable pickup/carry solutions from the planner.
  scenario.red = { true };
  scenario.green = { true };
  auto tree = makeTree();

  // WHEN both transfers complete through the shipped SubTree port mappings.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);

  // THEN each hand closes only after pickup and opens only after its own carry, not the other arm's solution.
  EXPECT_EQ(scenario.transfer_steps,
            (std::vector<std::string>{ "Open Right Gripper:right", "Plan Block Transfer:right", "right pickup",
                                       "Close Right Gripper:right", "right carry", "Open Right Gripper:right",
                                       "Open Left Gripper:left", "Plan Block Transfer:left", "left pickup",
                                       "Close Left Gripper:left", "left carry", "Open Left Gripper:left" }));
}

class SortBlocksCancellation : public SortBlocks, public ::testing::WithParamInterface<std::pair<std::string, int>>
{
};

TEST_P(SortBlocksCancellation, CancellationHaltsPlanningOrMotionWithoutClosingOrContinuing)
{
  // GIVEN an active complete-transfer plan, pickup, or carry.
  scenario.red = { true };
  scenario.hold_at = GetParam().first;
  scenario.hold_on_call = GetParam().second;
  auto tree = makeTree();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  const auto previous_motions = scenario.motions;

  // WHEN the Objective is cancelled.
  tree.haltTree();

  // THEN the active operation is halted, no later gripper or motion runs, and a later run starts cleanly.
  EXPECT_EQ(scenario.halts, 1);
  EXPECT_EQ(scenario.motions, previous_motions);
  scenario.hold_at.clear();
  scenario.red.clear();
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
}

INSTANTIATE_TEST_SUITE_P(TransferStages, SortBlocksCancellation,
                         ::testing::Values(std::make_pair("Plan Block Transfer:right", 1),
                                           std::make_pair("Execute MTC Solution:right", 1),
                                           std::make_pair("Execute MTC Solution:right", 2)),
                         [](const ::testing::TestParamInfo<std::pair<std::string, int>>& info) {
                           return info.param.first == "Plan Block Transfer:right" ? "Plan" :
                                  info.param.second == 1                          ? "Pickup" :
                                                                                    "Carry";
                         });
}  // namespace
