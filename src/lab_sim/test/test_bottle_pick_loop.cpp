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

#include <cstdlib>
#include <deque>
#include <string>
#include <vector>

namespace
{
struct Scenario
{
  std::deque<int> detections;
  std::string fail_at;
  std::string hold_at;
  std::vector<std::string> errors;
  int detection_calls = 0;
  int picks = 0;
  int placements = 0;
  int halts = 0;
};

class Operation : public BT::StatefulActionNode
{
public:
  Operation(const std::string& name, const BT::NodeConfig& config, Scenario* scenario)
    : BT::StatefulActionNode(name, config), scenario_(*scenario)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("operation") };
  }

private:
  BT::NodeStatus onStart() override
  {
    const auto operation = getInput<std::string>("operation").value();
    if (operation == scenario_.fail_at)
    {
      scenario_.errors.push_back(operation);
      return BT::NodeStatus::FAILURE;
    }
    if (operation == scenario_.hold_at)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (operation == "Detect AprilTags on Table" || operation == "Pick April Tag Labeled Object")
    {
      ++scenario_.detection_calls;
      if (scenario_.detections.empty())
      {
        scenario_.errors.push_back("Detection attempted after exhaustion");
        return BT::NodeStatus::FAILURE;
      }
      const int count = scenario_.detections.front();
      scenario_.detections.pop_front();
      config().blackboard->set("detection_count", count);
      // The standalone pick contract requires a detection, including in the original loop.
      if (operation == "Pick April Tag Labeled Object" && count == 0)
      {
        scenario_.errors.push_back("Empty detection passed to pick");
        return BT::NodeStatus::FAILURE;
      }
    }
    if (operation == "Pick from AprilTag Detections" || operation == "Pick April Tag Labeled Object")
    {
      ++scenario_.picks;
      EXPECT_GT(config().blackboard->get<int>("detection_count"), 0);
    }
    if (operation == "RemoveFromVector")
    {
      ++scenario_.placements;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    ++scenario_.halts;
  }

  Scenario& scenario_;
};

// Replace sensor and motion boundaries while retaining the Objective's actual control flow and scripts.
void substituteOperations(tinyxml2::XMLElement* element)
{
  const std::string tag = element->Name();
  const std::string id = element->Attribute("ID") ? element->Attribute("ID") : "";
  if (tag == "SubTree" || (tag == "Action" && id != "Script" && id != "AlwaysSuccess"))
  {
    while (element->FirstAttribute())
    {
      element->DeleteAttribute(element->FirstAttribute()->Name());
    }
    element->SetName("Action");
    element->SetAttribute("ID", "Operation");
    element->SetAttribute("operation", id.c_str());
  }
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    substituteOperations(child);
  }
}

class BottlePickLoop : public ::testing::Test
{
public:
  BT::Tree makeTree()
  {
    tinyxml2::XMLDocument source;
    const char* objective_xml = std::getenv("BOTTLE_OBJECTIVE_XML");
    if (source.LoadFile(objective_xml ? objective_xml : BOTTLE_OBJECTIVE_XML) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("Cannot load bottle Objective XML");
    }
    auto* sequence = source.RootElement()->FirstChildElement("BehaviorTree")->FirstChildElement();
    tinyxml2::XMLDocument fixture;
    fixture.Parse("<root BTCPP_format='4'><BehaviorTree ID='Bottle loop'><Sequence/></BehaviorTree></root>");
    auto* fixture_sequence = fixture.RootElement()->FirstChildElement()->FirstChildElement();
    // Setup is outside the loop contract. Retain its completion-state initializer when present.
    for (auto* child = sequence->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      if (child->NextSiblingElement() == nullptr ||
          (child->Attribute("ID") && std::string(child->Attribute("ID")) == "Script"))
      {
        fixture_sequence->InsertEndChild(child->DeepClone(&fixture));
      }
    }
    substituteOperations(fixture_sequence);
    tinyxml2::XMLPrinter printer;
    fixture.Print(&printer);
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

TEST_F(BottlePickLoop, EmptyTableSucceedsWithoutPickingOrErrors)
{
  // GIVEN an empty detection result.
  scenario.detections = { 0 };
  auto tree = makeTree();
  // WHEN the loop finishes, THEN it succeeds without attempting a pick.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.detection_calls, 1);
  EXPECT_EQ(scenario.picks, 0);
  EXPECT_EQ(scenario.placements, 0);
  EXPECT_TRUE(scenario.errors.empty());
}

TEST_F(BottlePickLoop, FiveBottlesArePlacedThenDetectionExhaustionSucceeds)
{
  // GIVEN five bottles followed by an empty table.
  scenario.detections = { 5, 4, 3, 2, 1, 0 };
  auto tree = makeTree();
  // WHEN every bottle is placed, THEN the next detection ends the loop without an extra pick.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.detection_calls, 6);
  EXPECT_EQ(scenario.picks, 5);
  EXPECT_EQ(scenario.placements, 5);
  EXPECT_TRUE(scenario.errors.empty());
}

class BottlePickFailure : public BottlePickLoop, public ::testing::WithParamInterface<std::string>
{
};

TEST_P(BottlePickFailure, OperationalFailurePropagates)
{
  // GIVEN a failure in a sensor or motion operation.
  scenario.detections = { 1, 0 };
  scenario.fail_at = GetParam();
  auto tree = makeTree();
  // WHEN that operation fails, THEN the Objective fails and retains the diagnostic.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  ASSERT_EQ(scenario.errors.size(), 1u);
  EXPECT_EQ(scenario.errors.front(), GetParam());
  EXPECT_EQ(scenario.placements, 0);
}

INSTANTIATE_TEST_SUITE_P(DetectionAndMotion, BottlePickFailure,
                         ::testing::Values("Detect AprilTags on Table", "Pick from AprilTag Detections",
                                           "PlanCartesianPath", "ExecuteTrajectory", "Execute MTC Solution"),
                         [](const ::testing::TestParamInfo<std::string>& info) {
                           std::string name;
                           for (const char character : info.param)
                           {
                             if (character != ' ')
                             {
                               name += character;
                             }
                           }
                           return name;
                         });

TEST_F(BottlePickLoop, HaltStopsRunningMotionAndRestartUsesFreshDetection)
{
  // GIVEN an executing trajectory after successful detection.
  scenario.detections = { 1 };
  scenario.hold_at = "ExecuteTrajectory";
  auto tree = makeTree();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // WHEN the Objective is halted, THEN motion is halted and a new run detects the table again.
  tree.haltTree();
  EXPECT_EQ(scenario.halts, 1);
  EXPECT_EQ(scenario.placements, 0);
  scenario.hold_at.clear();
  scenario.detections = { 0 };
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.detection_calls, 2);
  EXPECT_EQ(scenario.picks, 1);
  EXPECT_TRUE(scenario.errors.empty());
}
}  // namespace
