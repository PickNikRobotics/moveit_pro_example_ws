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
#include <rebot_bench_behaviors/motion_scales.hpp>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace
{
// Every Objective in this package is loaded with its TreeNodesModel, so SubTree
// port contracts are enforced as in the Objective Server. Leaf Actions and
// SubTrees from other packages become recording stubs; the production control
// flow, scripts and port remapping run unchanged. LoadMotionScales reads the
// shipped settings file with the Behavior's own parser.
const std::vector<std::string> kRecordedPorts{ "waypoint_name", "joint_group_name", "planning_group_name",
                                               "velocity_scale_factor", "acceleration_scale_factor" };
const std::set<std::string> kOwnStubs{ "LoadMotionScales", "IsJointAtLeast" };

// The speeds the deployed Stack Blue on Red (1cc5af8a) ran at on the real arm.
// The gripper's is the opening that worked there; a faster one was canceled.
const std::map<std::string, double> kArmProven{ { "joint", 0.25 }, { "cartesian", 0.15 }, { "gripper", 0.1 } };

struct Call
{
  std::string label;
  std::string kind;
  rebot_bench_behaviors::MotionScales scales{ 0.0, 0.0 };
};

void substituteOperations(tinyxml2::XMLElement* element, const std::set<std::string>& local_trees)
{
  const std::string tag = element->Name();
  const std::string id = element->Attribute("ID") ? element->Attribute("ID") : "";
  if ((tag == "Action" && !kOwnStubs.contains(id)) || (tag == "SubTree" && !local_trees.contains(id)))
  {
    std::string operation = id;
    if (element->Attribute("gripper_state"))
      operation += " holding measured jaw";
    std::vector<std::pair<std::string, std::string>> kept;
    for (const auto& port : kRecordedPorts)
      if (const char* value = element->Attribute(port.c_str()))
        kept.emplace_back(port, value);
    while (element->FirstAttribute())
      element->DeleteAttribute(element->FirstAttribute()->Name());
    element->SetName("Operation");
    element->SetAttribute("operation", operation.c_str());
    for (const auto& [port, value] : kept)
      element->SetAttribute(port.c_str(), value.c_str());
  }
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    substituteOperations(child, local_trees);
}

class StackObjectives : public testing::Test
{
protected:
  void SetUp() override
  {
    std::deque<tinyxml2::XMLDocument> documents;
    std::set<std::string> local_trees;
    for (const auto& entry : std::filesystem::directory_iterator(PACKAGE_DIR "/objectives"))
    {
      if (entry.path().extension() != ".xml")
        continue;
      auto& document = documents.emplace_back();
      ASSERT_EQ(document.LoadFile(entry.path().c_str()), tinyxml2::XML_SUCCESS) << entry.path();
      for (auto* tree = document.RootElement()->FirstChildElement("BehaviorTree"); tree;
           tree = tree->NextSiblingElement("BehaviorTree"))
        local_trees.insert(tree->Attribute("ID"));
    }
    factory.registerSimpleAction(
        "Operation",
        [this](BT::TreeNode& node) {
          Call call{ node.getInput<std::string>("operation").value() };
          if (const auto waypoint = node.getInput<std::string>("waypoint_name"))
            call.label += " " + waypoint.value();
          const auto group = node.getInput<std::string>("joint_group_name").value_or("");
          const auto planning_group = node.getInput<std::string>("planning_group_name").value_or("");
          if (call.label.starts_with("Move to Waypoint"))
            call.kind = group == "manipulator" ? "joint" : group;
          else if (call.label == "PlanToJointGoal" && planning_group == "manipulator")
            call.kind = "joint";
          else if (call.label == "PlanCartesianPath" && planning_group == "manipulator")
            call.kind = "cartesian";
          call.scales = { node.getInput<double>("velocity_scale_factor").value_or(0.0),
                          node.getInput<double>("acceleration_scale_factor").value_or(0.0) };
          calls.push_back(call);
          return call.label == failing ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("operation"), BT::InputPort<std::string>("waypoint_name"),
          BT::InputPort<std::string>("joint_group_name"), BT::InputPort<std::string>("planning_group_name"),
          BT::InputPort<double>("velocity_scale_factor"), BT::InputPort<double>("acceleration_scale_factor") });
    factory.registerSimpleAction(
        "LoadMotionScales",
        [this](BT::TreeNode& node) {
          const auto kind = node.getInput<std::string>("leg_kind").value();
          EXPECT_EQ(node.getInput<std::string>("package_name").value(), "rebot_bench_base_config");
          const auto scales = settings.empty() ?
                                  rebot_bench_behaviors::readMotionScales(
                                      PACKAGE_DIR "/" + node.getInput<std::string>("file_path").value(), kind) :
                                  settings.at(kind);
          node.setOutput("velocity_scale_factor", scales.velocity);
          node.setOutput("acceleration_scale_factor", scales.acceleration);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("package_name"), BT::InputPort<std::string>("file_path"),
          BT::InputPort<std::string>("leg_kind"), BT::OutputPort<double>("velocity_scale_factor"),
          BT::OutputPort<double>("acceleration_scale_factor") });
    // The measured joint7 after each opening, oldest first; fully open when unset.
    factory.registerSimpleAction("IsJointAtLeast",
                                 [this](BT::TreeNode& node) {
                                   EXPECT_EQ(node.getInput<std::string>("joint_name").value(), "joint7");
                                   const double measured = measured_jaw.empty() ? 5.25 : measured_jaw.front();
                                   if (!measured_jaw.empty())
                                     measured_jaw.pop_front();
                                   const bool open = measured >= node.getInput<double>("minimum").value();
                                   calls.push_back({ open ? "jaw open" : "jaw short" });
                                   return open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
                                 },
                                 { BT::InputPort("joint_state"), BT::InputPort<std::string>("joint_name"),
                                   BT::InputPort<double>("minimum") });
    for (auto& document : documents)
    {
      for (auto* tree = document.RootElement()->FirstChildElement("BehaviorTree"); tree;
           tree = tree->NextSiblingElement("BehaviorTree"))
        substituteOperations(tree, local_trees);
      tinyxml2::XMLPrinter printer;
      document.Print(&printer);
      factory.registerBehaviorTreeFromText(printer.CStr());
    }
  }

  BT::NodeStatus run(const std::string& subtree_call)
  {
    auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + subtree_call +
                                           "</BehaviorTree></root>");
    return tree.tickWhileRunning();
  }

  // Motions and the decisions between them, in order.
  std::vector<std::string> motions() const
  {
    std::vector<std::string> result;
    for (const auto& call : calls)
      if (call.label.starts_with("Move to Waypoint") || call.label.starts_with("HoldGripperInTrajectory") ||
          call.label.starts_with("jaw ") || call.label == "DetachObject" || call.label == "ExecuteTrajectory")
        result.push_back(call.label);
    return result;
  }

  std::vector<Call> legs() const
  {
    std::vector<Call> result;
    std::ranges::copy_if(calls, std::back_inserter(result), [](const Call& call) { return !call.kind.empty(); });
    return result;
  }

  void expectLegsCarry(const std::map<std::string, rebot_bench_behaviors::MotionScales>& expected) const
  {
    std::set<std::string> kinds;
    for (const auto& leg : legs())
    {
      SCOPED_TRACE(leg.label);
      kinds.insert(leg.kind);
      EXPECT_DOUBLE_EQ(leg.scales.velocity, expected.at(leg.kind).velocity);
      EXPECT_DOUBLE_EQ(leg.scales.acceleration, expected.at(leg.kind).acceleration);
    }
    EXPECT_EQ(kinds, (std::set<std::string>{ "joint", "cartesian", "gripper" }));
  }

  BT::BehaviorTreeFactory factory;
  std::vector<Call> calls;
  std::string failing;
  std::map<std::string, rebot_bench_behaviors::MotionScales> settings;
  std::deque<double> measured_jaw;
};

const std::map<std::string, rebot_bench_behaviors::MotionScales> kDistinct{ { "joint", { 0.21, 0.22 } },
                                                                            { "cartesian", { 0.11, 0.12 } },
                                                                            { "gripper", { 0.05, 0.06 } } };
const std::string kRelease = R"(<SubTree ID="reBot Release and Retreat" place_hover="{place_hover}" )"
                             R"(joint_velocity_scale_factor="0.21" joint_acceleration_scale_factor="0.22" )"
                             R"(cartesian_velocity_scale_factor="0.11" cartesian_acceleration_scale_factor="0.12" )"
                             R"(gripper_velocity_scale_factor="0.05" gripper_acceleration_scale_factor="0.06" />)";
const std::string kPick = R"(<SubTree ID="Pick Blue Cube" red_top="{red_top}" />)";
const std::string kStack = R"(<SubTree ID="Stack Blue on Red" />)";
const std::string kOpen = "Move to Waypoint Stack Gripper Open";
const std::string kSurvey = "Move to Waypoint Pick and Stack Survey";
const std::string kHome = "Move to Waypoint Zeroed Rest";
const std::vector<std::string> kReleased{
  kOpen, "jaw open", "DetachObject", "HoldGripperInTrajectory", "ExecuteTrajectory", kHome
};
const std::vector<std::string> kRecovered{ "DetachObject", "HoldGripperInTrajectory holding measured jaw",
                                           "ExecuteTrajectory", kHome };

std::vector<std::string> concat(std::vector<std::string> head, const std::vector<std::string>& tail)
{
  head.insert(head.end(), tail.begin(), tail.end());
  return head;
}

TEST_F(StackObjectives, ReleaseRetreatsHomeAndSucceeds)
{
  EXPECT_EQ(run(kRelease), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(motions(), kReleased);
  expectLegsCarry(kDistinct);
}

TEST_F(StackObjectives, ShortReleaseRetriesOnceThenRetreatsWithMeasuredJawGoesHomeAndFails)
{
  // A canceled opening reports success with the jaw still on the cube.
  measured_jaw = { 2.8, 2.8 };
  EXPECT_EQ(run(kRelease), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), concat({ kOpen, "jaw short", kOpen, "jaw short" }, kRecovered));
  expectLegsCarry(kDistinct);
}

TEST_F(StackObjectives, ShortReleaseOpenedOnRetryRetreatsHomeAndSucceeds)
{
  measured_jaw = { 2.8, 5.2 };
  EXPECT_EQ(run(kRelease), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(motions(), concat({ kOpen, "jaw short" }, kReleased));
}

TEST_F(StackObjectives, OpenFailureRetreatsWithMeasuredJawGoesHomeAndFails)
{
  failing = kOpen;
  EXPECT_EQ(run(kRelease), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), concat({ kOpen, kOpen }, kRecovered));
  expectLegsCarry(kDistinct);
}

TEST_F(StackObjectives, DetachFailureStillGoesHomeAndFails)
{
  failing = "DetachObject";
  EXPECT_EQ(run(kRelease), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), kReleased);
}

TEST_F(StackObjectives, RejectedRetreatStopsAndFails)
{
  failing = "ExecuteTrajectory";
  EXPECT_EQ(run(kRelease), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kOpen, "jaw open", "DetachObject", "HoldGripperInTrajectory",
                                                  "ExecuteTrajectory" }));
}

TEST_F(StackObjectives, StackOpensTheJawAtHomeFirstAndEndsHome)
{
  EXPECT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 3U);
  EXPECT_EQ(std::vector<std::string>(sequence.begin(), sequence.begin() + 3),
            (std::vector<std::string>{ kOpen, "jaw open", kSurvey }));
  EXPECT_EQ(sequence.back(), kHome);
}

TEST_F(StackObjectives, ShortHomeOpeningIsRetriedOnceBeforeTheSurvey)
{
  measured_jaw = { 0.085, 5.2 };
  EXPECT_EQ(run(kPick), BT::NodeStatus::SUCCESS);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 5U);
  EXPECT_EQ(std::vector<std::string>(sequence.begin(), sequence.begin() + 5),
            (std::vector<std::string>{ kOpen, "jaw short", kOpen, "jaw open", kSurvey }));
}

TEST_F(StackObjectives, TwoShortHomeOpeningsFailWithoutSurveying)
{
  measured_jaw = { 0.085, 0.085 };
  EXPECT_EQ(run(kPick), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kOpen, "jaw short", kOpen, "jaw short" }));
}

TEST_F(StackObjectives, OpeningMarginallyShortOfTheControllerStartToleranceFails)
{
  // Arm legs hold the 5.25 rad open command; the controller rejects a start more than 0.15 rad away.
  measured_jaw = { 5.05, 5.05 };
  EXPECT_EQ(run(kPick), BT::NodeStatus::FAILURE);
}

TEST_F(StackObjectives, PreparationRetriesShortOpeningAndConfirmsTheJaw)
{
  measured_jaw = { 2.8, 5.2 };
  EXPECT_EQ(run(R"(<SubTree ID="Prepare reBot Stack" blue_top="{blue_top}" red_top="{red_top}" />)"),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kOpen, "jaw short", kOpen, "jaw open" }));
}

TEST_F(StackObjectives, PreparationFailsAfterTwoShortOpenings)
{
  measured_jaw = { 2.8, 2.8 };
  EXPECT_EQ(run(R"(<SubTree ID="Prepare reBot Stack" blue_top="{blue_top}" red_top="{red_top}" />)"),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kOpen, "jaw short", kOpen, "jaw short" }));
}

class ShippedSpeeds : public StackObjectives, public testing::WithParamInterface<std::string>
{
};

TEST_P(ShippedSpeeds, NoLegIsFasterThanOnTheArm)
{
  ASSERT_EQ(run(GetParam()), BT::NodeStatus::SUCCESS);
  std::map<std::string, rebot_bench_behaviors::MotionScales> first;
  for (const auto& leg : legs())
  {
    SCOPED_TRACE(leg.label);
    first.try_emplace(leg.kind, leg.scales);
    EXPECT_DOUBLE_EQ(leg.scales.velocity, first.at(leg.kind).velocity);
    EXPECT_DOUBLE_EQ(leg.scales.acceleration, first.at(leg.kind).acceleration);
    EXPECT_GT(leg.scales.velocity, 0.0);
    EXPECT_GT(leg.scales.acceleration, 0.0);
    EXPECT_LE(leg.scales.velocity, kArmProven.at(leg.kind));
    EXPECT_LE(leg.scales.acceleration, kArmProven.at(leg.kind));
  }
  EXPECT_EQ(first.size(), 3U);
}

INSTANTIATE_TEST_SUITE_P(StackEntryPoints, ShippedSpeeds, testing::Values(kStack, kPick));

TEST_F(StackObjectives, SettingsReachEveryLegOfTheirKindIncludingHome)
{
  settings = kDistinct;
  ASSERT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(std::ranges::any_of(legs(), [](const Call& leg) { return leg.label == kHome; }));
  expectLegsCarry(kDistinct);
}
}  // namespace
