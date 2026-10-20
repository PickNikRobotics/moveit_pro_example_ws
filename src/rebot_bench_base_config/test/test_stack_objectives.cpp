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
#include <ranges>
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
const std::set<std::string> kOwnStubs{ "LoadMotionScales", "IsJointAtLeast", "GetSizeOfVector" };

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
          const auto planning_group = node.getInput<std::string>("planning_group_name").value_or("");
          // Only Move reBot Gripper generates a point-to-point trajectory.
          if (call.label == "GeneratePointToPointTrajectory")
            call.kind = "gripper";
          else if (call.label == "PlanToJointGoal" && planning_group == "manipulator")
            call.kind = "joint";
          else if (call.label == "PlanCartesianPath" && planning_group == "manipulator")
            call.kind = "cartesian";
          call.scales = { node.getInput<double>("velocity_scale_factor").value_or(0.0),
                          node.getInput<double>("acceleration_scale_factor").value_or(0.0) };
          // A plan home from home starts at its goal: a single point, which ValidateTrajectory rejects.
          if (call.kind == "joint" || call.kind == "cartesian")
          {
            const auto waypoint = std::ranges::find_if(calls.rbegin(), calls.rend(), [](const Call& earlier) {
              return earlier.label.starts_with("RetrieveWaypoint");
            });
            const bool home =
                call.kind == "joint" && waypoint != calls.rend() && waypoint->label == "RetrieveWaypoint Zeroed Rest";
            single_point_plan = home && arm_home;
            arm_home = home;
          }
          calls.push_back(call);
          // ReportUnreachableCube always fails; it only words the failure.
          const bool fail =
              call.label == "ReportUnreachableCube" || (call.label == "ValidateTrajectory" && single_point_plan) ||
              (call.label == failing &&
               (failing_occurrence == 0 || std::ranges::count(calls, failing, &Call::label) == failing_occurrence));
          return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
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
    // Blue poses located within a cube width of red on the mat, one per locate; none when unset.
    factory.registerSimpleAction("GetSizeOfVector",
                                 [this](BT::TreeNode& node) {
                                   node.setOutput("vector_size", blue_near_red.empty() ? 0 : blue_near_red.front());
                                   if (!blue_near_red.empty())
                                     blue_near_red.pop_front();
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort("vector_in"), BT::OutputPort<int>("vector_size") });
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
      if (call.label.starts_with("RetrieveWaypoint") || call.label.starts_with("Move to Waypoint") ||
          call.label == "HoldGripperInTrajectory holding measured jaw" || call.label.starts_with("jaw ") ||
          call.label == "DetachObject" || call.label == "PlanCartesianPath" || call.label == "ReportUnreachableCube")
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
  // Which call of `failing` fails, counting from 1; 0 fails every one.
  long failing_occurrence = 0;
  std::map<std::string, rebot_bench_behaviors::MotionScales> settings;
  std::deque<double> measured_jaw;
  std::deque<int> blue_near_red;
  bool arm_home = false;
  bool single_point_plan = false;
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
const std::string kOpen = "RetrieveWaypoint Stack Gripper Open";
const std::string kSurvey = "RetrieveWaypoint Pick and Stack Survey";
const std::string kHome = "RetrieveWaypoint Zeroed Rest";
const std::string kClose = "Move to Waypoint Gripper Closed";
const std::vector<std::string> kReleased{ kOpen, "jaw open", "DetachObject", "PlanCartesianPath", kHome };
const std::vector<std::string> kRecovered{ "DetachObject", "PlanCartesianPath",
                                           "HoldGripperInTrajectory holding measured jaw", kHome };

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
  failing = "GeneratePointToPointTrajectory";
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
  // The opening executes first; the second execution is the retreat.
  failing = "ExecuteTrajectory";
  failing_occurrence = 2;
  EXPECT_EQ(run(kRelease), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kOpen, "jaw open", "DetachObject", "PlanCartesianPath" }));
}

TEST_F(StackObjectives, StackClosesThenOpensTheJawAtHomeFirstAndEndsHomeClosed)
{
  EXPECT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 4U);
  // The native close makes the commanded home true for the jaw before it opens.
  EXPECT_EQ(std::vector<std::string>(sequence.begin(), sequence.begin() + 4),
            (std::vector<std::string>{ kClose, kOpen, "jaw open", kSurvey }));
  // The arm goes home, then the empty jaw closes there.
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 2, sequence.end()), (std::vector<std::string>{ kHome, kClose }));
  ASSERT_FALSE(legs().empty());
  EXPECT_EQ(legs().back().kind, "joint");
}

TEST_F(StackObjectives, UnreachableCubeFailsWithoutAnyMotion)
{
  // The reach check solves the pick hover, the pick, the tilted place hover and the place, in that order.
  failing = "ComputeInverseKinematics";
  for (const auto& entry_point : { kStack, kPick })
    for (const long unreachable : { 1, 2, 4 })
    {
      SCOPED_TRACE(entry_point + " IK " + std::to_string(unreachable));
      calls.clear();
      failing_occurrence = unreachable;
      EXPECT_EQ(run(entry_point), BT::NodeStatus::FAILURE);
      EXPECT_EQ(motions(), std::vector<std::string>{ "ReportUnreachableCube" });
      EXPECT_TRUE(legs().empty());
      EXPECT_EQ(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
    }
}

TEST_F(StackObjectives, FailedLegBeforeTheGraspEndsHomeClosed)
{
  failing = "PlanCartesianPath";  // the descent to the pick
  failing_occurrence = 1;
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 3U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 3, sequence.end()),
            (std::vector<std::string>{ "PlanCartesianPath", kHome, kClose }));
}

TEST_F(StackObjectives, FailedLegWhileHoldingTheCubeEndsHomeWithoutClosingOnIt)
{
  failing = "PlanCartesianPath";  // the descent to the place, after the pick and lift
  failing_occurrence = 3;
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 2U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 2, sequence.end()),
            (std::vector<std::string>{ "PlanCartesianPath", kHome }));
}

TEST_F(StackObjectives, FailureAfterTheGraspClosesEndsHomeWithoutClosingOnTheCube)
{
  failing = "AttachObject";
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 2U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 2, sequence.end()),
            (std::vector<std::string>{ "RetrieveWaypoint Stack Gripper Hold", kHome }));
}

TEST_F(StackObjectives, FailedReleaseOpeningEndsHomeWithoutClosingOnTheCube)
{
  // The opening at home succeeds; both release openings stay on the cube.
  measured_jaw = { 5.25, 2.8, 2.8 };
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  const auto tail = concat(kRecovered, { kHome });
  ASSERT_GE(sequence.size(), tail.size());
  EXPECT_EQ(std::vector<std::string>(sequence.end() - static_cast<long>(tail.size()), sequence.end()), tail);
  EXPECT_EQ(std::ranges::count(sequence, kClose), 1);
}

TEST_F(StackObjectives, FailedCloseAtHomeClosesAgainThoughTheMoveHomeIsRefused)
{
  // The final close aborts with the arm home, so the recovery's one-point plan home is refused.
  failing = kClose;
  failing_occurrence = 2;
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 4U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 4, sequence.end()),
            (std::vector<std::string>{ kHome, kClose, kHome, kClose }));
}

TEST_F(StackObjectives, RecoveryClosesTheEmptyJawEvenIfTheMoveHomeFails)
{
  // Every validation fails: the first leg after the reach check, at home, and then the recovery's move home.
  failing = "ValidateTrajectory";
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 2U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 2, sequence.end()), (std::vector<std::string>{ kHome, kClose }));
}

TEST_F(StackObjectives, ShortHomeOpeningIsRetriedOnceBeforeTheSurvey)
{
  measured_jaw = { 0.085, 5.2 };
  EXPECT_EQ(run(kPick), BT::NodeStatus::SUCCESS);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 6U);
  EXPECT_EQ(std::vector<std::string>(sequence.begin(), sequence.begin() + 6),
            (std::vector<std::string>{ kClose, kOpen, "jaw short", kOpen, "jaw open", kSurvey }));
}

TEST_F(StackObjectives, TwoShortHomeOpeningsFailWithoutSurveying)
{
  measured_jaw = { 0.085, 0.085 };
  EXPECT_EQ(run(kPick), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kClose, kOpen, "jaw short", kOpen, "jaw short" }));
}

TEST_F(StackObjectives, ShortHomeOpeningsSendTheStackHomeAndCloseTheJaw)
{
  measured_jaw = { 0.085, 0.085 };
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ kClose, kOpen, "jaw short", kOpen, "jaw short", kHome, kClose }));
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

TEST_F(StackObjectives, CubesLocatedAsOneRelocateThenFailWithoutAnyMotion)
{
  blue_near_red = { 1, 1, 1 };
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(std::ranges::count(calls, "FitCubeToMask", &Call::label), 6);
  EXPECT_TRUE(motions().empty());
  EXPECT_EQ(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);

  calls.clear();
  blue_near_red = { 1 };
  EXPECT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  // Red and blue, twice.
  EXPECT_EQ(std::ranges::count(calls, "FitCubeToMask", &Call::label), 4);
}

class ShippedSpeeds : public StackObjectives, public testing::WithParamInterface<std::string>
{
};

TEST_P(ShippedSpeeds, EveryLegUsesTheShippedSettingOfItsKind)
{
  ASSERT_EQ(run(GetParam()), BT::NodeStatus::SUCCESS);
  std::map<std::string, rebot_bench_behaviors::MotionScales> shipped;
  for (const auto* kind : { "joint", "cartesian", "gripper" })
    shipped.emplace(kind,
                    rebot_bench_behaviors::readMotionScales(PACKAGE_DIR "/objectives/rebot_stack_motion.yaml", kind));
  expectLegsCarry(shipped);
}

INSTANTIATE_TEST_SUITE_P(StackEntryPoints, ShippedSpeeds, testing::Values(kStack, kPick));

TEST_F(StackObjectives, SettingsReachEveryLegOfTheirKindIncludingHome)
{
  settings = kDistinct;
  ASSERT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  const auto home = std::ranges::find(calls, kHome, &Call::label);
  ASSERT_NE(home, calls.end());
  const auto home_leg = std::find_if(home, calls.end(), [](const Call& call) { return !call.kind.empty(); });
  ASSERT_NE(home_leg, calls.end());
  EXPECT_EQ(home_leg->kind, "joint");
  expectLegsCarry(kDistinct);
}
}  // namespace
