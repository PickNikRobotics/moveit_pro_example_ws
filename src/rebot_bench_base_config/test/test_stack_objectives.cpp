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
#include <limits>
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
const std::vector<std::string> kRecordedPorts{ "waypoint_name",
                                               "joint_group_name",
                                               "planning_group_name",
                                               "velocity_scale_factor",
                                               "acceleration_scale_factor",
                                               "name_b",
                                               "allow_collision",
                                               "task",
                                               "message",
                                               "cube",
                                               "max_offset",
                                               "min_height" };
const std::set<std::string> kOwnStubs{ "LoadMotionScales", "IsJointAtLeast", "GetSizeOfVector" };

struct Call
{
  std::string label;
  std::string kind;
  rebot_bench_behaviors::MotionScales scales{ 0.0, 0.0 };
  std::string cube;
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
          for (const auto* port : { "waypoint_name", "name_b", "allow_collision", "task", "message" })
            if (const auto value = node.getInput<std::string>(port))
              call.label += " " + value.value();
          const auto planning_group = node.getInput<std::string>("planning_group_name").value_or("");
          // Move reBot Gripper generates the gripper's point-to-point trajectories; the arm legs generate one
          // from the command when the planner refuses the measured start.
          if (call.label == "GeneratePointToPointTrajectory")
            call.kind = planning_group == "gripper" ? "gripper" : "joint";
          else if (call.label == "PlanToJointGoal" && planning_group == "manipulator")
            call.kind = "joint";
          else if (call.label == "PlanCartesianPath" && planning_group == "manipulator")
            call.kind = "cartesian";
          call.scales = { node.getInput<double>("velocity_scale_factor").value_or(0.0),
                          node.getInput<double>("acceleration_scale_factor").value_or(0.0) };
          call.cube = node.getInput<std::string>("cube").value_or("");
          calls.push_back(call);
          const auto count = std::ranges::count(calls, failing, &Call::label);
          const bool refused =
              call.label == failing &&
              (failing_occurrence == 0 || (count >= failing_occurrence && count < failing_occurrence + failing_count));
          // A plan home from home starts at its goal: a single point, which ValidateTrajectory rejects. A refused
          // plan moves nothing; the leg's fallback, if any, is the plan.
          if ((call.kind == "joint" || call.kind == "cartesian") && !refused)
          {
            const auto waypoint = std::ranges::find_if(calls.rbegin(), calls.rend(), [](const Call& earlier) {
              return earlier.label.starts_with("RetrieveWaypoint");
            });
            const bool home =
                call.kind == "joint" && waypoint != calls.rend() && waypoint->label == "RetrieveWaypoint Zeroed Rest";
            single_point_plan = home && arm_home;
            arm_home = home;
          }
          // ReportUnreachableCube always fails; it only words the failure.
          // The unstack postcondition compares blue to itself to check the same depth sample against
          // both height bounds. Other IsCubeStacked calls retain their existing success/failure stub.
          const bool depth_below_bound = call.label == "IsCubeStacked" &&
                                         node.getInput<double>("max_offset").value_or(-1.0) == 0.0 &&
                                         !(unstacked_blue_height >= node.getInput<double>("min_height").value());
          const bool fail = call.label == "ReportUnreachableCube" || depth_below_bound ||
                            (call.label == "ValidateTrajectory" && single_point_plan) || refused;
          return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("operation"), BT::InputPort<std::string>("waypoint_name"),
          BT::InputPort<std::string>("name_b"), BT::InputPort<std::string>("allow_collision"),
          BT::InputPort<std::string>("task"), BT::InputPort<std::string>("message"), BT::InputPort<std::string>("cube"),
          BT::InputPort<std::string>("joint_group_name"), BT::InputPort<std::string>("planning_group_name"),
          BT::InputPort<double>("velocity_scale_factor"), BT::InputPort<double>("acceleration_scale_factor"),
          BT::InputPort<double>("max_offset"), BT::InputPort<double>("min_height") });
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
  // Which call of `failing` fails, counting from 1; 0 fails every one. failing_count calls fail from there.
  long failing_occurrence = 0;
  long failing_count = 1;
  std::map<std::string, rebot_bench_behaviors::MotionScales> settings;
  std::deque<double> measured_jaw;
  std::deque<int> blue_near_red;
  double unstacked_blue_height = 0.05;
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
// Recover Home detaches the cube from the planning scene once home.
const std::string kDetach = "DetachObject";
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

TEST_F(StackObjectives, PickOrPlaceTooFarForTheHeatBudgetFailsWithoutAnyMotion)
{
  // Checked after the pick hover, the pick and the place on red, in that order.
  failing = "IsPlannedJointAtMost";
  for (const long refused : { 2, 3 })
  {
    SCOPED_TRACE(refused);
    calls.clear();
    failing_occurrence = refused;
    EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
    EXPECT_EQ(motions(), std::vector<std::string>{ "ReportUnreachableCube" });
    EXPECT_EQ(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
  }
}

TEST_F(StackObjectives, FailedLegBeforeTheGraspEndsHomeClosed)
{
  failing = "PlanCartesianPath";  // the descent to the pick
  failing_occurrence = 1;
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 4U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 4, sequence.end()),
            (std::vector<std::string>{ "PlanCartesianPath", kHome, kClose, kDetach }));
}

TEST_F(StackObjectives, FailedLegWhileHoldingTheCubeEndsHomeWithoutClosingOnIt)
{
  failing = "PlanCartesianPath";  // the descent to the place, after the pick and lift
  failing_occurrence = 3;
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 3U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 3, sequence.end()),
            (std::vector<std::string>{ "PlanCartesianPath", kHome, kDetach }));
}

TEST_F(StackObjectives, FailureAfterTheGraspClosesEndsHomeWithoutClosingOnTheCube)
{
  failing = "AttachObject";
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 3U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 3, sequence.end()),
            (std::vector<std::string>{ "RetrieveWaypoint Stack Gripper Hold", kHome, kDetach }));
}

TEST_F(StackObjectives, FailedReleaseOpeningEndsHomeWithoutClosingOnTheCube)
{
  // The opening at home succeeds; both release openings stay on the cube.
  measured_jaw = { 5.25, 2.8, 2.8 };
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  const auto tail = concat(kRecovered, { kHome, kDetach });
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
  ASSERT_GE(sequence.size(), 5U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 5, sequence.end()),
            (std::vector<std::string>{ kHome, kClose, kHome, kClose, kDetach }));
}

TEST_F(StackObjectives, RecoveryClosesTheEmptyJawEvenIfTheMoveHomeFails)
{
  // Every validation fails: the first leg after the reach check, at home, and then the recovery's move home.
  failing = "ValidateTrajectory";
  EXPECT_EQ(run(kStack), BT::NodeStatus::FAILURE);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 3U);
  EXPECT_EQ(std::vector<std::string>(sequence.end() - 3, sequence.end()),
            (std::vector<std::string>{ kHome, kClose, kDetach }));
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
  EXPECT_EQ(motions(),
            (std::vector<std::string>{ kClose, kOpen, "jaw short", kOpen, "jaw short", kHome, kClose, kDetach }));
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

const std::string kUnstack = R"(<SubTree ID="Unstack Blue from Red" />)";
const std::string kCycle = R"(<SubTree ID="Stack and Unstack Cycle" num_cycles="2" />)";
const std::string kStackTask = "RecordEpisode stack the blue on the red";
const std::string kUnstackTask = "RecordEpisode unstack the blue from the red";

std::vector<std::string> labelsStartingWith(const std::vector<Call>& calls, const std::vector<std::string>& prefixes)
{
  std::vector<std::string> result;
  for (const auto& call : calls)
    if (std::ranges::any_of(prefixes, [&](const auto& prefix) { return call.label.starts_with(prefix); }))
      result.push_back(call.label);
  return result;
}

std::vector<std::string> tail(const std::vector<std::string>& sequence, std::size_t count)
{
  return std::vector<std::string>(sequence.end() - std::min(count, sequence.size()), sequence.end());
}

TEST_F(StackObjectives, UnstackLiftsBlueOffRedSetsItOnTheMatAndEndsHomeClosed)
{
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::SUCCESS);
  const auto sequence = motions();
  ASSERT_GE(sequence.size(), 4U);
  // The scene camera sees the cubes from home: no survey.
  EXPECT_EQ(std::vector<std::string>(sequence.begin(), sequence.begin() + 4),
            (std::vector<std::string>{ kClose, kOpen, "jaw open", "PlanCartesianPath" }));
  EXPECT_EQ(std::ranges::count(sequence, kSurvey), 0);
  EXPECT_EQ(tail(sequence, 2), (std::vector<std::string>{ kHome, kClose }));
  // The held cube may touch red only while it is lifted off, and the mat only from the descent on.
  EXPECT_EQ(tail(labelsStartingWith(calls, { "SetCollisionRule rebot_red", "SetCollisionRule rebot_bench" }), 3),
            (std::vector<std::string>{ "SetCollisionRule rebot_red true", "SetCollisionRule rebot_red false",
                                       "SetCollisionRule rebot_bench true" }));
}

TEST_F(StackObjectives, UnstackRejectsAFalseRgbStackBeforeSceneSetupOrMotion)
{
  failing = "IsCubeStacked";  // the aligned-depth check rejects the RGB top-face fit
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(motions().empty());
  EXPECT_EQ(std::ranges::count(calls, "AddCollisionBox", &Call::label), 0);
  EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 0);

  calls.clear();
  EXPECT_EQ(run(R"(<SubTree ID="Recover Cube from Wall" stacked="true" />)"), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(motions().empty());
}

TEST_F(StackObjectives, UnstackWithoutAlignedDepthRefusesBeforeMotion)
{
  failing = "GetPointCloud";
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(motions().empty());
}

TEST_F(StackObjectives, StackedWallRecoverySelectsRedSupportWithTheDefaultInputs)
{
  EXPECT_EQ(run(R"(<SubTree ID="Recover Cube from Wall" stacked="true" />)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "SetCollisionRule rebot_red true", &Call::label), 1);
  EXPECT_EQ(tail(labelsStartingWith(calls, { "SetCollisionRule rebot_red" }), 2),
            (std::vector<std::string>{ "SetCollisionRule rebot_red true", "SetCollisionRule rebot_red false" }));
}

TEST_F(StackObjectives, NonStackedWallRecoveryKeepsItsConfiguredSupport)
{
  EXPECT_EQ(run(R"(<SubTree ID="Recover Cube from Wall" support_object="custom_support" />)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "SetCollisionRule custom_support true", &Call::label), 1);
  EXPECT_EQ(std::ranges::count(calls, "SetCollisionRule custom_support false", &Call::label), 1);
}

TEST_F(StackObjectives, WallRecoveryReportsTheCarriedCubeWhenEveryApproachIsUnreachable)
{
  failing = "ComputeInverseKinematics";
  for (const auto& [entry_point, name] :
       { std::pair{ R"(<SubTree ID="Recover Cube from Wall" stacked="true" />)", "blue cube" },
         std::pair{ R"(<SubTree ID="Recover Cube from Wall" prompt="green cube" />)", "green cube" } })
  {
    SCOPED_TRACE(entry_point);
    calls.clear();
    EXPECT_EQ(run(entry_point), BT::NodeStatus::FAILURE);
    EXPECT_EQ(motions(), std::vector<std::string>{ "ReportUnreachableCube" });
    ASSERT_FALSE(calls.empty());
    EXPECT_EQ(calls.back().label, "ReportUnreachableCube");
    EXPECT_EQ(calls.back().cube, name);
    EXPECT_EQ(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
  }
}

TEST_F(StackObjectives, UnstackRejectsSourcePosesOutsideTheHeatReachBound)
{
  for (const int occurrence : { 1, 2, 3 })
  {
    SCOPED_TRACE(occurrence);
    calls.clear();
    failing = "IsPlannedJointAtMost";
    failing_occurrence = occurrence;
    // When the straight lift is refused, refuse the selected raised lift too.
    failing_count = occurrence == 3 ? 2 : 1;
    EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
    EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 0);
    EXPECT_EQ(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
  }
}

TEST_F(StackObjectives, UnstackLiftsToTheRaisedHoverWhereTheStraightLiftIsOutOfReach)
{
  failing = "ComputeInverseKinematics";
  failing_occurrence = 4;  // approach selection, heat reach, grasp, then the straight lift
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(motions(), "ReportUnreachableCube"), 0);
}

TEST_F(StackObjectives, UnstackRecoveryClosesTheEmptyJawEvenIfTheMoveHomeFails)
{
  // As when a gripper leg fails at home: the plan home has one point and is refused.
  failing = "ValidateTrajectory";
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tail(motions(), 3), (std::vector<std::string>{ kHome, kClose, kDetach }));
}

TEST_F(StackObjectives, StackTouchesTheMatOnlyWhileLiftingAndRedOnlyFromTheDescent)
{
  EXPECT_EQ(run(kStack), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tail(labelsStartingWith(calls, { "SetCollisionRule rebot_red", "SetCollisionRule rebot_bench" }), 3),
            (std::vector<std::string>{ "SetCollisionRule rebot_bench true", "SetCollisionRule rebot_bench false",
                                       "SetCollisionRule rebot_red true" }));
}

TEST_F(StackObjectives, UnreachableStackedCubeFailsWithoutMoving)
{
  failing = "ComputeInverseKinematics";
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(motions(), (std::vector<std::string>{ "ReportUnreachableCube" }));
  EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 0);
}

TEST_F(StackObjectives, UnreachableSpotIsRedrawnBeforeAnyMotion)
{
  // Four source checks (approach selection, heat reach, grasp, lift), then for the spot: the next stack's pick hover
  // and pick, then the spot's own place hover, travel hover and release pose.
  failing = "ComputeInverseKinematics";
  failing_occurrence = 9;
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 2);
  const auto redraw =
      std::find_if(calls.rbegin(), calls.rend(), [](const Call& call) { return call.label == "DrawPlaceSpot"; });
  const auto first_motion = std::ranges::find(calls, kClose, &Call::label);
  EXPECT_LT(std::distance(calls.begin(), redraw.base()), std::distance(calls.begin(), first_motion));
}

TEST_F(StackObjectives, SpotsThatStayUnreachableFailWithOneSpotMessageWithoutMoving)
{
  failing = "ComputeInverseKinematics";
  EXPECT_EQ(run(R"(<SubTree ID="reBot Draw Place Spot" red_top="{red_top}" spot="{spot}" travel="{travel}" />)"),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 20);
  // A rejected draw names no cube: none is out of reach, only the drawn spots are.
  EXPECT_TRUE(motions().empty());
  EXPECT_EQ(labelsStartingWith(calls, { "LogMessage" }),
            std::vector<std::string>{ "LogMessage No reachable free spot for blue after 20 draws." });
}

TEST_F(StackObjectives, FailedUnstackBeforeTheGraspEndsHomeClosed)
{
  failing = "PlanCartesianPath";  // the descent to the grasp
  failing_occurrence = 1;
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tail(motions(), 4), (std::vector<std::string>{ "PlanCartesianPath", kHome, kClose, kDetach }));
}

TEST_F(StackObjectives, FailedUnstackWhileHoldingTheCubeEndsHomeWithoutClosingOnIt)
{
  failing = "PlanCartesianPath";  // the lift, after the grasp
  failing_occurrence = 2;
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tail(motions(), 3), (std::vector<std::string>{ "PlanCartesianPath", kHome, kDetach }));
}

TEST_F(StackObjectives, PlansRefusedAtTheMeasuredStartMoveFromTheCommand)
{
  // The bench arm's sagged home fails the planner's start check, so every joint leg moves from the command.
  failing = "PlanToJointGoal";
  for (const auto& objective : { kStack, kUnstack })
  {
    SCOPED_TRACE(objective);
    calls.clear();
    EXPECT_EQ(run(objective), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(tail(motions(), 2), (std::vector<std::string>{ kHome, kClose }));
    std::size_t joint_legs = 0;
    for (const auto& leg : legs())
      if (leg.kind == "joint" && leg.label != "PlanToJointGoal")
      {
        ++joint_legs;
        EXPECT_EQ(leg.label, "GeneratePointToPointTrajectory");
      }
    EXPECT_GE(joint_legs, 2U);
  }
}

TEST_F(StackObjectives, CycleRecordsEachLegAsOneEpisodeWithItsPrompt)
{
  EXPECT_EQ(run(kCycle), BT::NodeStatus::SUCCESS);
  const auto one_cycle =
      concat({ kStackTask, "SaveEpisode", "StopRecording" }, { kUnstackTask, "SaveEpisode", "StopRecording" });
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            concat(one_cycle, one_cycle));
  // Each leg ends home with the jaw closed before the next starts.
  const std::vector<std::string> stack{ kStackTask, kClose, kHome, kClose };
  const std::vector<std::string> unstack{ kUnstackTask, kClose, kHome, kClose };
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", kHome, kClose }),
            concat(concat(stack, unstack), concat(stack, unstack)));
  // Each saved episode settles before its session closes.
  std::vector<std::string> after_save;
  for (std::size_t i = 0; i + 2 < calls.size(); ++i)
    if (calls[i].label == "SaveEpisode")
      after_save.push_back(calls[i + 1].label + ", " + calls[i + 2].label);
  EXPECT_EQ(after_save, std::vector<std::string>(4, "WaitForDuration, StopRecording"));
}

TEST_F(StackObjectives, LocateTriesAFreshFrameBeforeRefusing)
{
  failing = "FitCubeToMask";
  failing_occurrence = 1;
  failing_count = 2;
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "FitCubeToMask", &Call::label), 3);
}

TEST_F(StackObjectives, CycleRetriesARecorderStillWritingThePreviousEpisode)
{
  failing = kUnstackTask;
  failing_occurrence = 1;
  failing_count = 2;
  EXPECT_EQ(run(kCycle), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "SaveEpisode", &Call::label), 4);
}

TEST_F(StackObjectives, FailedRecorderStartupLeavesTheExistingSessionUntouched)
{
  for (const auto& [objective, task] :
       std::vector<std::pair<std::string, std::string>>{ { R"(<SubTree ID="Recorded Stack Leg" />)", kStackTask },
                                                         { R"(<SubTree ID="Recorded Unstack Leg" />)", kUnstackTask } })
  {
    SCOPED_TRACE(objective);
    calls.clear();
    failing = task;
    EXPECT_EQ(run(objective), BT::NodeStatus::FAILURE);
    EXPECT_EQ(std::ranges::count(calls, task, &Call::label), 10);
    EXPECT_EQ(std::ranges::count(calls, "StopRecording", &Call::label), 0);
    EXPECT_EQ(std::ranges::count(calls, "SaveEpisode", &Call::label), 0);
    EXPECT_TRUE(motions().empty());
  }
}

TEST_F(StackObjectives, CorrectedReleaseIsCheckedBeforeMotionAndRedrawnOnRefusal)
{
  // Four unstack source checks, two next-pick checks, the corrected
  // release hover and the carry hover, then the corrected release itself.
  failing = "ComputeInverseKinematics";
  failing_occurrence = 9;
  EXPECT_EQ(run(kUnstack), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::ranges::count(calls, "DrawPlaceSpot", &Call::label), 2);
  const auto first_motion = std::ranges::find(calls, kClose, &Call::label);
  EXPECT_EQ(std::count_if(calls.begin(), first_motion, [](const Call& call) { return call.label == "DrawPlaceSpot"; }),
            2);
}

TEST_F(StackObjectives, UnconfirmedStackDiscardsItsEpisodeAndStops)
{
  // Stack Blue on Red locates red and blue; the third fit is the stack check, tried on three frames.
  failing = "FitCubeToMask";
  failing_occurrence = 3;
  failing_count = 3;
  EXPECT_EQ(run(kCycle), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kStackTask, "StopRecording" }));
}

TEST_F(StackObjectives, BlueLocatedOffRedDiscardsTheStackEpisodeAndStops)
{
  // Blue fits a cube on a 50 mm support, but not over red as located before the stack, or not that high in depth.
  failing = "IsCubeStacked";
  EXPECT_EQ(run(kCycle), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kStackTask, "StopRecording" }));
}

TEST_F(StackObjectives, StackWithoutAlignedDepthDiscardsItsEpisodeAndStops)
{
  // With the scene camera's depth off, the stack cannot be confirmed.
  failing = "GetPointCloud";
  EXPECT_EQ(run(kCycle), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kStackTask, "StopRecording" }));
}

TEST_F(StackObjectives, FailedUnstackKeepsTheSavedStackDiscardsItselfAndStops)
{
  // The stack's four straight moves come first; the fifth is the unstack's descent to the grasp.
  failing = "PlanCartesianPath";
  failing_occurrence = 5;
  EXPECT_EQ(run(kCycle), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kStackTask, "SaveEpisode", "StopRecording", kUnstackTask, "StopRecording" }));
  EXPECT_EQ(tail(motions(), 4), (std::vector<std::string>{ "PlanCartesianPath", kHome, kClose, kDetach }));
  ASSERT_FALSE(calls.empty());
  EXPECT_EQ(calls.back().label, "StopRecording");
}

TEST_F(StackObjectives, UnstackDepthAtMatHeightAllowsTheEpisodeToBeSaved)
{
  for (const double height : { 0.025, 0.05, 0.074 })
  {
    SCOPED_TRACE(height);
    calls.clear();
    unstacked_blue_height = height;
    EXPECT_EQ(run(R"(<SubTree ID="Recorded Unstack Leg" />)"), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
              (std::vector<std::string>{ kUnstackTask, "SaveEpisode", "StopRecording" }));
  }
}

TEST_F(StackObjectives, MissedPhysicalUnstackGraspDiscardsItsEpisodeEvenWhenMotionAndRgbSucceed)
{
  unstacked_blue_height = 0.095;  // Blue never left red, despite successful motions and RGB fits.
  EXPECT_EQ(run(kCycle), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kStackTask, "SaveEpisode", "StopRecording", kUnstackTask, "StopRecording" }));
  EXPECT_GT(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
  EXPECT_EQ(std::ranges::count(calls, "ReportUnreachableCube", &Call::label), 0);
}

TEST_F(StackObjectives, MissingUnstackConfirmationDepthDiscardsItsEpisode)
{
  failing = "GetPointCloud";
  failing_occurrence = 2;  // The source stack was confirmed; the post-motion depth stream is missing.
  EXPECT_EQ(run(R"(<SubTree ID="Recorded Unstack Leg" />)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
            (std::vector<std::string>{ kUnstackTask, "StopRecording" }));
  EXPECT_GT(std::ranges::count(calls, "ExecuteTrajectory", &Call::label), 0);
}

TEST_F(StackObjectives, InvalidOrOutOfRangeUnstackDepthCannotPassTheInvertedUpperBound)
{
  for (const double height : { 0.0, 0.024, 0.075, 0.10, std::numeric_limits<double>::quiet_NaN() })
  {
    SCOPED_TRACE(height);
    calls.clear();
    unstacked_blue_height = height;
    EXPECT_EQ(run(R"(<SubTree ID="Recorded Unstack Leg" />)"), BT::NodeStatus::FAILURE);
    EXPECT_EQ(labelsStartingWith(calls, { "RecordEpisode", "SaveEpisode", "StopRecording" }),
              (std::vector<std::string>{ kUnstackTask, "StopRecording" }));
  }
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

INSTANTIATE_TEST_SUITE_P(StackEntryPoints, ShippedSpeeds, testing::Values(kStack, kPick, kUnstack));

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
