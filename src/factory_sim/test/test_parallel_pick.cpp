// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <moveit_pro/task_constructor/solvers.h>
#include <moveit_pro/task_constructor/stages.h>
#include <moveit_pro/task_constructor/task.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <factory_sim/get_joint_state_from_topic.hpp>
#include <factory_sim/is_bracket_pick_start_current.hpp>
#include <factory_sim/is_lookahead_enabled.hpp>
#include <factory_sim/pad_planning_scene_for_motion.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_behavior/behaviors/core/repeat_unless_failure_each_tick.hpp>
#include <moveit_pro_behavior/behaviors/mtc_task_setup/setup_mtc_from_solution.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <moveit_pro_test_utils/test_robot.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
using Solution = moveit_task_constructor_msgs::msg::Solution;
constexpr auto kJointNames = std::array{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
constexpr double kTolerance = 0.02;

class PickStart : public ::testing::Test
{
public:
  PickStart()
  {
    factory.registerNodeType<factory_sim::IsBracketPickStartCurrent>("IsBracketPickStartCurrent");
    solution.sub_trajectory.resize(1);
    auto& trajectory = solution.sub_trajectory.front().trajectory.joint_trajectory;
    trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());
    trajectory.points.resize(1);
    trajectory.points.front().positions.assign(6, 0.0);
    scene.robot_state.joint_state.name = trajectory.joint_names;
    scene.robot_state.joint_state.position.assign(6, 0.0);
    board = BT::Blackboard::create();
  }
  BT::NodeStatus run()
  {
    board->set("solution", solution);
    board->set("scene", scene);
    board->set("tolerance", tolerance);
    feedback.name = scene.robot_state.joint_state.name;
    feedback.position = scene.robot_state.joint_state.position;
    board->set("feedback", feedback);
    board->set("now", now);
    auto tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="check"><Condition ID="IsBracketPickStartCurrent" solution="{solution}" planning_scene="{scene}" position_tolerance="{tolerance}" joint_state="{feedback}" current_time="{now}" reason="{reason}" /></BehaviorTree></root>)",
        board);
    return tree.tickOnce();
  }
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board;
  Solution solution;
  moveit_msgs::msg::PlanningScene scene;
  double tolerance = kTolerance;
  double now = 100.0;
  sensor_msgs::msg::JointState feedback = [] {
    sensor_msgs::msg::JointState state;
    state.header.stamp.sec = 100;
    state.velocity.assign(6, 0.0);
    return state;
  }();
};

TEST_F(PickStart, MissingJointRejectsCandidate)
{
  // GIVEN a live scene missing one arm joint.
  scene.robot_state.joint_state.name.pop_back();
  scene.robot_state.joint_state.position.pop_back();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the reason identifies the missing joint.
  EXPECT_NE(board->get<std::string>("reason").find("joint_6"), std::string::npos);
}

TEST_F(PickStart, EmptySolutionRejectsCandidate)
{
  // GIVEN a solution with no trajectory.
  solution.sub_trajectory.clear();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN a stale empty output cannot authorize execution.
  EXPECT_FALSE(board->get<std::string>("reason").empty());
}

TEST_F(PickStart, DuplicateTrajectoryJointsRejectCandidate)
{
  // GIVEN six positions naming the same arm joint twice.
  solution.sub_trajectory.front().trajectory.joint_trajectory.joint_names.back() = "joint_1";
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the omitted arm joint is identified.
  EXPECT_NE(board->get<std::string>("reason").find("joint_6"), std::string::npos);
}

TEST_F(PickStart, DiffSceneRejectsCandidate)
{
  // GIVEN an incremental scene without a complete state.
  scene.is_diff = true;
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the condition requires a fresh full scene.
  EXPECT_NE(board->get<std::string>("reason").find("full planning scene"), std::string::npos);
}

TEST_F(PickStart, NonfinitePositionRejectsCandidate)
{
  // GIVEN an invalid planned joint position.
  solution.sub_trajectory.front().trajectory.joint_trajectory.points.front().positions.front() =
      std::numeric_limits<double>::quiet_NaN();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the invalid joint is identified.
  EXPECT_NE(board->get<std::string>("reason").find("joint_1"), std::string::npos);
}

TEST_F(PickStart, NegativeToleranceRejectsCandidate)
{
  // GIVEN a negative angular tolerance.
  tolerance = -0.01;
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN configuration validation explains the allowed range.
  EXPECT_NE(board->get<std::string>("reason").find("nonnegative"), std::string::npos);
}

TEST_F(PickStart, BoundaryToleranceIsInclusive)
{
  // GIVEN a start exactly at the configured angular tolerance.
  scene.robot_state.joint_state.position.front() = kTolerance;
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  // THEN a start just outside that tolerance is rejected.
  scene.robot_state.joint_state.position.front() += 1e-6;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  scene.robot_state.joint_state.position.front() = kTolerance - 1e-6;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
}

TEST_F(PickStart, SceneJointsAreMatchedByName)
{
  // GIVEN matching states in opposite joint orders.
  auto& positions = solution.sub_trajectory.front().trajectory.joint_trajectory.points.front().positions;
  positions = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
  scene.robot_state.joint_state.position = positions;
  std::reverse(scene.robot_state.joint_state.name.begin(), scene.robot_state.joint_state.name.end());
  std::reverse(scene.robot_state.joint_state.position.begin(), scene.robot_state.joint_state.position.end());
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  // THEN successful validation clears the rejection reason.
  EXPECT_TRUE(board->get<std::string>("reason").empty());
}

TEST_F(PickStart, RobotStateDiffRejectsCandidate)
{
  // GIVEN a full scene containing an incremental robot state.
  scene.robot_state.is_diff = true;
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the state must be fetched as a complete snapshot.
  EXPECT_NE(board->get<std::string>("reason").find("full planning scene"), std::string::npos);
}

TEST_F(PickStart, MultiDofMotionRejectsCandidate)
{
  // GIVEN motion that the six-joint Fanuc condition cannot represent.
  solution.sub_trajectory.front().trajectory.multi_dof_joint_trajectory.points.resize(1);
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN unsupported motion is rejected explicitly.
  EXPECT_NE(board->get<std::string>("reason").find("multi-DOF"), std::string::npos);
}

TEST_F(PickStart, ShortTrajectoryPositionsRejectCandidate)
{
  // GIVEN a trajectory naming six joints but providing five positions.
  solution.sub_trajectory.front().trajectory.joint_trajectory.points.front().positions.pop_back();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN malformed trajectory data cannot authorize motion.
  EXPECT_NE(board->get<std::string>("reason").find("positions must match"), std::string::npos);
}

TEST_F(PickStart, ShortScenePositionsRejectCandidate)
{
  // GIVEN a scene whose names and positions differ in length.
  scene.robot_state.joint_state.position.pop_back();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN malformed live state cannot authorize motion.
  EXPECT_NE(board->get<std::string>("reason").find("differ in size"), std::string::npos);
}

TEST_F(PickStart, DuplicateLiveJointsRejectCandidate)
{
  // GIVEN an ambiguous live position for an arm joint.
  scene.robot_state.joint_state.name.push_back("joint_1");
  scene.robot_state.joint_state.position.push_back(0.0);
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the duplicated state is rejected.
  EXPECT_NE(board->get<std::string>("reason").find("duplicate joints"), std::string::npos);
}

TEST_F(PickStart, NonfiniteLivePositionRejectsCandidate)
{
  // GIVEN an infinite live position.
  scene.robot_state.joint_state.position.front() = std::numeric_limits<double>::infinity();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN the invalid state is rejected.
  EXPECT_NE(board->get<std::string>("reason").find("nonfinite positions"), std::string::npos);
}

TEST_F(PickStart, NonfiniteToleranceRejectsCandidate)
{
  // GIVEN a NaN tolerance that would defeat an unchecked numeric comparison.
  tolerance = std::numeric_limits<double>::quiet_NaN();
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  // THEN a finite angular tolerance is required.
  EXPECT_NE(board->get<std::string>("reason").find("finite"), std::string::npos);
}

TEST_F(PickStart, SceneOnlyPrefixUsesFirstMotionStart)
{
  // GIVEN a normal MTC fixed-state or collision-rule prefix without motion.
  solution.sub_trajectory.insert(solution.sub_trajectory.begin(), moveit_task_constructor_msgs::msg::SubTrajectory{});
  // WHEN checking the handoff.
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  // THEN the first motion segment, rather than the empty prefix, defines the start.
  EXPECT_TRUE(board->get<std::string>("reason").empty());
}

TEST_F(PickStart, MovingArmRejectsCandidate)
{
  feedback.velocity.front() = 0.03;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(board->get<std::string>("reason").find("not settled"), std::string::npos);
  feedback.velocity.front() = 0.02;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
}

TEST_F(PickStart, StaleAndUnstampedFeedbackRejectCandidate)
{
  feedback.header.stamp.sec = 99;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  feedback.header.stamp.sec = 0;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  feedback.header.stamp.sec = 101;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(PickStart, MissingAndNonfiniteVelocitiesRejectCandidate)
{
  feedback.velocity.clear();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  feedback.velocity.assign(6, 0.0);
  feedback.velocity.back() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST(PickStartPorts, MissingRequiredPortsRejectCandidate)
{
  // GIVEN no solution or planning-scene inputs.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<factory_sim::IsBracketPickStartCurrent>("IsBracketPickStartCurrent");
  auto board = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="missing"><Condition ID="IsBracketPickStartCurrent" solution="{missing_solution}" planning_scene="{missing_scene}" reason="{reason}" /></BehaviorTree></root>)",
      board);
  // WHEN checking the handoff.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  // THEN both missing input ports are reported.
  EXPECT_NE(board->get<std::string>("reason").find("missing_solution"), std::string::npos);
  EXPECT_NE(board->get<std::string>("reason").find("missing_scene"), std::string::npos);
}

class Phase final : public BT::StatefulActionNode
{
public:
  Phase(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<std::vector<std::string>> events,
        BT::Blackboard::Ptr root)
    : BT::StatefulActionNode(name, config), root_(std::move(root)), events_(std::move(events))
  {
    config.blackboard->enableAutoRemapping(true);
  }
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("ticks", 1, "Ticks until phase completion."), BT::InputPort<std::string>("label"),
             BT::InputPort<bool>("fail", false, "Fail this phase when true.") };
  }
  BT::NodeStatus onStart() override
  {
    remaining_ = getInput<int>("ticks").value();
    label_ = getInput<std::string>("label").value();
    events_->push_back(label_ + " start");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (--remaining_ > 0)
      return BT::NodeStatus::RUNNING;
    events_->push_back(label_ + " end");
    return root_->get<bool>(label_ == "motion" ? "motion_failure" : "planning_failure") ? BT::NodeStatus::FAILURE :
                                                                                          BT::NodeStatus::SUCCESS;
  }
  void onHalted() override
  {
    events_->push_back(label_ + " halt");
  }

private:
  BT::Blackboard::Ptr root_;
  int remaining_ = 0;
  std::string label_;
  std::shared_ptr<std::vector<std::string>> events_;
};

class PickOverlap : public ::testing::Test
{
public:
  PickOverlap()
  {
    factory.registerNodeType<factory_sim::IsLookaheadEnabled>("IsLookaheadEnabled");
    factory.registerSimpleAction("LogMessage", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<std::string>("message"), BT::InputPort<std::string>("log_level") });
    factory.registerBuilder<Phase>("Phase", [this](const auto& name, const auto& config) {
      return std::make_unique<Phase>(name, config, events, board);
    });
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Finish Bracket Cycle"><Phase ticks="8" label="motion" fail="{motion_failure}" /></BehaviorTree>
      <BehaviorTree ID="Look for Bracket Part or Empty Bin"><Sequence><Phase ticks="1" label="perception" /><Script code="part_found := !empty_bin" /></Sequence></BehaviorTree>
      <BehaviorTree ID="Plan Bracket Pick"><Phase ticks="1" label="planning" fail="{planning_failure}" /></BehaviorTree>
    </root>)");
    factory.registerBehaviorTreeFromFile(std::string(FACTORY_OBJECTIVES_DIR) +
                                         "/prepare_pick_during_bracket_cycle.xml");
    board->set("parallel", true);
    board->set("near_jig_wait", false);
    board->set("cache_transfers", false);
    board->set("cycle", 1);
    board->set("num_cycles", 3);
    board->set("next_ready", true);
    board->set("next_cycle", -1);
    board->set("motion_failure", false);
    board->set("planning_failure", false);
    board->set("empty_bin", false);
    board->set("next_empty", false);
  }
  BT::Tree makeTree()
  {
    return factory.createTree("Prepare Pick During Bracket Cycle", board);
  }
  BT::NodeStatus finish(BT::Tree& tree)
  {
    for (int i = 0; i < 25; ++i)
      if (const auto status = tree.tickOnce(); status != BT::NodeStatus::RUNNING)
        return status;
    return BT::NodeStatus::RUNNING;
  }
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board = BT::Blackboard::create();
  std::shared_ptr<std::vector<std::string>> events = std::make_shared<std::vector<std::string>>();
};

TEST_F(PickOverlap, ParallelPlanningStartsBeforeMotionEnds)
{
  // GIVEN the production orchestration with deterministic motion and planning phases.
  auto tree = makeTree();
  // WHEN running the parallel mode to completion.
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN planning overlaps motion and publishes a candidate for the next cycle.
  const auto plan = std::find(events->begin(), events->end(), "planning start");
  const auto motion = std::find(events->begin(), events->end(), "motion end");
  EXPECT_LT(plan, motion);
  EXPECT_TRUE(board->get<bool>("next_ready"));
  EXPECT_EQ(board->get<int>("next_cycle"), 2);
}

TEST_F(PickOverlap, SequentialPlanningStartsAfterMotionEnds)
{
  // GIVEN the same production tree with parallelism disabled.
  board->set("parallel", false);
  auto tree = makeTree();
  // WHEN running the sequential mode.
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN the motion finishes before planning begins.
  EXPECT_LT(std::find(events->begin(), events->end(), "motion end"),
            std::find(events->begin(), events->end(), "planning start"));
}

TEST_F(PickOverlap, LiteralXmlBooleansSelectBothModes)
{
  // GIVEN the same string-valued flags sent by an Objective XML caller.
  for (const bool enabled : { false, true })
  {
    board->set("parallel", std::string(enabled ? "true" : "false"));
    events->clear();
    auto tree = makeTree();
    // WHEN the typed boolean port selects the scheduling mode.
    ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
    // THEN the literal flag preserves the requested ordering.
    const auto planning = std::find(events->begin(), events->end(), "planning start");
    const auto motion = std::find(events->begin(), events->end(), "motion end");
    EXPECT_EQ(planning < motion, enabled);
  }
}

TEST(LookaheadFlag, MissingFlagReportsItsPort)
{
  // GIVEN a boolean port wired to an unset blackboard entry.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<factory_sim::IsLookaheadEnabled>("IsLookaheadEnabled");
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="flag"><Condition ID="IsLookaheadEnabled" enabled="{missing}" /></BehaviorTree></root>)");
  // WHEN reading the missing flag.
  try
  {
    std::ignore = tree.tickOnce();
    FAIL() << "An unset lookahead flag must reject the Objective.";
  }
  catch (const BT::RuntimeError& error)
  {
    // THEN the configuration error identifies the boolean port.
    EXPECT_NE(std::string(error.what()).find("enabled"), std::string::npos);
  }
}

TEST(LookaheadFlag, InvalidLiteralReportsItsPort)
{
  // GIVEN a literal that cannot be parsed as a boolean.
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<factory_sim::IsLookaheadEnabled>("IsLookaheadEnabled");
  // WHEN constructing or reading the invalid flag.
  try
  {
    auto tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="flag"><Condition ID="IsLookaheadEnabled" enabled="not_a_boolean" /></BehaviorTree></root>)");
    std::ignore = tree.tickOnce();
    FAIL() << "An invalid lookahead flag must reject the Objective.";
  }
  catch (const BT::LogicError& error)
  {
    // THEN the typed port rejects the value before either mode can run.
    EXPECT_NE(std::string(error.what()).find("enabled"), std::string::npos);
  }
}

TEST_F(PickOverlap, FailedSpeculationInvalidatesPriorCandidate)
{
  // GIVEN an earlier valid candidate followed by a speculative planning failure.
  board->set("planning_failure", true);
  auto tree = makeTree();
  // WHEN completing the current motion.
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN the failed speculative branch leaves no executable candidate.
  EXPECT_FALSE(board->get<bool>("next_ready"));
}

TEST_F(PickOverlap, HaltInvalidatesCompletedCandidate)
{
  // GIVEN a candidate ready while the current motion still runs.
  auto tree = makeTree();
  for (int i = 0; i < 5; ++i)
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(board->get<bool>("next_ready"));
  // WHEN stopping the Objective.
  tree.haltTree();
  // THEN the candidate is invalidated and motion is halted.
  EXPECT_FALSE(board->get<bool>("next_ready"));
  EXPECT_NE(std::find(events->begin(), events->end(), "motion halt"), events->end());
}

TEST_F(PickOverlap, ExecutionFailureInvalidatesCompletedCandidate)
{
  // GIVEN a speculative candidate that completes before an execution failure.
  board->set("motion_failure", true);
  auto tree = makeTree();
  // WHEN the current motion fails.
  ASSERT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN the predicted endpoint cannot authorize the next pick.
  EXPECT_FALSE(board->get<bool>("next_ready"));
}

TEST_F(PickOverlap, EmptyBinFinishesMotionWithoutPreparingAPick)
{
  board->set("empty_bin", true);
  auto tree = makeTree();
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(board->get<bool>("next_ready"));
  EXPECT_TRUE(board->get<bool>("next_empty"));
  EXPECT_EQ(std::find(events->begin(), events->end(), "planning start"), events->end());
  EXPECT_NE(std::find(events->begin(), events->end(), "motion end"), events->end());
}

TEST_F(PickOverlap, FinalCycleDoesNotPrepareAnotherPick)
{
  // GIVEN the final bracket cycle and a candidate from an earlier cycle.
  board->set("cycle", 3);
  auto tree = makeTree();
  // WHEN the cycle completes.
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN no perception or planning starts, and the old candidate is invalidated.
  EXPECT_EQ(std::find(events->begin(), events->end(), "perception start"), events->end());
  EXPECT_FALSE(board->get<bool>("next_ready"));
}

TEST(FuturePickStart, PriorSolutionFinalSceneSeedsNextPlan)
{
  // GIVEN a predecessor whose end differs from its original start state.
  const auto model = moveit_pro::test_utils::createTestRobot(moveit_pro::test_utils::kMinimalUrdfString,
                                                             moveit_pro::test_utils::kMinimalSrdfString);
  auto bounds = model->getJointModel("joint1")->getVariableBounds("joint1");
  bounds.acceleration_bounded = true;
  bounds.max_acceleration = 1.0;
  bounds.min_acceleration = -1.0;
  model->getJointModel("joint1")->setVariableBounds("joint1", bounds);
  auto scene = std::make_shared<moveit_pro::base::planning_scene::PlanningScene>(model);
  Solution predecessor;
  scene->getPlanningSceneMsg(predecessor.start_scene);
  predecessor.start_scene.robot_state.joint_state.position.front() = -0.7;
  predecessor.sub_trajectory.resize(1);
  auto& diff = predecessor.sub_trajectory.front().scene_diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  diff.robot_state.joint_state.name = { "joint1" };
  diff.robot_state.joint_state.position = { -0.3 };
  auto task = std::make_shared<moveit_pro::task_constructor::Task>("future_pick", false);
  task->setRobotModel(model);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<moveit_pro::behaviors::SetupMTCFromSolution>("SetupMTCFromSolution");
  auto board = BT::Blackboard::create();
  board->set("task", task);
  board->set("previous", predecessor);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="seed"><Action ID="SetupMTCFromSolution" task="{task}" solution="{previous}" /></BehaviorTree></root>)",
      board);
  // WHEN planning the next motion from the real seeding Behavior.
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto planner = std::make_shared<moveit_pro::task_constructor::solvers::JointInterpolationPlanner>();
  auto move = std::make_unique<moveit_pro::task_constructor::stages::MoveTo>("next_pick", planner);
  move->setGroup("joint_group");
  move->setGoal(std::map<std::string, double>{ { "joint1", 0.4 } });
  task->add(std::move(move));
  ASSERT_TRUE(task->plan(1));
  moveit_task_constructor_msgs::msg::Solution result;
  task->solutions().front()->toMsg(result);
  // THEN the next plan starts at the predecessor's final position, not its old live position.
  ASSERT_FALSE(result.sub_trajectory.empty());
  const auto motion = std::find_if(result.sub_trajectory.begin(), result.sub_trajectory.end(), [](const auto& segment) {
    return !segment.trajectory.joint_trajectory.points.empty();
  });
  ASSERT_NE(motion, result.sub_trajectory.end());
  EXPECT_DOUBLE_EQ(motion->trajectory.joint_trajectory.points.front().positions.front(), -0.3);
}

class PickHandoffWait : public ::testing::Test
{
public:
  PickHandoffWait()
  {
    factory.registerNodeType<moveit_pro::behaviors::RepeatUnlessFailureEachTick>("RepeatUnlessFailureEachTick");
    factory.registerSimpleAction(
        "GetJointStateFromTopic", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
        factory_sim::GetJointStateFromTopic::providedPorts());
    factory.registerSimpleAction("GetCurrentPlanningScene", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg") });
    factory.registerSimpleCondition(
        "IsBracketPickStartCurrent",
        [this](BT::TreeNode&) { return ++checks >= ready_after ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; },
        factory_sim::IsBracketPickStartCurrent::providedPorts());
    factory.registerBehaviorTreeFromFile(std::string(FACTORY_OBJECTIVES_DIR) + "/wait_for_bracket_pick_handoff.xml");
  }
  BT::NodeStatus finish(BT::Tree& tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    auto status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      status = tree.tickOnce();
    }
    return status;
  }
  BT::BehaviorTreeFactory factory;
  int checks = 0;
  int ready_after = 3;
};

TEST_F(PickHandoffWait, SettlingFeedbackPreservesCandidate)
{
  auto tree = factory.createTree("Wait for Bracket Pick Handoff");
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(checks, 3);
}

TEST_F(PickHandoffWait, PersistentMismatchTimesOutInsteadOfAuthorizingPick)
{
  ready_after = std::numeric_limits<int>::max();
  auto tree = factory.createTree("Wait for Bracket Pick Handoff");
  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now() - started, std::chrono::milliseconds(1900));
  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(3));
}

TEST(PickHandoffFreshness, SceneLatencyPrecedesFeedbackReceipt)
{
  // GIVEN a full scene request that consumes more than the allowed feedback age.
  double clock = 10.0;
  Solution solution;
  solution.sub_trajectory.resize(1);
  auto& trajectory = solution.sub_trajectory.front().trajectory.joint_trajectory;
  trajectory.joint_names.assign(kJointNames.begin(), kJointNames.end());
  trajectory.points.resize(1);
  trajectory.points.front().positions.assign(6, 0.0);
  moveit_msgs::msg::PlanningScene scene;
  scene.robot_state.joint_state.name = trajectory.joint_names;
  scene.robot_state.joint_state.position.assign(6, 0.0);
  auto board = BT::Blackboard::create();
  board->set("solution", solution);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<moveit_pro::behaviors::RepeatUnlessFailureEachTick>("RepeatUnlessFailureEachTick");
  factory.registerNodeType<factory_sim::IsBracketPickStartCurrent>("IsBracketPickStartCurrent");
  factory.registerSimpleAction("GetCurrentPlanningScene",
                               [&](BT::TreeNode& node) {
                                 clock += 0.3;
                                 EXPECT_TRUE(node.setOutput("planning_scene_msg", scene));
                                 return BT::NodeStatus::SUCCESS;
                               },
                               { BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg") });
  factory.registerSimpleAction(
      "GetJointStateFromTopic",
      [&](BT::TreeNode& node) {
        auto feedback = scene.robot_state.joint_state;
        feedback.velocity.assign(6, 0.0);
        feedback.header.stamp = rclcpp::Time(static_cast<int64_t>(clock * 1e9));
        EXPECT_TRUE(node.setOutput("message_out", feedback));
        EXPECT_TRUE(node.setOutput("current_time", clock));
        return BT::NodeStatus::SUCCESS;
      },
      factory_sim::GetJointStateFromTopic::providedPorts());
  factory.registerBehaviorTreeFromFile(std::string(FACTORY_OBJECTIVES_DIR) + "/wait_for_bracket_pick_handoff.xml");
  auto tree = factory.createTree("Wait for Bracket Pick Handoff", board);
  // WHEN the production handoff runs with the real freshness condition.
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // THEN its receipt clock includes scene latency and its feedback is newly sampled.
  EXPECT_DOUBLE_EQ(board->get<double>("now"), clock);
  const auto feedback = board->get<sensor_msgs::msg::JointState>("feedback");
  EXPECT_NEAR(rclcpp::Time(feedback.header.stamp).seconds(), clock, 1e-9);
}

class CachedExecution final : public BT::StatefulActionNode
{
public:
  CachedExecution(const std::string& name, const BT::NodeConfiguration& config, BT::Blackboard::Ptr state)
    : BT::StatefulActionNode(name, config), state_(std::move(state))
  {
  }
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg"),
             BT::InputPort<std::string>("execution_pipeline"), BT::InputPort<std::string>("controller_action_name"),
             BT::InputPort<double>("goal_duration_tolerance") };
  }
  BT::NodeStatus onStart() override
  {
    state_->set("executions", state_->get<int>("executions") + 1);
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (state_->get<bool>("running"))
      return BT::NodeStatus::RUNNING;
    return state_->get<bool>("execution_failure") ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override
  {
    state_->set("halted", true);
  }

private:
  BT::Blackboard::Ptr state_;
};

class CachedMotion : public ::testing::Test
{
public:
  CachedMotion()
  {
    board->set("cache_enabled", true);
    board->set("cache_ready", false);
    board->set("plans", 0);
    board->set("executions", 0);
    board->set("running", false);
    board->set("execution_failure", false);
    board->set("halted", false);
    board->set("cached_solution", Solution{});
    factory.registerNodeType<factory_sim::IsLookaheadEnabled>("IsLookaheadEnabled");
    factory.registerSimpleAction("StopwatchBegin",
                                 [](BT::TreeNode& node) {
                                   std::ignore = node.setOutput("timepoint", 0);
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::OutputPort<int>("timepoint") });
    factory.registerSimpleAction("StopwatchEnd", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<int>("timepoint"), BT::InputPort<std::string>("message") });
    factory.registerSimpleAction("LogMessage", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<std::string>("message"), BT::InputPort<std::string>("log_level") });
    factory.registerSimpleAction("SwitchController", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<std::vector<std::string>>("activate_controllers") });
    factory.registerSimpleAction("PlanMTCTask",
                                 [this](BT::TreeNode& node) {
                                   board->set("plans", board->get<int>("plans") + 1);
                                   std::ignore = node.setOutput("solution", Solution{});
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::BidirectionalPort<moveit_pro::task_constructor::TaskPtr>("task"),
                                   BT::OutputPort<Solution>("solution") });
    factory.registerSimpleAction("ConvertMtcSolutionToJointTrajectory",
                                 [](BT::TreeNode& node) {
                                   std::ignore =
                                       node.setOutput("joint_trajectory", trajectory_msgs::msg::JointTrajectory{});
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<Solution>("solution"), BT::InputPort<std::string>("joint_group"),
                                   BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory") });
    factory.registerSimpleAction("PadPlanningSceneForMotion",
                                 [](BT::TreeNode& node) {
                                   std::ignore = node.setOutput("padded_scene", moveit_msgs::msg::PlanningScene{});
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene"),
                                   BT::InputPort<double>("minimum_padding_m"),
                                   BT::OutputPort<moveit_msgs::msg::PlanningScene>("padded_scene") });
    factory.registerSimpleAction("ValidateTrajectory",
                                 [this](BT::TreeNode&) {
                                   return collision_free ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
                                 },
                                 { BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg"),
                                   BT::InputPort<std::string>("planning_group_name"),
                                   BT::InputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg") });
    factory.registerSimpleCondition("CacheStartAccepted", [this](BT::TreeNode&) {
      return start_matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory.registerBuilder<CachedExecution>("ExecuteTrajectory", [this](const auto& name, const auto& config) {
      return std::make_unique<CachedExecution>(name, config, board);
    });
    factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4">
      <BehaviorTree ID="Wait for Bracket Pick Handoff"><CacheStartAccepted /></BehaviorTree>
      <BehaviorTree ID="Execute MTC Solution"><ExecuteTrajectory /></BehaviorTree>
    </root>)");
    factory.registerBehaviorTreeFromFile(std::string(FACTORY_OBJECTIVES_DIR) +
                                         "/plan_or_execute_cached_fanuc_motion.xml");
  }
  BT::NodeStatus finish(BT::Tree& tree)
  {
    auto status = tree.tickOnce();
    for (int i = 0; i < 4 && status == BT::NodeStatus::RUNNING; ++i)
      status = tree.tickOnce();
    return status;
  }
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr board = BT::Blackboard::create();
  bool collision_free = true;
  bool start_matches = true;
};

TEST_F(CachedMotion, SecondExecutionReusesValidatedMotion)
{
  auto tree = factory.createTree("Plan or Execute Cached Fanuc Motion", board);
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(board->get<bool>("cache_ready"));
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<int>("plans"), 1);
  EXPECT_EQ(board->get<int>("executions"), 2);
}

TEST_F(CachedMotion, SceneOrStartRejectionReplansBeforeExecution)
{
  for (const bool reject_scene : { false, true })
  {
    board->set("cache_ready", true);
    board->set("plans", 0);
    board->set("executions", 0);
    start_matches = reject_scene;
    collision_free = !reject_scene;
    auto tree = factory.createTree("Plan or Execute Cached Fanuc Motion", board);
    EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(board->get<int>("plans"), 1);
    EXPECT_EQ(board->get<int>("executions"), 1);
  }
}

TEST_F(CachedMotion, ExecutionFailureInvalidatesWithoutRetry)
{
  board->set("cache_ready", true);
  board->set("execution_failure", true);
  auto tree = factory.createTree("Plan or Execute Cached Fanuc Motion", board);
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(board->get<bool>("cache_ready"));
  EXPECT_EQ(board->get<int>("plans"), 0);
  EXPECT_EQ(board->get<int>("executions"), 1);
}

TEST_F(CachedMotion, HaltInvalidatesSelectedMotion)
{
  board->set("cache_ready", true);
  board->set("running", true);
  auto tree = factory.createTree("Plan or Execute Cached Fanuc Motion", board);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_TRUE(board->get<bool>("halted"));
  EXPECT_FALSE(board->get<bool>("cache_ready"));
}

TEST_F(CachedMotion, DisabledCachePlansEachExecution)
{
  board->set("cache_enabled", false);
  auto tree = factory.createTree("Plan or Execute Cached Fanuc Motion", board);
  ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<int>("plans"), 2);
}

TEST(PickBehaviorLoader, RegistersHandoffCondition)
{
  // GIVEN the installed workspace plugin description.
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  auto plugin = loader.createSharedInstance("factory_sim::PickBehaviorsLoader");
  BT::BehaviorTreeFactory factory;
  // WHEN loading the plugin without ROS resources.
  plugin->registerBehaviors(factory, nullptr);
  // THEN the exact Behavior ID is available for the Objective.
  EXPECT_TRUE(factory.manifests().contains("IsBracketPickStartCurrent"));
}

class PickObjectiveLibrary : public moveit_pro::test_utils::RosTest
{
};

TEST_F(PickObjectiveLibrary, JointFeedbackTimeoutAndHaltAreBounded)
{
  auto node = std::make_shared<rclcpp::Node>("bracket_feedback_timeout_test");
  node->declare_parameter<std::string>("robot_description", moveit_pro::test_utils::kMinimalUrdfString);
  node->declare_parameter<std::string>("robot_description_semantic", moveit_pro::test_utils::kMinimalSrdfString);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false, true);
  BT::BehaviorTreeFactory factory;
  moveit_pro::behaviors::registerBehavior<factory_sim::GetJointStateFromTopic>(factory, "GetJointStateFromTopic",
                                                                               context);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="timeout"><GetJointStateFromTopic topic_name="/absent_bracket_feedback" publisher_timeout_sec="0.05" message_timeout_sec="0.05" message_out="{feedback}" current_time="{now}" /></BehaviorTree></root>)");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  auto status = tree.tickOnce();
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    status = tree.tickOnce();
  }
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  tree.haltTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  const auto before = std::chrono::steady_clock::now();
  tree.haltTree();
  EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::seconds(1));
}

TEST_F(PickObjectiveLibrary, JointFeedbackRejectsUnboundedTimeouts)
{
  auto node = std::make_shared<rclcpp::Node>("bracket_feedback_ports_test");
  node->declare_parameter<std::string>("robot_description", moveit_pro::test_utils::kMinimalUrdfString);
  node->declare_parameter<std::string>("robot_description_semantic", moveit_pro::test_utils::kMinimalSrdfString);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false, true);
  BT::BehaviorTreeFactory factory;
  moveit_pro::behaviors::registerBehavior<factory_sim::GetJointStateFromTopic>(factory, "GetJointStateFromTopic",
                                                                               context);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="invalid"><GetJointStateFromTopic message_timeout_sec="-1" message_out="{feedback}" current_time="{now}" /></BehaviorTree></root>)");
  auto status = tree.tickOnce();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    status = tree.tickOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(PickObjectiveLibrary, PaddingCopyPreservesMarginsAndRejectsInvalidData)
{
  auto node = std::make_shared<rclcpp::Node>("cached_motion_padding_test");
  node->declare_parameter<std::string>("robot_description", moveit_pro::test_utils::kMinimalUrdfString);
  node->declare_parameter<std::string>("robot_description_semantic", moveit_pro::test_utils::kMinimalSrdfString);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false, true);
  BT::BehaviorTreeFactory factory;
  moveit_pro::behaviors::registerBehavior<factory_sim::PadPlanningSceneForMotion>(factory, "PadPlanningSceneForMotion",
                                                                                  context);
  auto board = BT::Blackboard::create();
  moveit_msgs::msg::PlanningScene scene;
  const auto& names = context->robot_model->getLinkModelNamesWithCollisionGeometry();
  ASSERT_FALSE(names.empty());
  moveit_msgs::msg::LinkPadding existing;
  existing.link_name = names.front();
  existing.padding = 0.03;
  scene.link_padding.push_back(existing);
  board->set("scene", scene);
  board->set("padding", 0.01);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="pad"><PadPlanningSceneForMotion planning_scene="{scene}" minimum_padding_m="{padding}" padded_scene="{out}" /></BehaviorTree></root>)",
      board);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto result = board->get<moveit_msgs::msg::PlanningScene>("out");
  ASSERT_EQ(result.link_padding.size(), names.size());
  for (const auto& padding : result.link_padding)
    EXPECT_DOUBLE_EQ(padding.padding, padding.link_name == names.front() ? 0.03 : 0.01);
  EXPECT_EQ(board->get<moveit_msgs::msg::PlanningScene>("scene"), scene);
  for (double invalid : { -0.01, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
  {
    board->set("padding", invalid);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  board->set("padding", 0.01);
  scene.is_diff = true;
  board->set("scene", scene);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  scene.is_diff = false;
  scene.link_padding.push_back(existing);
  board->set("scene", scene);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(PickObjectiveLibrary, MissingSceneRejectsPadding)
{
  auto node = std::make_shared<rclcpp::Node>("cached_motion_missing_scene_test");
  node->declare_parameter<std::string>("robot_description", moveit_pro::test_utils::kMinimalUrdfString);
  node->declare_parameter<std::string>("robot_description_semantic", moveit_pro::test_utils::kMinimalSrdfString);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false, true);
  BT::BehaviorTreeFactory factory;
  moveit_pro::behaviors::registerBehavior<factory_sim::PadPlanningSceneForMotion>(factory, "PadPlanningSceneForMotion",
                                                                                  context);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="pad"><PadPlanningSceneForMotion planning_scene="{missing}" padded_scene="{out}" /></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(PickObjectiveLibrary, InstantiatesCompletePrototypeWithInstalledPlugins)
{
  // GIVEN the real Runtime loaders and both installed Objective libraries.
  auto node = std::make_shared<rclcpp::Node>("bracket_objective_library_test");
  node->declare_parameter<std::string>("robot_description", moveit_pro::test_utils::kMinimalUrdfString);
  node->declare_parameter<std::string>("robot_description_semantic", moveit_pro::test_utils::kMinimalSrdfString);
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false, true);
  BT::BehaviorTreeFactory factory;
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins;
  for (const auto* name :
       { "moveit_pro::behaviors::ConverterBehaviorsLoader", "moveit_pro::behaviors::CoreBehaviorsLoader",
         "moveit_pro::behaviors::MTCCoreBehaviorsLoader", "moveit_pro::behaviors::VisionBehaviorsLoader",
         "moveit_pro::behaviors::MujocoBehaviorsLoader", "factory_sim::PickBehaviorsLoader" })
  {
    plugins.push_back(loader.createSharedInstance(name));
    plugins.back()->registerBehaviors(factory, context);
  }
  const auto core =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("moveit_pro_objectives")) / "objectives";
  for (const auto* directory : { "core", "motion", "perception", "visualization", "mujoco" })
    for (const auto& entry : std::filesystem::directory_iterator(core / directory))
      if (entry.path().extension() == ".xml")
        factory.registerBehaviorTreeFromFile(entry.path().string());
  for (const auto& entry : std::filesystem::directory_iterator(FACTORY_OBJECTIVES_DIR))
    if (entry.path().extension() == ".xml")
      factory.registerBehaviorTreeFromFile(entry.path().string());
  auto board = BT::Blackboard::create();
  board->set("parallel", true);
  board->set("near_jig_wait", false);
  board->set("cache_transfers", false);
  board->set("num_cycles", 3);
  // WHEN instantiating the complete tree without ticking any robot motion.
  EXPECT_NO_THROW(factory.createTree("Pick and Place Brackets with Lookahead", board));
  factory.registerBehaviorTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="caller"><BehaviorTree ID="caller"><SubTree ID="Pick and Place Brackets with Lookahead" parallel="true" num_cycles="3" /></BehaviorTree></root>)");
  EXPECT_NO_THROW(factory.createTree("caller"));
  factory.registerBehaviorTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="near_jig_caller"><SubTree ID="Pick and Place Brackets with Lookahead" parallel="true" near_jig_wait="true" cache_transfers="true" num_cycles="3" /></BehaviorTree></root>)");
  EXPECT_NO_THROW(factory.createTree("near_jig_caller"));
  for (const int count : { 0, 4 })
  {
    auto invalid = BT::Blackboard::create();
    invalid->set("parallel", true);
    invalid->set("num_cycles", count);
    auto tree = factory.createTree("Pick and Place Brackets with Lookahead", invalid);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }
  // THEN the concrete plugin manifest can validate this exact Objective outside the licensed action server.
  std::ofstream manifest(std::string(FACTORY_OBJECTIVES_DIR) + "/../../../build/factory_sim/bracket_manifest.xml");
  ASSERT_TRUE(manifest.is_open());
  manifest << BT::writeTreeNodesModelXML(factory, true);
}
}  // namespace
