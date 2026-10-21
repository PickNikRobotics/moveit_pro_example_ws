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
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_behavior/behaviors/core/plan_to_joint_goal.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/service_client_interface.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rebot_bench_behaviors/anchor_trajectory_to_command.hpp>
#include <rebot_bench_behaviors/clamp_robot_state_to_bounds.hpp>
#include <rebot_bench_behaviors/hold_gripper_in_trajectory.hpp>
#include <rebot_bench_behaviors/is_joint_at_least.hpp>
#include <rebot_bench_behaviors/plan_to_joint_goal_from_bounded_start.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

namespace
{
// The shipped jaw and waypoint legs run with the real AnchorTrajectoryToCommand and IsJointAtLeast. The planners
// are linear stand-ins that plan from their start state at the joint speed limits, and ExecuteTrajectory moves the
// measured state to each trajectory's end unless the test cancels the move, which then still reports success.
using Positions = std::map<std::string, double>;

const std::vector<std::string> kArm{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
const std::map<std::string, std::vector<std::string>> kGroups{
  { "gripper", { "joint7" } },
  { "manipulator", kArm },
  { "manipulator_and_gripper", { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" } }
};
const std::map<std::string, Positions> kWaypoints{
  { "Stack Gripper Open", { { "joint7", 5.25 } } },
  { "Pick and Stack Survey",
    { { "joint1", 0.0 }, { "joint2", 0.6 }, { "joint3", 1.2 }, { "joint4", 0.3 }, { "joint5", 0.0 }, { "joint6", 0.0 } } },
  { "Zeroed Rest",
    { { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", 0.0 }, { "joint5", 0.0 }, { "joint6", 0.0 } } }
};
constexpr double kJawSpeedLimit = 3.0;
constexpr double kArmSpeedLimit = 0.6;
// joint4 sags below its home command, as on the bench.
const Positions kSaggedHome{ { "joint1", 0.0 },     { "joint2", 0.0 }, { "joint3", 0.0 },
                             { "joint4", -0.0155 }, { "joint5", 0.0 }, { "joint6", 0.0 } };

std::string readFile(const std::string& path)
{
  std::ifstream stream(path);
  return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
}

moveit_msgs::msg::RobotState robotState(const Positions& positions)
{
  moveit_msgs::msg::RobotState state;
  for (const auto& [joint, position] : positions)
  {
    state.joint_state.name.push_back(joint);
    state.joint_state.position.push_back(position);
  }
  return state;
}

Positions positionsOf(const sensor_msgs::msg::JointState& state)
{
  Positions positions;
  for (std::size_t i = 0; i < state.name.size(); ++i)
    positions[state.name[i]] = state.position.at(i);
  return positions;
}

Positions withJaw(Positions positions, const double jaw)
{
  positions["joint7"] = jaw;
  return positions;
}

double seconds(const builtin_interfaces::msg::Duration& duration)
{
  return rclcpp::Duration(duration).seconds();
}

trajectory_msgs::msg::JointTrajectory linearTrajectory(const std::vector<std::string>& joints, const Positions& start,
                                                       const Positions& goal)
{
  double duration = 0.5;
  for (const auto& joint : joints)
    duration = std::max(duration, std::abs(goal.at(joint) - start.at(joint)) /
                                      (joint == "joint7" ? kJawSpeedLimit : kArmSpeedLimit));
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joints;
  constexpr int kSegments = 10;
  for (int i = 0; i <= kSegments; ++i)
  {
    const double s = static_cast<double>(i) / kSegments;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(s * duration);
    for (const auto& joint : joints)
    {
      const double distance = goal.at(joint) - start.at(joint);
      point.positions.push_back(start.at(joint) + s * distance);
      point.velocities.push_back(i == 0 || i == kSegments ? 0.0 : distance / duration);
      point.accelerations.push_back(0.0);
    }
    trajectory.points.push_back(point);
  }
  return trajectory;
}

std::vector<double> column(const trajectory_msgs::msg::JointTrajectory& trajectory, const std::string& joint)
{
  const auto index =
      static_cast<std::size_t>(std::ranges::find(trajectory.joint_names, joint) - trajectory.joint_names.begin());
  std::vector<double> positions;
  for (const auto& point : trajectory.points)
    positions.push_back(point.positions.at(index));
  return positions;
}

// The fastest the trajectory commands joint7, from its velocities and from the change between points.
double peakJawSpeed(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  const auto index =
      static_cast<std::size_t>(std::ranges::find(trajectory.joint_names, "joint7") - trajectory.joint_names.begin());
  double peak = 0.0;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto& point = trajectory.points[i];
    if (!point.velocities.empty())
      peak = std::max(peak, std::abs(point.velocities.at(index)));
    if (i > 0)
    {
      const auto& previous = trajectory.points[i - 1];
      peak = std::max(peak, std::abs(point.positions.at(index) - previous.positions.at(index)) /
                                (seconds(point.time_from_start) - seconds(previous.time_from_start)));
    }
  }
  return peak;
}

// The scene service's answer: the robot's own scene, with the robot where the arm measures.
class MeasuredScene final : public moveit_pro::behaviors::ClientInterfaceBase<moveit_pro::behaviors::GetPlanningScene>
{
public:
  MeasuredScene(const moveit_msgs::msg::PlanningScene& scene, const Positions& measured)
    : scene_(scene), measured_(measured)
  {
  }
  void initialize(const std::string&, std::chrono::duration<double>, std::chrono::duration<double>) override
  {
  }
  bool waitForServiceServer() const override
  {
    return true;
  }
  tl::expected<moveit_pro::behaviors::GetPlanningScene::Response, std::string>
  syncSendRequest(const moveit_pro::behaviors::GetPlanningScene::Request&) override
  {
    moveit_pro::behaviors::GetPlanningScene::Response response;
    response.scene = scene_;
    response.scene.robot_state = robotState(measured_);
    return response;
  }
  void cancelRequest() override
  {
  }

private:
  const moveit_msgs::msg::PlanningScene& scene_;
  const Positions& measured_;
};

// The arm off HOME with joint3 sagged below its -0.01 rad lower bound, far enough to put link1 in self-collision.
const Positions kDeepSag{ { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", -0.08 },
                          { "joint4", 0.5 }, { "joint5", 0.3 }, { "joint6", 0.0 } };

class JawLegs : public testing::Test
{
protected:
  void SetUp() override
  {
    using moveit_msgs::msg::PlanningScene;
    using moveit_msgs::msg::RobotState;
    using trajectory_msgs::msg::JointTrajectory;
    const auto urdf = urdf::parseURDF(readFile(TEST_URDF));
    ASSERT_TRUE(urdf);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, readFile(PACKAGE_DIR "/config/moveit/rebot.srdf")));
    context->robot_model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
    // The planning limits the runtime applies over the URDF.
    for (const auto& entry : YAML::LoadFile(PACKAGE_DIR "/config/moveit/joint_limits.yaml")["joint_limits"])
    {
      const auto name = entry.first.as<std::string>();
      auto* joint = context->robot_model->getJointModel(name);
      ASSERT_NE(joint, nullptr);
      auto bounds = joint->getVariableBounds().front();
      if (entry.second["has_acceleration_limits"].as<bool>(false))
      {
        bounds.acceleration_bounded = true;
        bounds.max_acceleration = entry.second["max_acceleration"].as<double>();
        bounds.min_acceleration = -bounds.max_acceleration;
      }
      joint->setVariableBounds(name, bounds);
    }
    moveit_pro::behaviors::registerBehavior<rebot_bench_behaviors::AnchorTrajectoryToCommand>(
        factory, "AnchorTrajectoryToCommand", context);
    moveit_pro::behaviors::registerBehavior<rebot_bench_behaviors::ClampRobotStateToBounds>(
        factory, "ClampRobotStateToBounds", context);
    moveit_pro::behaviors::registerBehavior<rebot_bench_behaviors::IsJointAtLeast>(factory, "IsJointAtLeast", context);
    moveit_pro::behaviors::registerBehavior<rebot_bench_behaviors::HoldGripperInTrajectory>(
        factory, "HoldGripperInTrajectory", context);
    factory.registerSimpleAction("ResetVector", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("vector") });
    factory.registerSimpleAction("PushBackVector", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_vector"),
                                   BT::InputPort<geometry_msgs::msg::PoseStamped>("element"),
                                   BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_vector") });
    factory.registerSimpleAction(
        "ComputeInverseKinematics",
        [this](BT::TreeNode& node) {
          node.setOutput("solution", robotState(pose_goal));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("target_poses"),
          BT::InputPort<std::string>("tip_links"), BT::InputPort<PlanningScene>("planning_scene_msg"),
          BT::InputPort<std::string>("planning_group_name"), BT::InputPort<double>("ik_timeout_s"),
          BT::InputPort<std::string>("solve_mode"), BT::InputPort<double>("optimization_timeout_s"),
          BT::InputPort<double>("link_padding"), BT::OutputPort<RobotState>("solution") });
    factory.registerSimpleAction(
        "RetrieveWaypoint",
        [](BT::TreeNode& node) {
          node.setOutput("waypoint_joint_state",
                         robotState(kWaypoints.at(node.getInput<std::string>("waypoint_name").value())));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("joint_group_name"), BT::InputPort<std::string>("waypoint_name"),
          BT::OutputPort<RobotState>("waypoint_joint_state") });
    factory.registerSimpleAction("GetCurrentPlanningScene",
                                 [](BT::TreeNode& node) {
                                   node.setOutput("planning_scene_msg", PlanningScene{});
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::OutputPort<PlanningScene>("planning_scene_msg") });
    factory.registerSimpleAction(
        "GetJointState",
        [this](BT::TreeNode& node) {
          Positions group;
          for (const auto& joint : kGroups.at(node.getInput<std::string>("planning_group_name").value()))
            group[joint] = measured.at(joint);
          const auto state = robotState(group);
          node.setOutput("robot_joint_state", state);
          node.setOutput("joint_state", state.joint_state);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<PlanningScene>("planning_scene"), BT::InputPort<std::string>("planning_group_name"),
          BT::OutputPort<RobotState>("robot_joint_state"),
          BT::OutputPort<sensor_msgs::msg::JointState>("joint_state") });
    factory.registerSimpleAction(
        "GeneratePointToPointTrajectory",
        [](BT::TreeNode& node) {
          const auto& joints = kGroups.at(node.getInput<std::string>("planning_group_name").value());
          const auto start = positionsOf(node.getInput<RobotState>("start_state").value().joint_state);
          auto goal = start;
          for (const auto& [joint, position] :
               positionsOf(node.getInput<RobotState>("target_state").value().joint_state))
            goal[joint] = position;
          node.setOutput("joint_trajectory_msg", linearTrajectory(joints, start, goal));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("planning_group_name"), BT::InputPort<RobotState>("start_state"),
          BT::InputPort<RobotState>("target_state"), BT::InputPort<double>("velocity_scale_factor"),
          BT::InputPort<double>("acceleration_scale_factor"), BT::InputPort<double>("jerk_scale_factor"),
          BT::InputPort<double>("trajectory_sampling_rate"), BT::InputPort<double>("start_time"),
          BT::OutputPort<JointTrajectory>("joint_trajectory_msg") });
    // The pose legs plan with PlanToJointGoal, the waypoint legs with PlanToJointGoalFromBoundedStart.
    for (const auto* planner : { "PlanToJointGoal", "PlanToJointGoalFromBoundedStart" })
      factory.registerSimpleAction(
          planner,
          [this](BT::TreeNode& node) {
            if (planner_refuses_start)
              return BT::NodeStatus::FAILURE;
            const auto& joints = kGroups.at(node.getInput<std::string>("planning_group_name").value());
            auto goal = measured;
            for (const auto& [joint, position] :
                 positionsOf(node.getInput<RobotState>("joint_goal").value().joint_state))
              goal[joint] = position;
            node.setOutput("joint_trajectory_msg", linearTrajectory(joints, measured, goal));
            return BT::NodeStatus::SUCCESS;
          },
          { BT::InputPort<RobotState>("joint_goal"), BT::InputPort<std::string>("planning_group_name"),
            BT::InputPort<double>("velocity_scale_factor"), BT::InputPort<double>("acceleration_scale_factor"),
            BT::InputPort<std::string>("collision_check_params"),
            BT::OutputPort<JointTrajectory>("joint_trajectory_msg") });
    // Validation checks only that the trajectory is for the group it names.
    factory.registerSimpleAction(
        "ValidateTrajectory",
        [this](BT::TreeNode& node) {
          if (++validations == refused_validation)
            return BT::NodeStatus::FAILURE;
          const auto& group = kGroups.at(node.getInput<std::string>("planning_group_name").value());
          const auto names = node.getInput<JointTrajectory>("joint_trajectory_msg").value().joint_names;
          return std::ranges::all_of(names, [&](const auto& joint) { return std::ranges::count(group, joint) > 0; }) ?
                     BT::NodeStatus::SUCCESS :
                     BT::NodeStatus::FAILURE;
        },
        { BT::InputPort<PlanningScene>("planning_scene_msg"), BT::InputPort<std::string>("planning_group_name"),
          BT::InputPort<JointTrajectory>("joint_trajectory_msg"), BT::InputPort<double>("joint_space_step"),
          BT::InputPort<double>("cartesian_space_step"), BT::OutputPort<std::string>("debug_solution") });
    factory.registerSimpleAction("RecordRebotTrajectory", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<JointTrajectory>("trajectory") });
    factory.registerSimpleAction(
        "ExecuteTrajectory",
        [this](BT::TreeNode& node) {
          const auto trajectory = node.getInput<JointTrajectory>("joint_trajectory_msg").value();
          executed.push_back(trajectory);
          if (cancels > 0)
          {
            --cancels;
            return BT::NodeStatus::SUCCESS;
          }
          for (std::size_t j = 0; j < trajectory.joint_names.size(); ++j)
            measured[trajectory.joint_names[j]] = trajectory.points.back().positions.at(j);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::string>("execution_pipeline"), BT::InputPort<std::string>("controller_action_name"),
          BT::InputPort<JointTrajectory>("joint_trajectory_msg"), BT::InputPort<double>("goal_duration_tolerance") });
    factory.registerSimpleAction("SwitchController", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                                 { BT::InputPort<std::string>("activate_controllers"),
                                   BT::InputPort<std::string>("automatic_deactivation"),
                                   BT::InputPort<std::string>("strictness") });
    factory.registerSimpleAction(
        "UnpackJointStateMessage",
        [](BT::TreeNode& node) {
          node.setOutput("position", node.getInput<sensor_msgs::msg::JointState>("joint_state").value().position);
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<sensor_msgs::msg::JointState>("joint_state"),
          BT::OutputPort<std::vector<double>>("position") });
    factory.registerSimpleAction(
        "GetElementOfVector",
        [](BT::TreeNode& node) {
          const auto vector = node.getInput<std::vector<double>>("vector_in").value();
          node.setOutput("element", vector.at(static_cast<std::size_t>(node.getInput<int>("index").value())));
          return BT::NodeStatus::SUCCESS;
        },
        { BT::InputPort<std::vector<double>>("vector_in"), BT::InputPort<int>("index"),
          BT::OutputPort<double>("element") });
    factory.registerSimpleAction("CreateRobotState",
                                 [](BT::TreeNode& node) {
                                   const auto names = node.getInput<std::vector<std::string>>("joint_names").value();
                                   const auto positions = node.getInput<std::vector<double>>("positions").value();
                                   Positions state;
                                   for (std::size_t i = 0; i < names.size(); ++i)
                                     state[names[i]] = positions.at(i);
                                   node.setOutput("joint_state_msg", robotState(state));
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<std::vector<std::string>>("joint_names"),
                                   BT::InputPort<std::vector<double>>("positions"),
                                   BT::OutputPort<RobotState>("joint_state_msg") });
    for (const auto* objective : { "move_rebot_gripper.xml", "rebot_open_jaw.xml", "rebot_move_to_arm_waypoint.xml",
                                   "rebot_move_to_pose.xml", "return_rebot_leader_home.xml" })
      factory.registerBehaviorTreeFromFile(std::string(PACKAGE_DIR "/objectives/") + objective);
  }

  BT::NodeStatus run(const std::string& subtree_call, const Positions& commanded)
  {
    tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + subtree_call +
                                      "</BehaviorTree></root>");
    tree.rootBlackboard()->set("commanded_state", robotState(commanded));
    tree.rootBlackboard()->set("target", geometry_msgs::msg::PoseStamped{});
    return tree.tickWhileRunning();
  }

  // Replaces the planner stubs with the native planners, which need a MoveIt Pro license.
  void useNativePlanners()
  {
    factory.unregisterBuilder("PlanToJointGoal");
    factory.unregisterBuilder("PlanToJointGoalFromBoundedStart");
    registerPlanner<moveit_pro::behaviors::PlanToJointGoal>("PlanToJointGoal");
    registerPlanner<rebot_bench_behaviors::PlanToJointGoalFromBoundedStart>("PlanToJointGoalFromBoundedStart");
    moveit_pro::base::planning_scene::PlanningScene(context->robot_model).getPlanningSceneMsg(scene);
  }

  template <typename Planner>
  void registerPlanner(const std::string& id)
  {
    factory.registerBuilder(BT::CreateManifest<Planner>(id), [this](const std::string& name,
                                                                    const BT::NodeConfig& config) {
      return std::make_unique<Planner>(name, config, context, std::make_unique<MeasuredScene>(scene, measured));
    });
  }

  BT::NodeStatus planHome(const std::string& planner)
  {
    tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><Action ID=")" + planner +
        R"(" joint_goal="{goal}" planning_group_name="manipulator" )"
        R"(velocity_scale_factor="1.0" acceleration_scale_factor="1.0" )"
        R"(collision_check_params="[{override_scene_link_padding: true, link_padding: 0.0}]" )"
        R"(joint_trajectory_msg="{plan}" /></BehaviorTree></root>)");
    tree.rootBlackboard()->set("goal", robotState(kWaypoints.at("Zeroed Rest")));
    return tree.tickWhileRunning();
  }

  bool withinBounds(const Positions& positions) const
  {
    moveit_pro::base::RobotState state(context->robot_model);
    state.setToDefaultValues();
    for (const auto& [joint, position] : positions)
      state.setVariablePosition(joint, position);
    return std::ranges::all_of(positions, [&](const auto& joint) {
      return state.satisfiesBounds(context->robot_model->getJointModel(joint.first), 0.0);
    });
  }

  Positions committed()
  {
    return positionsOf(tree.rootBlackboard()->get<moveit_msgs::msg::RobotState>("commanded_state").joint_state);
  }

  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context =
      std::make_shared<moveit_pro::behaviors::BehaviorContext>(std::make_shared<rclcpp::Node>("test_jaw_legs"), false);
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  Positions measured;
  moveit_msgs::msg::PlanningScene scene;
  std::vector<trajectory_msgs::msg::JointTrajectory> executed;
  int cancels = 0;
  int validations = 0;
  int refused_validation = -1;
  Positions pose_goal = withJaw(kWaypoints.at("Pick and Stack Survey"), 0.0);
  // The planner refuses the measured start, as it does the bench arm's sagged home.
  bool planner_refuses_start = false;
};

const std::string kOpenJaw = R"(<SubTree ID="reBot Open Jaw" gripper_velocity_scale_factor="1.0" )"
                             R"(gripper_acceleration_scale_factor="1.0" commanded_state="{commanded_state}" />)";
const std::string kHome = R"(<SubTree ID="reBot Move to Arm Waypoint" waypoint_name="Zeroed Rest" )"
                          R"(velocity_scale_factor="1.0" acceleration_scale_factor="1.0" )"
                          R"(commanded_state="{commanded_state}" />)";

TEST_F(JawLegs, NativePlannerRefusesADeepSagThatTheBoundedStartPlansFromItsBound)
{
  // GIVEN the arm off HOME with joint3 sagged into self-collision below its lower bound
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  measured = withJaw(kDeepSag, 2.0);
  ASSERT_FALSE(withinBounds(measured));

  // WHEN planning home with the native planner, from the measured and from the bounded start
  // THEN only the bounded start plans, and its plan starts at the bound and ends home
  EXPECT_EQ(planHome("PlanToJointGoal"), BT::NodeStatus::FAILURE);
  ASSERT_EQ(planHome("PlanToJointGoalFromBoundedStart"), BT::NodeStatus::SUCCESS);
  const auto plan = tree.rootBlackboard()->get<trajectory_msgs::msg::JointTrajectory>("plan");
  EXPECT_DOUBLE_EQ(column(plan, "joint3").front(), -0.01);
  EXPECT_NEAR(column(plan, "joint4").front(), 0.5, 1e-9);
  for (const auto& joint : kArm)
    EXPECT_NEAR(column(plan, joint).back(), 0.0, 1e-6) << joint;
}

TEST_F(JawLegs, NativePlannerStartsAShallowSagPastTheBoundWhereTheBoundedStartBeginsAtIt)
{
  // GIVEN the arm off HOME with joint3 sagged just below its lower bound
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  auto start = kDeepSag;
  start["joint3"] = -0.02;
  measured = withJaw(start, 2.0);

  // WHEN planning home with the native planner, from the measured and from the bounded start
  // THEN the measured plan begins past the bound, and the bounded one at it
  ASSERT_EQ(planHome("PlanToJointGoal"), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(column(tree.rootBlackboard()->get<trajectory_msgs::msg::JointTrajectory>("plan"), "joint3").front(),
                   -0.02);
  ASSERT_EQ(planHome("PlanToJointGoalFromBoundedStart"), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(column(tree.rootBlackboard()->get<trajectory_msgs::msg::JointTrajectory>("plan"), "joint3").front(),
                   -0.01);
}

TEST_F(JawLegs, LeaderHomeReturnPlansBothLegsFromTheBoundedArmWhenAJointSagsPastItsLimit)
{
  // GIVEN a session that ended with the wrist raised and yawed, and joint3 sagged into self-collision below its bound
  if (std::getenv("MOVEIT_LICENSE_KEY") == nullptr)
    GTEST_SKIP() << "Native planning needs a MoveIt Pro license in MOVEIT_LICENSE_KEY.";
  useNativePlanners();
  measured = withJaw(kDeepSag, 2.0);

  // WHEN returning home, planning with the native planner
  EXPECT_EQ(run(R"(<SubTree ID="Return reBot Leader Home" />)", {}), BT::NodeStatus::SUCCESS);

  // THEN both legs start from joint3 at its bound, yaw centres with the wrist still raised, the arm ends home and
  // the jaw is held where it measures
  ASSERT_EQ(executed.size(), 2U);
  EXPECT_DOUBLE_EQ(column(executed[0], "joint3").front(), -0.01);
  EXPECT_DOUBLE_EQ(column(executed[1], "joint3").front(), -0.01);
  for (const auto& position : column(executed[0], "joint4"))
    EXPECT_NEAR(position, 0.5, 1e-6);
  EXPECT_NEAR(column(executed[0], "joint5").back(), 0.0, 1e-6);
  for (const auto& trajectory : executed)
    for (const auto& position : column(trajectory, "joint7"))
      EXPECT_DOUBLE_EQ(position, 2.0);
  for (const auto& [joint, position] : kWaypoints.at("Zeroed Rest"))
    EXPECT_NEAR(measured.at(joint), position, 1e-6) << joint;
  EXPECT_DOUBLE_EQ(measured.at("joint7"), 2.0);
}

TEST_F(JawLegs, OpenJawRetryReopensFromTheMeasuredJawAfterACanceledOpening)
{
  // GIVEN a jaw commanded open but measured at 2.0, and an opening the controller cancels yet reports as success
  measured = withJaw(kSaggedHome, 2.0);
  cancels = 1;

  // WHEN opening the jaw
  EXPECT_EQ(run(kOpenJaw, withJaw(kWaypoints.at("Zeroed Rest"), 5.25)), BT::NodeStatus::SUCCESS);

  // THEN both attempts move the jaw from where it measures to open, holding the arm at its command
  ASSERT_EQ(executed.size(), 2U);
  for (const auto& trajectory : executed)
  {
    EXPECT_DOUBLE_EQ(column(trajectory, "joint7").front(), 2.0);
    EXPECT_DOUBLE_EQ(column(trajectory, "joint7").back(), 5.25);
    for (const auto& position : column(trajectory, "joint4"))
      EXPECT_DOUBLE_EQ(position, 0.0);
    EXPECT_LE(peakJawSpeed(trajectory), kJawSpeedLimit * (1.0 + 1e-6));
  }
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 5.25);
}

TEST_F(JawLegs, HomeLegHoldsAStaleJawWhereItMeasures)
{
  // GIVEN the arm at home after a canceled opening: the jaw commanded open but measured at 2.0
  measured = withJaw(kSaggedHome, 2.0);

  // WHEN returning home, as the stack's recovery does
  EXPECT_EQ(run(kHome, withJaw(kWaypoints.at("Zeroed Rest"), 5.25)), BT::NodeStatus::SUCCESS);

  // THEN the jaw stays where it measures, so Close Gripper closes it from there
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.0);
  EXPECT_LE(peakJawSpeed(executed.front()), kJawSpeedLimit);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.0);
}

TEST_F(JawLegs, HomeLegKeepsTheGraspSqueeze)
{
  // GIVEN the jaw squeezing a held cube: commanded to the 45 mm grasp, stopped by the cube where it stalls on the
  // bench, about 0.049 rad wider than the command
  measured = withJaw(kSaggedHome, 2.8615);

  // WHEN returning home
  EXPECT_EQ(run(kHome, withJaw(kWaypoints.at("Zeroed Rest"), 2.8125)), BT::NodeStatus::SUCCESS);

  // THEN the jaw command stays at the grasp throughout
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.8125);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.8125);
}
TEST_F(JawLegs, ArmLegMovesFromTheCommandWhenThePlannerRefusesTheMeasuredStart)
{
  // GIVEN the arm at its sagged home, which the planner refuses as a start, and the jaw commanded open but measured
  // at 2.0
  measured = withJaw(kSaggedHome, 2.0);
  planner_refuses_start = true;

  // WHEN moving to the survey waypoint
  EXPECT_EQ(run(R"(<SubTree ID="reBot Move to Arm Waypoint" waypoint_name="Pick and Stack Survey" )"
                R"(velocity_scale_factor="1.0" acceleration_scale_factor="1.0" commanded_state="{commanded_state}" />)",
                withJaw(kWaypoints.at("Zeroed Rest"), 5.25)),
            BT::NodeStatus::SUCCESS);

  // THEN the arm moves from its command, not its sag, to the waypoint, and the jaw stays where it measures
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& [joint, position] : kWaypoints.at("Pick and Stack Survey"))
  {
    EXPECT_DOUBLE_EQ(column(executed.front(), joint).front(), 0.0) << joint;
    EXPECT_DOUBLE_EQ(column(executed.front(), joint).back(), position) << joint;
    EXPECT_DOUBLE_EQ(committed().at(joint), position) << joint;
  }
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.0);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.0);
}
TEST_F(JawLegs, WaypointFallbackFadesStaleArmStartAndHoldsStaleJaw)
{
  measured = withJaw(kSaggedHome, 2.0);
  const double measured_start = measured.at("joint4");
  auto stale = withJaw(kWaypoints.at("Zeroed Rest"), 5.25);
  stale["joint4"] = 0.2;
  planner_refuses_start = true;
  EXPECT_EQ(run(R"(<SubTree ID="reBot Move to Arm Waypoint" waypoint_name="Pick and Stack Survey" )"
                R"(velocity_scale_factor="1.0" acceleration_scale_factor="1.0" commanded_state="{commanded_state}" />)",
                stale),
            BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 1U);
  EXPECT_NEAR(column(executed.front(), "joint4").front(), measured_start, 1e-12);
  for (const auto& [joint, goal] : kWaypoints.at("Pick and Stack Survey"))
  {
    EXPECT_DOUBLE_EQ(column(executed.front(), joint).back(), goal) << joint;
    EXPECT_DOUBLE_EQ(committed().at(joint), goal) << joint;
  }
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.0);
  EXPECT_DOUBLE_EQ(committed().at("joint7"), 2.0);
  EXPECT_EQ(validations, 2);
}

const std::string kPose = R"(<SubTree ID="reBot Move to Pose" target="{target}" gripper_position="2.8125" )"
                          R"(velocity_scale_factor="1.0" acceleration_scale_factor="1.0" )"
                          R"(commanded_state="{commanded_state}" />)";

TEST_F(JawLegs, PoseFallbackHoldsTheGraspAndKeepsThePlannedEndpoint)
{
  measured = withJaw(kSaggedHome, 2.8615);
  planner_refuses_start = true;
  EXPECT_EQ(run(kPose, withJaw(kWaypoints.at("Zeroed Rest"), 2.8125)), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 1U);
  for (const auto& position : column(executed.front(), "joint7"))
    EXPECT_DOUBLE_EQ(position, 2.8125);
  for (const auto& joint : kArm)
    EXPECT_DOUBLE_EQ(column(executed.front(), joint).back(), pose_goal.at(joint));
  EXPECT_EQ(validations, 2);
}

TEST_F(JawLegs, PoseCompensationKeepsTheEndpointWhenTheArmCommandIsStale)
{
  measured = withJaw(kSaggedHome, 2.8615);
  const double measured_start = measured.at("joint4");
  auto stale = withJaw(kWaypoints.at("Zeroed Rest"), 2.8125);
  stale["joint4"] = 0.2;
  planner_refuses_start = true;
  EXPECT_EQ(run(kPose, stale), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(executed.size(), 1U);
  EXPECT_NEAR(column(executed.front(), "joint4").front(), measured_start, 1e-12);
  EXPECT_DOUBLE_EQ(column(executed.front(), "joint4").back(), pose_goal.at("joint4"));
}

TEST_F(JawLegs, PoseRefusesAnInvalidAnchoredPathBeforeExecution)
{
  measured = withJaw(kSaggedHome, 2.8615);
  planner_refuses_start = true;
  refused_validation = 2;
  EXPECT_EQ(run(kPose, withJaw(kWaypoints.at("Zeroed Rest"), 2.8125)), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(executed.empty());
  EXPECT_EQ(validations, 2);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
