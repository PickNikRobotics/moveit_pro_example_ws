// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
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

// Plan-only companion for test_block_transfer.py. No execution or live-scene clients.
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro/task_constructor/stages/fixed_state.h>
#include <moveit_pro/task_constructor/task.h>
#include <tinyxml2.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model_loader/robot_model_loader.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Json = nlohmann::json;
using Solution = moveit_task_constructor_msgs::msg::Solution;

[[nodiscard]] Json trajectories(const Solution& solution)
{
  Json result = Json::array();
  for (const auto& stage : solution.sub_trajectory)
  {
    const auto& trajectory = stage.trajectory.joint_trajectory;
    Json row{ { "joints", trajectory.joint_names }, { "points", Json::array() } };
    for (const auto& point : trajectory.points)
    {
      row["points"].push_back(
          { { "t", point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 }, { "q", point.positions } });
    }
    result.push_back(row);
  }
  return result;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "Expected cases JSON, Objective XML, and result JSON paths.\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "block_transfer_plan_test", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
      node, std::make_unique<moveit_pro::behavior::LoggerBase>(), false, false);
  const auto model_loader = moveit_pro::base::robot_model_loader::RobotModelLoader::create(
      node, moveit_pro::base::robot_model_loader::RobotModelLoader::Options());
  if (!model_loader)
  {
    std::cerr << model_loader.error() << '\n';
    return 2;
  }
  context->robot_model = (*model_loader)->getModel();
  geometry_msgs::msg::TransformStamped anchor;
  anchor.header.frame_id = "odom";
  anchor.child_frame_id = "test_anchor";
  anchor.transform.rotation.w = 1;
  context->transform_buffer_ptr->setTransform(anchor, "fixture", true);
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins;
  BT::BehaviorTreeFactory factory;
  for (const auto* name : { "CoreBehaviorsLoader", "MTCCoreBehaviorsLoader", "ConverterBehaviorsLoader" })
  {
    plugins.push_back(loader.createSharedInstance(std::string("moveit_pro::behaviors::") + name));
    plugins.back()->registerBehaviors(factory, context);
  }
  auto scene = std::make_shared<moveit_pro::base::planning_scene::PlanningScene>(context->robot_model);
  moveit_msgs::msg::RobotState waypoint;
  // Replace service-backed state acquisition only; all pose math, planners and solution splitting are real.
  std::ignore = factory.unregisterBuilder("GetCurrentPlanningScene");
  factory.registerSimpleAction("GetCurrentPlanningScene",
                               [&](BT::TreeNode& behavior) {
                                 moveit_msgs::msg::PlanningScene message;
                                 scene->getPlanningSceneMsg(message);
                                 behavior.setOutput("planning_scene_msg", message);
                                 return BT::NodeStatus::SUCCESS;
                               },
                               { BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg") });
  std::ignore = factory.unregisterBuilder("RetrieveWaypoint");
  factory.registerSimpleAction("RetrieveWaypoint",
                               [&](BT::TreeNode& behavior) {
                                 behavior.setOutput("waypoint_joint_state", waypoint);
                                 return BT::NodeStatus::SUCCESS;
                               },
                               { BT::InputPort<std::string>("joint_group_name"),
                                 BT::InputPort<std::string>("waypoint_name"),
                                 BT::OutputPort<moveit_msgs::msg::RobotState>("waypoint_joint_state") });
  std::ignore = factory.unregisterBuilder("SetupMTCCurrentState");
  factory.registerSimpleAction(
      "SetupMTCCurrentState",
      [&](BT::TreeNode& behavior) {
        const auto task = behavior.getInput<moveit_pro::task_constructor::TaskPtr>("task").value();
        auto state = std::make_unique<moveit_pro::task_constructor::stages::FixedState>("current state");
        state->setState(scene);
        // CurrentState does not check fixed table/table-leg overlaps; moving-group checks remain enabled below.
        state->setIgnoreCollisions(true);
        task->add(std::move(state));
        return BT::NodeStatus::SUCCESS;
      },
      { BT::InputPort<moveit_pro::task_constructor::TaskPtr>("task") });
  tinyxml2::XMLDocument xml;
  if (xml.LoadFile(argv[2]) != tinyxml2::XML_SUCCESS)
  {
    std::cerr << "Cannot load Plan Block Transfer Objective XML.\n";
    return 2;
  }
  // No telemetry server is needed by an offline planning test.
  std::function<void(tinyxml2::XMLElement*)> disable_telemetry = [&](tinyxml2::XMLElement* element) {
    if (element->Attribute("ID") && std::string(element->Attribute("ID")) == "InitializeMTCTask")
    {
      element->SetAttribute("enable_introspection", "false");
    }
    for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      disable_telemetry(child);
    }
  };
  disable_telemetry(xml.RootElement());
  tinyxml2::XMLPrinter printer;
  xml.Print(&printer);
  factory.registerBehaviorTreeFromText(printer.CStr());
  std::ifstream input(argv[1]);
  const Json cases = Json::parse(input);
  Json results = Json::array();
  for (const auto& item : cases)
  {
    const std::string arm = item.at("arm");
    const std::string group = arm + "_manipulator_without_spine";
    auto& state = scene->getCurrentStateNonConst();
    state.setToDefaultValues();
    for (const auto& [name, value] : item.at("joints").items())
    {
      if (context->robot_model->hasJointModel(name))
      {
        state.setVariablePosition(name, value.get<double>());
      }
    }
    state.update();
    moveit_pro::base::collision_detection::CollisionResult::ContactMap contacts;
    scene->getCollidingPairs(contacts, state, group);
    if (!contacts.empty())
    {
      std::cerr << "Recorded moving group starts in collision: " << item.at("name") << '\n';
      return 3;
    }
    waypoint = moveit_msgs::msg::RobotState{};
    for (const auto& [name, value] : item.at("waypoint").items())
    {
      waypoint.joint_state.name.push_back(name);
      waypoint.joint_state.position.push_back(value.get<double>());
    }
    auto board = BT::Blackboard::create();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    const auto& xyz = item.at("approach").at("xyz");
    const auto& quat = item.at("approach").at("wxyz");
    pose.pose.position.x = xyz[0];
    pose.pose.position.y = xyz[1];
    pose.pose.position.z = xyz[2];
    pose.pose.orientation.w = quat[0];
    pose.pose.orientation.x = quat[1];
    pose.pose.orientation.y = quat[2];
    pose.pose.orientation.z = quat[3];
    board->set("approach_pose", pose);
    board->set("planning_group", group);
    board->set("hand_frame", arm + "_fr3v2_hand_tcp");
    board->set("place_waypoint", "fixture");
    board->set("corner_y", arm == "right" ? -.4 : .4);
    board->set("carry_speed", .25);
    board->set("carry_acceleration", .0625);
    auto tree = factory.createTree("Plan Block Transfer", board);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    BT::NodeStatus status;
    do
    {
      status = tree.tickExactlyOnce();
      if (status != BT::NodeStatus::RUNNING)
      {
        break;
      }
      tree.sleep(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    Json result{ { "name", item.at("name") }, { "status", BT::toStr(status) } };
    Solution pickup;
    Solution carry;
    if (board->get("pickup_solution", pickup))
    {
      result["pickup"] = trajectories(pickup);
    }
    if (board->get("carry_solution", carry))
    {
      result["carry"] = trajectories(carry);
    }
    if (status != BT::NodeStatus::SUCCESS)
    {
      result["error"] = context->logger->consumeErrorLogBuffer();
      tree.haltTree();
    }
    std::cout << result.at("name") << ": " << BT::toStr(status) << '\n';
    results.push_back(result);
  }
  std::ofstream(argv[3]) << results.dump(2);
  factory.clearRegisteredBehaviorTrees();
  rclcpp::shutdown();
  return 0;
}
