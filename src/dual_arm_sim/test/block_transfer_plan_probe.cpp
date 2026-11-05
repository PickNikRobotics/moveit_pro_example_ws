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

// Offline companion for test_block_transfer.py. Only local fixture services; no robot execution.
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro/task_constructor/stages/fixed_state.h>
#include <moveit_pro/task_constructor/task.h>
#include <tinyxml2.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>
#include <moveit_pro_base/robot_model_loader/robot_model_loader.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_studio_agent_msgs/srv/get_waypoints.hpp>
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
  const auto shutdown = std::shared_ptr<void>(nullptr, [](void*) { rclcpp::shutdown(); });
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
  const bool targets_only = argc > 4 && std::string(argv[4]) == "--targets-only";
  using GetWaypoints = moveit_studio_agent_msgs::srv::GetWaypoints;
  moveit_studio_agent_msgs::msg::Waypoint waypoint;
  std::mutex waypoint_mutex;
  const auto fixture_node = std::make_shared<rclcpp::Node>("block_transfer_waypoints_fixture");
  const auto service = fixture_node->create_service<GetWaypoints>(
      "get_saved_waypoints",
      [&](const GetWaypoints::Request::SharedPtr, const GetWaypoints::Response::SharedPtr response) {
        const auto lock = std::scoped_lock(waypoint_mutex);
        response->status.success = true;
        response->waypoints = { waypoint };
      });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fixture_node);
  std::thread spin([&executor] { executor.spin(); });
  const auto stop_executor = std::shared_ptr<void>(nullptr, [&executor, &spin](void*) {
    executor.cancel();
    spin.join();
  });
  // Scene acquisition is local; waypoint filtering, pose math, planners and solution splitting are real.
  std::ignore = factory.unregisterBuilder("GetCurrentPlanningScene");
  factory.registerSimpleAction("GetCurrentPlanningScene",
                               [&](BT::TreeNode& behavior) {
                                 moveit_msgs::msg::PlanningScene message;
                                 scene->getPlanningSceneMsg(message);
                                 behavior.setOutput("planning_scene_msg", message);
                                 return BT::NodeStatus::SUCCESS;
                               },
                               { BT::OutputPort<moveit_msgs::msg::PlanningScene>("planning_scene_msg") });
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
  if (targets_only)
  {
    auto* sequence = xml.RootElement()->FirstChildElement("BehaviorTree")->FirstChildElement();
    auto* child = sequence->FirstChildElement()->NextSiblingElement();
    while (child)
    {
      auto* next = child->NextSiblingElement();
      sequence->DeleteChild(child);
      child = next;
    }
  }
  tinyxml2::XMLPrinter printer;
  xml.Print(&printer);
  factory.registerBehaviorTreeFromText(printer.CStr());
  if (targets_only)
  {
    factory.registerSimpleAction("ApplyJointState",
                                 [&](BT::TreeNode& behavior) {
                                   const auto target =
                                       behavior.getInput<moveit_msgs::msg::RobotState>("target_joint_state").value();
                                   scene->getCurrentStateNonConst().setVariableValues(target.joint_state);
                                   scene->getCurrentStateNonConst().update();
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 { BT::InputPort<moveit_msgs::msg::RobotState>("target_joint_state") });
    const auto core = ament_index_cpp::get_package_share_directory("moveit_pro_objectives") + "/objectives/motion/";
    tinyxml2::XMLDocument motion;
    if (motion.LoadFile((core + "move_to_joint_state.xml").c_str()) != tinyxml2::XML_SUCCESS)
    {
      return 2;
    }
    auto* body = motion.RootElement()->FirstChildElement("BehaviorTree");
    body->DeleteChildren();
    auto* apply = motion.NewElement("ApplyJointState");
    apply->SetAttribute("target_joint_state", "{target_joint_state}");
    body->InsertEndChild(apply);
    tinyxml2::XMLPrinter motion_printer;
    motion.Print(&motion_printer);
    factory.registerBehaviorTreeFromText(motion_printer.CStr());
    factory.registerBehaviorTreeFromFile(core + "move_to_named_waypoint.xml");
    factory.registerBehaviorTreeFromFile(core + "move_to_waypoint.xml");
    tinyxml2::XMLDocument sort;
    if (argc < 6 || sort.LoadFile(argv[5]) != tinyxml2::XML_SUCCESS)
    {
      return 2;
    }
    // Execute initialization up to the perception loop; completed motion is the only substituted boundary.
    auto* sequence = sort.RootElement()->FirstChildElement("BehaviorTree")->FirstChildElement();
    auto* child = sequence->FirstChildElement("Decorator");
    while (child)
    {
      auto* next = child->NextSiblingElement();
      sequence->DeleteChild(child);
      child = next;
    }
    tinyxml2::XMLPrinter sort_printer;
    sort.Print(&sort_printer);
    factory.registerBehaviorTreeFromText(sort_printer.CStr());
  }
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
    const auto serve = [&](const Json& values, const std::string& name, const std::vector<std::string>& groups) {
      const auto lock = std::scoped_lock(waypoint_mutex);
      waypoint = moveit_studio_agent_msgs::msg::Waypoint{};
      waypoint.name = name;
      waypoint.joint_group_names = groups;
      for (const auto& [joint, value] : values.items())
      {
        waypoint.joint_state.name.push_back(joint);
        waypoint.joint_state.position.push_back(value.get<double>());
      }
    };
    const auto run = [](BT::Tree& tree) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
      {
        status = tree.tickExactlyOnce();
        tree.sleep(std::chrono::milliseconds(1));
      }
      return status;
    };
    if (item.value("initialize_sort", false))
    {
      serve(item.at("home"), "Home", { "manipulator" });
      auto initialization = factory.createTree("Sort Blocks");
      if (run(initialization) != BT::NodeStatus::SUCCESS)
      {
        std::cerr << "Sort initialization failed: " << context->logger->consumeErrorLogBuffer();
        return 3;
      }
    }
    serve(item.at("waypoint"), "fixture", item.value("waypoint_groups", std::vector<std::string>{ group }));
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
    const auto status = run(tree);
    Json result{ { "name", item.at("name") }, { "status", BT::toStr(status) } };
    if (targets_only)
    {
      geometry_msgs::msg::PoseStamped release;
      if (board->get("release_pose", release))
      {
        result["release_xyz"] = { release.pose.position.x, release.pose.position.y, release.pose.position.z };
      }
      moveit_msgs::msg::RobotState retrieved;
      if (board->get("place_state", retrieved))
      {
        result["retrieved_joints"] = retrieved.joint_state.name;
      }
      result["initial_spine"] = state.getVariablePosition("franka_spine_vertical_joint");
      result["missing_group_joints"] = Json::object();
      const auto lock = std::scoped_lock(waypoint_mutex);
      for (const auto& name : waypoint.joint_group_names)
      {
        auto missing = std::vector<std::string>{};
        for (const auto& joint : context->robot_model->getJointModelGroup(name)->getActiveJointModelNames())
        {
          if (!item.at("waypoint").contains(joint))
          {
            missing.push_back(joint);
          }
        }
        result["missing_group_joints"][name] = missing;
      }
    }
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
  return 0;
}
