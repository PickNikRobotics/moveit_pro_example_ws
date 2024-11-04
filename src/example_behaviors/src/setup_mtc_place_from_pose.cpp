#include <example_behaviors/setup_mtc_place_from_pose.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_common/utils/yaml_parsing_tools.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
const auto kLogger = rclcpp::get_logger("SetupMTCPickFromPose");
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

// Port names for input and output ports.
constexpr auto kPortIDTask = "task";
constexpr auto kPortIDGraspPose = "place_pose";

// Behavior constants
constexpr auto kWorldFrame = "world";
constexpr auto kArmGroupName = "manipulator";
constexpr auto kEndEffectorGroupName = "gripper";
constexpr auto kEndEffectorName = "moveit_ee";
constexpr auto kHandFrameName = "grasp_link";
constexpr auto kHandOpenName = "open";
constexpr auto kHandCloseName = "close";
constexpr auto kApproachDistance = 0.1;
constexpr auto kPropertyNameTrajectoryExecutionInfo = "trajectory_execution_info";
constexpr auto kIKTimeoutSeconds = 1.0;
constexpr auto kMaxIKSolutions = 20;
constexpr auto kSceneObjectNameOctomap = "<octomap>";
}  // namespace

namespace example_behaviors
{
SetupMtcPlaceFromPose::SetupMtcPlaceFromPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetupMtcPlaceFromPose::providedPorts()
{
  return {
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortIDTask, "{mtc_task}", "MoveIt Task Constructor task."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose, "{place_pose}",
                                                   "The stamped pose message used in this MTC stage."),
  };
}

BT::KeyValueVector SetupMtcPlaceFromPose::metadata()
{
  return { { "subcategory", "Example" },
           { "description", "Adds the stages to describe a place motion to the MTC task." } };
}

BT::NodeStatus SetupMtcPlaceFromPose::tick()
{
  using namespace moveit_studio::behaviors;

  // ----------------------------------------
  // Load data from the behavior input ports.
  // ----------------------------------------
  const auto place_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose);
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);

  // Check that all required input data ports were set
  if (const auto error = maybe_error(place_pose, task); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // Create planners
  const auto mtc_pipeline_planner =
      std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_resources_->node);
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Place From Pose");
  container->properties().set(kPropertyNameTrajectoryExecutionInfo,
                              boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                  task.value()->properties().get(kPropertyNameTrajectoryExecutionInfo)));
  container->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

  container->setProperty("group", kArmGroupName);
  container->setProperty("hand", kEndEffectorGroupName);
  container->setProperty("eef", kEndEffectorName);
  container->setProperty("ik_frame", kHandFrameName);

  /** Allow Collision since the hand holds a collision object */
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision 2 (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(kEndEffectorGroupName)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->insert(std::move(stage));
  }

  /** Move To Pre-Place Pose **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
        "Move to Pre-Approach Pose",
        moveit::task_constructor::stages::Connect::GroupPlannerVector{ { kArmGroupName, mtc_pipeline_planner } });
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

    stage->properties().set(kPropertyNameTrajectoryExecutionInfo,
                            boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                container->properties().get(kPropertyNameTrajectoryExecutionInfo)));
    stage->setTimeout(1.0);
    container->add(std::move(stage));
  }

  const Eigen::Vector3d approach_vector{ 0.0, 0.0, kApproachDistance };

  /** Approach Place Pose **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", mtc_cartesian_planner);
    stage->restrictDirection(moveit::task_constructor::stages::MoveRelative::BACKWARD);
    stage->setGroup(kArmGroupName);
    stage->setIKFrame(kHandFrameName);

    geometry_msgs::msg::Vector3Stamped approach_vector_msg;
    tf2::toMsg(approach_vector, approach_vector_msg.vector);
    approach_vector_msg.header.frame_id = kHandFrameName;

    stage->setDirection(approach_vector_msg);
    stage->setTimeout(10);
    container->add(std::move(stage));
  }

  /** Generate the Inverse Kinematics (IK) solutions to move to the pose specified in the "place_pose" input port.
      This will generate up to kMaxIKSolutions IK solution candidates to sample from, unless the timeout specified in
      kIKTimeoutSeconds is reached first.
      Collision checking is ignored for IK pose generation. Solutions that result in forbidden collisions will be
      eliminated by failures in the stages before and after this one. **/
  {
    // Specify pose to generate for
    auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("Generate pose");
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_frame");
    stage->setPose(place_pose.value());
    stage->setMonitoredStage(task.value()->stages()->findChild("current state"));  // Hook into current state

    // Compute IK
    auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("Pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(kMaxIKSolutions);
    wrapper->setTimeout(kIKTimeoutSeconds);
    wrapper->setIKFrame(kHandFrameName);
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
    wrapper->setIgnoreCollisions(true);
    container->add(std::move(wrapper));
  }

  /** Allow Collision
      This stage allows collisions between the gripper and object for stages after this one (during the Close Hand,
      Lift, and Retreat stages). */
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(kEndEffectorGroupName)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    container->add(std::move(stage));
  }

  /** Open Hand **/
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveTo>("Open hand", mtc_joint_interpolation_planner);
    stage->setGroup(kEndEffectorGroupName);
    stage->setGoal(kHandOpenName);
    container->add(std::move(stage));
  }

  /** Forbid Collision
      This stage forbids collisions between the gripper and the object for subsequent stages. **/
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Forbid collision (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(kEndEffectorGroupName)
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->add(std::move(stage));
  }

  /** Retreat **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Retreat", mtc_cartesian_planner);
    stage->setGroup(kArmGroupName);
    stage->setIKFrame(kHandFrameName);

    geometry_msgs::msg::Vector3Stamped retreat_vector_msg;
    tf2::toMsg(approach_vector * -1, retreat_vector_msg.vector);
    retreat_vector_msg.header.frame_id = kHandFrameName;

    stage->setDirection(retreat_vector_msg);
    container->add(std::move(stage));
  }

  task.value()->add(std::move(container));

  return BT::NodeStatus::SUCCESS;
}
}  // namespace example_behaviors
