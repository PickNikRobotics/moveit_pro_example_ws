#include <example_behaviors/example_convert_mtc_solution_to_joint_trajectory.hpp>

namespace
{
const auto kLogger = rclcpp::get_logger("ExampleConvertMtcSolutionToJointTrajectory");

// Port names for input and output ports.
constexpr auto kPortIDSolution = "solution";
constexpr auto kPortIDJointGroup = "joint_group";
constexpr auto kPortIDJointTrajectory = "joint_trajectory";
constexpr auto kPortIDVelocityScalingFactor = "velocity_scaling_factor";
constexpr auto kPortIDAccelerationScalingFactor = "acceleration_scaling_factor";
constexpr auto kPortIDSamplingRate = "sampling_rate";
}  // namespace

namespace example_behaviors
{
ExampleConvertMtcSolutionToJointTrajectory::ExampleConvertMtcSolutionToJointTrajectory(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ExampleConvertMtcSolutionToJointTrajectory::providedPorts()
{
  return {
    BT::InputPort<moveit_task_constructor_msgs::msg::Solution>(kPortIDSolution, "{mtc_solution}",
                                                               "MoveIt Task Constructor solution."),
    BT::InputPort<std::string>(kPortIDJointGroup, "manipulator", "Joint group name used in the MTC solution."),
    BT::InputPort<double>(kPortIDVelocityScalingFactor, 1.0, "Velocity scaling factor for trajectory generation."),
    BT::InputPort<double>(kPortIDAccelerationScalingFactor, 1.0,
                          "Acceleration scaling factor for trajectory generation."),
    BT::InputPort<int>(kPortIDSamplingRate, 100, "Sampling rate for trajectory generation."),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory, "{joint_trajectory_msg}",
                                                          "Resulting joint trajectory."),
  };
}

BT::KeyValueVector ExampleConvertMtcSolutionToJointTrajectory::metadata()
{
  return { { "subcategory", "Motion Planning" },
           { "description",
             "Extracts joint space trajectories from an MTC solution and adds time parameterization using TOTG." } };
}

BT::NodeStatus ExampleConvertMtcSolutionToJointTrajectory::tick()
{
  using namespace moveit_pro::behaviors;

  // Load data from the behavior input ports.
  const auto solution = getInput<moveit_task_constructor_msgs::msg::Solution>(kPortIDSolution);
  const auto joint_group_name = getInput<std::string>(kPortIDJointGroup);
  const auto velocity_scaling_factor = getInput<double>(kPortIDVelocityScalingFactor);
  const auto acceleration_scaling_factor = getInput<double>(kPortIDAccelerationScalingFactor);
  const auto sampling_rate = getInput<int>(kPortIDSamplingRate);

  // Check that the required input data port was set
  if (const auto error =
          maybe_error(solution, joint_group_name, velocity_scaling_factor, acceleration_scaling_factor, sampling_rate);
      error)
  {
    spdlog::error("Failed to get required value from input data port: {}", error.value());
    return BT::NodeStatus::FAILURE;
  }
  // Initialize the robot model loader if this is the first time this Behavior has been run.
  if (robot_model_loader_ == nullptr)
  {
    robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(shared_resources_->node);
  }

  // Get the robot model from the robot model loader.
  const auto robot_model = robot_model_loader_->getModel();
  if (robot_model == nullptr)
  {
    // Return as a failure case if the robot model loader has no robot model.
    spdlog::error("Failed to load robot model.");
    return BT::NodeStatus::FAILURE;
  }

  // Get the JointModelGroup using the joint group name from the input port
  auto joint_model_group = robot_model->getJointModelGroup(joint_group_name.value());
  if (!joint_model_group)
  {
    spdlog::error("Failed to get JointModelGroup '{}'.", joint_group_name.value());
    return BT::NodeStatus::FAILURE;
  }

  // Extract joint positions from the MTC solution
  const auto& waypoints = extractJointPositions(solution.value());

  // Use trajectory_utils.hpp to create a trajectory
  auto trajectory_result =
      cartesian_planning::createTrajectoryFromWaypoints(*joint_model_group, waypoints, velocity_scaling_factor.value(),
                                                        acceleration_scaling_factor.value(), sampling_rate.value());

  if (!trajectory_result)
  {
    spdlog::error("Trajectory generation failed: {}", trajectory_result.error());
    return BT::NodeStatus::FAILURE;
  }

  // Set the output port with the resulting joint trajectory
  setOutput(kPortIDJointTrajectory, trajectory_result.value());

  spdlog::info("Successfully converted MTC Solution to JointTrajectory with {} points.",
               trajectory_result.value().points.size());

  return BT::NodeStatus::SUCCESS;
}

const std::vector<Eigen::VectorXd>& ExampleConvertMtcSolutionToJointTrajectory::extractJointPositions(
    const moveit_task_constructor_msgs::msg::Solution& solution)
{
  static std::vector<Eigen::VectorXd> waypoints;
  waypoints.clear();
  for (size_t sub_traj_idx = 0; sub_traj_idx < solution.sub_trajectory.size(); ++sub_traj_idx)
  {
    const auto& sub_traj = solution.sub_trajectory[sub_traj_idx];
    const auto& joint_traj = sub_traj.trajectory.joint_trajectory;

    if (joint_traj.points.empty())
    {
      spdlog::warn("Sub-trajectory {} has no points.", sub_traj_idx);
      continue;
    }

    spdlog::info("Processing sub-trajectory {} with {} points.", sub_traj_idx, joint_traj.points.size());

    for (const auto& point : joint_traj.points)
    {
      Eigen::VectorXd positions(point.positions.size());
      for (size_t i = 0; i < point.positions.size(); ++i)
      {
        positions[i] = point.positions[i];
      }
      waypoints.push_back(positions);
    }
  }
  return waypoints;
}

}  // namespace example_behaviors
