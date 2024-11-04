#include <example_behaviors/setup_mtc_wave_hand.hpp>

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>

namespace
{
// Define a constant for the ID of the Behavior's data port.
constexpr auto kPortIDTask = "task";

// Define constants for the names of the behavior parameters used by the SetupMTCWaveHand behavior.
// These defaults are set to match the UR5e robot defined in the moveit_studio_custom_site_config_example package.
constexpr auto kPrimaryGroupName = "manipulator";
constexpr auto kHandFrameName = "grasp_link";

// Create a rclcpp Logger to use when printing messages to the console.
const rclcpp::Logger kLogger = rclcpp::get_logger("WaveHello");
}  // namespace

namespace example_behaviors
{
SetupMTCWaveHand::SetupMTCWaveHand(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SetupMTCWaveHand::providedPorts()
{
  return {
    BT::BidirectionalPort<std::shared_ptr<moveit::task_constructor::Task>>(kPortIDTask, "{mtc_task}",
                                                                           "MoveIt Task Constructor task."),
  };
}

BT::KeyValueVector SetupMTCWaveHand::metadata()
{
  return { { "subcategory", "Example" }, { "description", "Wave hello with the end effector." } };
}

tl::expected<bool, std::string> SetupMTCWaveHand::doWork()
{
  // Retrieve the "task" data port
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(task); error)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Failed to get required values from input data ports:\n" << error.value());
    // Task setup cannot succeed if we failed to retrieve the MTC task shared_ptr from the "task" port, so we return false.
    return tl::make_unexpected("Failed to get required values from input data ports: " + error.value());
  }

  // Create a Cartesian path planner used to perform linear moves relative to the end effector frame.
  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

  // Create an MTC stage to define a motion that translates along the end effector X+ axis. Assumes that the robot is
  // not already at its joint limits.

  // Set the direction to move along as a TwistStamped.
  geometry_msgs::msg::TwistStamped wave_first_direction;
  wave_first_direction.header.frame_id = kHandFrameName;
  wave_first_direction.twist.linear.x = 1.0;

  // Create a new MTC stage
  {
    // Create a new MoveRelative stage that uses the Cartesian motion planner
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(std::string("Wave One Direction"),
                                                                                  cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    // Configure the stage to move the UR-5e's arm.
    stage->setGroup(kPrimaryGroupName);
    // Configure the stage to move relative to the UR-5e's gripper frame.
    stage->setIKFrame(kHandFrameName);
    // Configure the stage to move along the direction specified by the TwistStamped.
    stage->setDirection(wave_first_direction);
    // Set that we will move at most 0.2m along the vector.
    stage->setMaxDistance(0.2);
    // Add the stage to the MTC task which was retrieved from the "task" data port.
    task.value()->add(std::move(stage));
  }

  // Create a second MTC stage to define a motion that translates along the end effector X- axis, which is the opposite
  // motion from what we did in the previous stage.

  // Set the direction to move along as a TwistStamped.
  geometry_msgs::msg::TwistStamped wave_second_direction;
  wave_second_direction.header.frame_id = kHandFrameName;
  wave_second_direction.twist.linear.x = -1.0;

  {
    // Create a new MoveRelative stage that uses the Cartesian motion planner
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        std::string("Wave Opposite Direction"), cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    // Configure the stage to move the UR-5e's arm.
    stage->setGroup(kPrimaryGroupName);
    // Configure the stage to move relative to the UR-5e's gripper frame.
    stage->setIKFrame(kHandFrameName);
    // Configure the stage to move along the direction specified by the TwistStamped.
    stage->setDirection(wave_second_direction);
    // Set that we will move at most 0.2m along the vector.
    stage->setMaxDistance(0.2);
    // Add the stage to the MTC task which was retrieved from the "task" data port.
    task.value()->add(std::move(stage));
  }

  // Once the work is done, we can return true so that the node returns SUCCESS next time it is ticked
  return true;
}

}  // namespace example_behaviors
