#include <pi0_behaviors/get_pi0_trajectory.hpp>

namespace pi0_behaviors
{

GetPi0Trajectory::GetPi0Trajectory(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetPi0Trajectory::providedPorts()
{
  return { BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg") };
}

BT::NodeStatus GetPi0Trajectory::onStart()
{
  latest_ = nullptr;

  // Subscribe once — keep subscription alive while behavior is running.
  // BEST_EFFORT matches the bridge publisher; KEEP_LAST(1) always gets the freshest.
  auto qos = rclcpp::QoS(1).best_effort().keep_last(1);
  sub_ = shared_resources_->node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/pi0_trajectory", qos,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        latest_ = msg;
      });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetPi0Trajectory::onRunning()
{
  if (!latest_)
  {
    return BT::NodeStatus::RUNNING;
  }

  setOutput("joint_trajectory_msg", *latest_);
  latest_ = nullptr;
  return BT::NodeStatus::SUCCESS;
}

void GetPi0Trajectory::onHalted()
{
  sub_.reset();
  latest_ = nullptr;
}

}  // namespace pi0_behaviors
