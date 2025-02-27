#include <example_behaviors/T_pushing.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision/utils/tf_utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
// Port names for input and output ports.
constexpr auto kPortIDTwistStamped = "twist_stamped";
constexpr auto kPortIDWrenchStamped = "wrench_stamped";
}  // namespace

namespace example_behaviors
{
T_pushing::T_pushing(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
  marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_markers", rclcpp::SystemDefaultsQoS());
}

BT::PortsList T_pushing::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList({
      BT::OutputPort<geometry_msgs::msg::TwistStamped>(kPortIDTwistStamped, "{desired_twist}",
                                                       "Desired twist in the velocity controlled axes."),
      BT::OutputPort<geometry_msgs::msg::WrenchStamped>(kPortIDWrenchStamped, "{desired_wrench}", "mcok."),
  });
}

BT::KeyValueVector T_pushing::metadata()
{
  return { { "subcategory", "Example Behaviors" }, { "description", "Push the T" } };
}

tl::expected<bool, std::string> T_pushing::doWork()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs();

  // If a port was set incorrectly, log an error message and return FAILURE
  if (!ports.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get required value from input data port: {}", ports.error()));
  }
  // const auto& [] = ports.value();

  std::vector<Eigen::Isometry3d> site_transforms;
  Eigen::Isometry3d ee_world_transform;
  Eigen::Isometry3d com_world_transform;
  Eigen::Isometry3d target_world_transform;
  try
  {
    com_world_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("world", "com", tf2::TimePointZero));
    ee_world_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("world", "grasp_link", tf2::TimePointZero));
    target_world_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("world", "target", tf2::TimePointZero));

    for (int i = 1; i <= 8; ++i)
    {
      auto tf_msg = shared_resources_->transform_buffer_ptr->lookupTransform("world", fmt::format("point_{}", i),
                                                                             tf2::TimePointZero);
      site_transforms.push_back(tf2::transformToEigen(tf_msg));
    }
  }
  catch (tf2::TransformException& ex)
  {
    return tl::make_unexpected(ex.what());
  }

  Eigen::Vector3d velocity;
  velocity << 0, 0, 0.02;
  velocity = ee_world_transform.linear().inverse() * velocity;

  // model
  // (P - COM)*w + xd = V_robot
  // [(P - COM)  1 ][w; xd] = [V_robot_x; V_robot_y]
  // A = [(P - COM)  1 ]
  // b = [V_robot_x; V_robot_y]
  // min ||Ax - b|| = ||[V_robot_x; V_robot_y]||

  geometry_msgs::msg::TwistStamped msg;
  msg.header.frame_id = "world";
  msg.header.stamp = shared_resources_->node->now();
  msg.twist.linear.x = velocity[0];
  msg.twist.linear.y = velocity[1];
  msg.twist.linear.z = velocity[2];

  setOutput(kPortIDTwistStamped, msg);
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header = msg.header;

  setOutput(kPortIDWrenchStamped, wrench_msg);

  return true;
}
}  // namespace example_behaviors
