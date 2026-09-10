// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/localization_gates.hpp>
#include <hangar_sim_behaviors/seed_localization_at_pose.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/behavior_subcategories.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/utils.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortXyStdDev = "xy_std_dev";
constexpr auto kPortYawStdDev = "yaw_std_dev";
constexpr auto kPortTopic = "initial_pose_topic";
constexpr auto kPortSubscriberTimeout = "subscriber_timeout";

constexpr auto kDefaultTopic = "/initialpose";
constexpr auto kMapFrame = "map";
constexpr double kDefaultSubscriberTimeout = 5.0;

/// Indices of the x, y and yaw variances on a 6x6 row-major covariance.
constexpr std::size_t kCovarianceXX = 0;
constexpr std::size_t kCovarianceYY = 7;
constexpr std::size_t kCovarianceYawYaw = 35;

inline constexpr auto kDescription = R"(
                <p>Re-seeds the particle filter at a pose you supply, with a stated spread, by publishing a <code>PoseWithCovarianceStamped</code> on the localizer's initial-pose topic.</p>
                <p>Core's <code>SetInitialPose</code> seeds from TF -- from the estimate the filter already holds -- so it can tighten a cloud but never move it. Use this one when the pose comes from somewhere else: an operator correcting the robot's position on the map, or putting that correction back after a refinement is rejected.</p>
                <p>The spread is not a claim about accuracy. It sets the resolution of anything that refines from here, and it bounds how far such a refinement can travel from the seed. Widening <code>yaw_std_dev</code> is how a filter converges facing the wrong way.</p>
                <p>The pose is projected to the plane -- x, y and yaw are used, z, roll and pitch are discarded -- because the localizer is 2D. A pose that is not already in the map frame is refused rather than reinterpreted.</p>
                <p>Exits with FAILURE if the pose is in the wrong frame, if the spreads are not positive, or if no subscriber appears on the topic before <code>subscriber_timeout</code>. That last check matters: a message published into a topic nobody has subscribed to yet is simply lost, and the seed would silently never be applied.</p>
            )";
}  // namespace

namespace hangar_sim_behaviors
{
SeedLocalizationAtPose::SeedLocalizationAtPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList SeedLocalizationAtPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "{seed_pose}",
                                                   "Pose to seed the filter at. Must be in the map frame."),
    BT::InputPort<double>(kPortXyStdDev, localization::kSeedXyStdDev,
                          "Radius of the particle scatter, in metres. Sets both the resolution of a subsequent "
                          "refinement and how far that refinement can land from this pose."),
    BT::InputPort<double>(kPortYawStdDev, localization::kSeedYawStdDev,
                          "Heading spread of the scatter, in radians (0.26 = 15 degrees). Do not widen it: "
                          "heading is the axis a click is worst at and a wide yaw cloud is how localization "
                          "converges facing the wrong way."),
    BT::InputPort<std::string>(kPortTopic, kDefaultTopic, "Topic the localizer takes initial poses on."),
    BT::InputPort<double>(kPortSubscriberTimeout, kDefaultSubscriberTimeout,
                          "Seconds to wait for the localizer to subscribe before giving up. A seed published "
                          "with no subscriber is lost silently."),
  };
}

BT::KeyValueVector SeedLocalizationAtPose::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey,
             std::string(moveit_pro::behaviors::toString(moveit_pro::behaviors::Subcategory::Navigation)) },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus SeedLocalizationAtPose::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortPose), getInput<double>(kPortXyStdDev),
      getInput<double>(kPortYawStdDev), getInput<std::string>(kPortTopic), getInput<double>(kPortSubscriberTimeout));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "SeedLocalizationAtPose: missing required input: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose, xy_std_dev, yaw_std_dev, topic, subscriber_timeout] = ports.value();

  if (pose.header.frame_id != kMapFrame)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("SeedLocalizationAtPose: pose is in frame '{}', but the localizer seeds in '{}'. "
                            "Transform it first rather than letting it be reinterpreted.",
                            pose.header.frame_id, kMapFrame));
    return BT::NodeStatus::FAILURE;
  }
  if (xy_std_dev <= 0.0 || yaw_std_dev <= 0.0)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("SeedLocalizationAtPose: both spreads must be positive, got xy_std_dev {:.3f} and "
                            "yaw_std_dev {:.3f}. A zero spread gives the filter nothing to select from.",
                            xy_std_dev, yaw_std_dev));
    return BT::NodeStatus::FAILURE;
  }

  const auto node = getBehaviorContext()->node;
  // Held in the context's cache rather than created per tick: a publisher destroyed immediately
  // after publishing can take the sample with it before the middleware has delivered it.
  auto publisher_base = getBehaviorContext()->persistent_publishers.getOrCreate(topic, [&node, &topic]() {
    return std::static_pointer_cast<rclcpp::PublisherBase>(
        node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, rclcpp::QoS(1).reliable()));
  });
  auto publisher =
      std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>>(publisher_base);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(subscriber_timeout);
  while (publisher->get_subscription_count() == 0)
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), fmt::format("SeedLocalizationAtPose: nothing subscribed to '{}' within {:.1f} s, so the seed "
                              "would have been dropped. Is the localizer running and activated?",
                              topic, subscriber_timeout));
      return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  geometry_msgs::msg::PoseWithCovarianceStamped seed;
  seed.header.frame_id = kMapFrame;
  seed.header.stamp = node->now();
  seed.pose.pose.position.x = pose.pose.position.x;
  seed.pose.pose.position.y = pose.pose.position.y;
  seed.pose.pose.position.z = 0.0;

  // Project to the plane. An operator's click carries whatever roll and pitch the click surface had;
  // feeding that through would leave the filter a quaternion whose yaw is not the yaw that was meant.
  const double yaw = tf2::getYaw(pose.pose.orientation);
  seed.pose.pose.orientation.x = 0.0;
  seed.pose.pose.orientation.y = 0.0;
  seed.pose.pose.orientation.z = std::sin(yaw * 0.5);
  seed.pose.pose.orientation.w = std::cos(yaw * 0.5);

  seed.pose.covariance[kCovarianceXX] = xy_std_dev * xy_std_dev;
  seed.pose.covariance[kCovarianceYY] = xy_std_dev * xy_std_dev;
  seed.pose.covariance[kCovarianceYawYaw] = yaw_std_dev * yaw_std_dev;

  publisher->publish(seed);

  getBehaviorContext()->logger->publishInfoMessage(
      name(), fmt::format("Seeded localization at ({:.3f}, {:.3f}) yaw {:.1f} deg, spread {:.2f} m / {:.1f} deg.",
                          seed.pose.pose.position.x, seed.pose.pose.position.y, yaw * 180.0 / M_PI, xy_std_dev,
                          yaw_std_dev * 180.0 / M_PI));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hangar_sim_behaviors
