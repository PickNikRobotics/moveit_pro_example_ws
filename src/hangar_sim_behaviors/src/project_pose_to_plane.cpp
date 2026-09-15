// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/planar_pose.hpp>
#include <hangar_sim_behaviors/project_pose_to_plane.hpp>

#include <moveit_pro_behavior_interface/behavior_subcategories.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortPlanarPose = "planar_pose";

inline constexpr auto kDescription = R"(
                <p>Flattens a pose into the plane a 2D localizer works in: <code>x</code>, <code>y</code> and yaw are kept, and <code>z</code>, roll and pitch are set to zero. The frame is passed through unchanged.</p>
                <p>Use it before measuring a distance against a pose that was seeded with <code>SeedLocalizationAtPose</code>, which flattens the same way. Comparing an unflattened pose against a 2D estimate measures the tilt and height of whatever surface the pose came off, not the thing the comparison is meant to bound.</p>
            )";
}  // namespace

namespace hangar_sim_behaviors
{
ProjectPoseToPlane::ProjectPoseToPlane(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ProjectPoseToPlane::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "{pose}", "Pose to flatten."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortPlanarPose, "{planar_pose}",
                                                    "The same pose with z, roll and pitch removed, in the same "
                                                    "frame."),
  };
}

BT::KeyValueVector ProjectPoseToPlane::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey,
             std::string(moveit_pro::behaviors::toString(moveit_pro::behaviors::Subcategory::Navigation)) },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus ProjectPoseToPlane::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortPose));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "ProjectPoseToPlane: missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose] = ports.value();

  geometry_msgs::msg::PoseStamped planar;
  planar.header = pose.header;
  planar.pose = localization::projectToPlane(pose.pose);

  setOutput(kPortPlanarPose, planar);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hangar_sim_behaviors
