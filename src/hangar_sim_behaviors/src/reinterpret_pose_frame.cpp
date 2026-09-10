// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/reinterpret_pose_frame.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/behavior_subcategories.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace
{
constexpr auto kPortInputPose = "input_pose";
constexpr auto kPortFrameId = "frame_id";
constexpr auto kPortOutputPose = "output_pose";

inline constexpr auto kDescription = R"(
                <p>Relabels a pose's frame WITHOUT transforming it. The coordinates are copied unchanged and only <code>header.frame_id</code> is replaced.</p>
                <p>This is not <code>TransformPoseFrame</code>. There is no TF lookup, so nothing moves. It asserts that the numbers already mean the same thing in the target frame, which is only true when the two frames are related by an identity transform. Whoever uses it owes an explanation at the call site of why that holds.</p>
                <p>It exists for the case where the transform cannot be trusted rather than the case where it is inconvenient: a pose that has to reach a localizer's global frame before that localizer is seeded cannot be transformed into it, because the edge that would carry the transform is what the seed creates. Relabeling in the open beats transforming through an estimate that is not yet a measurement.</p>
                <p>Exits with FAILURE if the target frame is empty. Prefer <code>TransformPoseFrame</code> whenever the transform exists and means something.</p>
            )";
}  // namespace

namespace hangar_sim_behaviors
{
ReinterpretPoseFrame::ReinterpretPoseFrame(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ReinterpretPoseFrame::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortInputPose, "{input_pose}",
                                                   "Pose whose coordinates are already correct in the target "
                                                   "frame."),
    BT::InputPort<std::string>(kPortFrameId, "map",
                               "Frame to relabel the pose into. No transform is applied, so this must be "
                               "related to the pose's current frame by an identity."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortOutputPose, "{output_pose}",
                                                    "The same coordinates, carrying the new frame."),
  };
}

BT::KeyValueVector ReinterpretPoseFrame::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey,
             std::string(moveit_pro::behaviors::toString(moveit_pro::behaviors::Subcategory::Navigation)) },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus ReinterpretPoseFrame::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortInputPose),
                                                              getInput<std::string>(kPortFrameId));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "ReinterpretPoseFrame: missing required input: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [input_pose, frame_id] = ports.value();

  if (frame_id.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "ReinterpretPoseFrame: the target frame is empty. A pose with no frame is not a pose.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped output_pose = input_pose;
  output_pose.header.frame_id = frame_id;
  setOutput(kPortOutputPose, output_pose);

  // Say it out loud. A relabel that is wrong looks exactly like a relabel that is right, so the one
  // thing this Behavior can do for whoever reads the log afterwards is name both frames.
  getBehaviorContext()->logger->publishInfoMessage(
      name(),
      fmt::format("Reinterpreted a pose at ({:.3f}, {:.3f}) from frame '{}' as frame '{}' without "
                  "transforming it.",
                  output_pose.pose.position.x, output_pose.pose.position.y, input_pose.header.frame_id, frame_id));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hangar_sim_behaviors
