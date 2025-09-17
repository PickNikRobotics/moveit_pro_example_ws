// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_filenames_from_directory/push_back_vector.hpp>

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_behavior_interface/metadata_fields.hpp"

#include <fmt/format.h>

namespace
{
inline constexpr auto kDescriptionPushBackVector = R"(
    <p>
        Push an element to the back of a vector.
    </p>
)";

// Port names for input and output ports.
constexpr auto kPortIDInputVector = "input_vector";
constexpr auto kPortIDElement = "element";
constexpr auto kPortIDOutputVector = "output_vector";

}  // namespace

namespace moveit_studio::behaviors
{
PushBackVector::PushBackVector(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PushBackVector::providedPorts()
{
  return { BT::InputPort<std::vector<BT::Any>>(kPortIDInputVector, "{input_vector}", "Input BT::Any vector."),
           BT::InputPort<BT::Any>(kPortIDElement, "{element}", "Element to insert into the vector."),
           BT::OutputPort<std::vector<BT::Any>>(kPortIDOutputVector, "{output_vector}",
                                                "Output vector with element inserted.") };
}

BT::KeyValueVector PushBackVector::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "Vector Handling" },
           { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionPushBackVector } };
}

BT::NodeStatus PushBackVector::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<std::vector<BT::Any>>(kPortIDInputVector),
                                                                 getInput<BT::Any>(kPortIDElement));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input data port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  auto [output_vector, element] = ports.value();

  // Check element and vector type compatibility.
  if (!output_vector.empty() &&  // Don't type check if the vector is empty, allow insertion of any type.
      element.type() != output_vector[0].type())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Type mismatch: element type does not match vector element types.");
    return BT::NodeStatus::FAILURE;
  }

  // Insert the element at the given index.
  output_vector.push_back(element);

  setOutput(kPortIDOutputVector, output_vector);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace moveit_studio::behaviors
