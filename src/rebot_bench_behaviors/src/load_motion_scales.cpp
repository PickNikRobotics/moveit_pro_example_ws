// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
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
#include <rebot_bench_behaviors/load_motion_scales.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
BT::PortsList LoadMotionScales::providedPorts()
{
  return { BT::InputPort<std::string>("package_name", "ROS package holding the motion settings file."),
           BT::InputPort<std::string>("file_path",
                                      "Motion settings YAML file, relative to the package share directory."),
           BT::InputPort<std::string>("leg_kind", "Settings entry to load, for example joint, cartesian or gripper."),
           BT::OutputPort<double>("velocity_scale_factor"), BT::OutputPort<double>("acceleration_scale_factor") };
}

BT::KeyValueVector LoadMotionScales::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Loads one kind of motion leg's planning velocity and acceleration scale factors from a YAML file, so a "
             "configuration file sets the speed of every leg of that kind. Fails on a missing entry or a factor "
             "outside (0, 1]." } };
}

BT::NodeStatus LoadMotionScales::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::string>("package_name"), getInput<std::string>("file_path"), getInput<std::string>("leg_kind"));
  if (!inputs)
    return fail("Provide the settings package, file and leg kind: " + inputs.error());
  const auto& [package_name, file_path, leg_kind] = inputs.value();
  try
  {
    const auto scales = readMotionScales(
        (std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) / file_path).string(),
        leg_kind);
    setOutput("velocity_scale_factor", scales.velocity);
    setOutput("acceleration_scale_factor", scales.acceleration);
  }
  catch (const std::exception& error)
  {
    return fail(error.what());
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
