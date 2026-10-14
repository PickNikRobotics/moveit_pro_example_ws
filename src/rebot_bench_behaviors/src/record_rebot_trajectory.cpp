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
#include <chrono>
#include <fstream>
#include <iomanip>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <mutex>
#include <rebot_bench_behaviors/record_rebot_trajectory.hpp>
#include <sstream>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rebot_bench_behaviors
{
BT::PortsList RecordRebotTrajectory::providedPorts()
{
  return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>("trajectory"),
           BT::InputPort<std::string>("file_path", "/tmp/rebot-trajectory-audit.yaml",
                                      "Append-only trajectory audit file.") };
}
BT::KeyValueVector RecordRebotTrajectory::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Record full trajectory before dispatch; fails execution if persistence fails." } };
}
BT::NodeStatus RecordRebotTrajectory::tick()
{
  const auto trajectory = getInput<trajectory_msgs::msg::JointTrajectory>("trajectory");
  const auto path = getInput<std::string>("file_path");
  if (!trajectory || !path || trajectory->points.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Provide a trajectory with at least one point and a file_path.");
    return BT::NodeStatus::FAILURE;
  }
  // A dispatch timestamp is not an execution start timestamp: pair this record
  // with the controller state's clock and trajectory time to reconstruct progress.
  const double at = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream record;
  record << "---\ndispatched_at: " << std::setprecision(17) << at << "\ntrajectory:\n";
  std::istringstream yaml(trajectory_msgs::msg::to_yaml(trajectory.value()));
  for (std::string line; std::getline(yaml, line);)
    record << "  " << line << '\n';
  record << "...\n";
  static std::mutex mutex;
  const std::scoped_lock lock(mutex);
  std::ofstream output(path.value(), std::ios::app);
  output << record.str();
  output.flush();
  if (!output)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Cannot persist trajectory audit: " + path.value());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
