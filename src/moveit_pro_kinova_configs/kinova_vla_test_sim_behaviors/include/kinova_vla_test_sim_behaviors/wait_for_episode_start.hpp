// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <string>

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/service_client_interface.hpp>
#include <moveit_studio_internal_msgs/srv/get_active_recording.hpp>
#include <tl_expected/expected.hpp>

namespace kinova_vla_test_sim_behaviors
{
using GetActiveRecordingSrv = moveit_studio_internal_msgs::srv::GetActiveRecording;

/**
 * @brief Blocks until the active Trainer recording session has opened an episode.
 */
class WaitForEpisodeStart : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  WaitForEpisodeStart(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();

  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

private:
  std::shared_future<tl::expected<bool, std::string>> future_;
  std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<GetActiveRecordingSrv>> client_;
  std::atomic_bool halted_{ false };
};
}  // namespace kinova_vla_test_sim_behaviors
