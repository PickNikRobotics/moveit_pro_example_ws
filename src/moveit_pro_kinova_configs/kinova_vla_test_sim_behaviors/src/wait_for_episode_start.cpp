// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kinova_vla_test_sim_behaviors/wait_for_episode_start.hpp>

#include <chrono>
#include <thread>

#include <fmt/format.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <nlohmann/json.hpp>

namespace
{
inline constexpr auto kDescriptionWaitForEpisodeStart = R"(
                <p>
                    Blocks until the active Trainer recording session reports that it has opened
                    an episode, then succeeds.
                </p>
                <p>
                    <code>RecordEpisode</code> returns as soon as the recorder process has been
                    spawned, but the episode does not begin until the Trainer publishes its start
                    marker, seconds later. Motion before that marker is silently discarded at
                    conversion: the converter segments on markers and drops any segment holding
                    none, treating it as recorder spillover. Demonstrations then begin partway
                    through the motion, with no error to notice. Tick this between
                    <code>RecordEpisode</code> and the demonstrated motion.
                </p>
                <p>
                    This polls the session state rather than watching for the marker, so it cannot
                    miss the transition by subscribing a moment too late. It is still worth pairing
                    with a short <code>WaitForDuration</code>, which keeps the arm still across the
                    few frames straddling the marker; the converter's idle trimming removes them.
                </p>
                <p>
                    Fails when no episode opens within <code>timeout</code>, naming the last state
                    seen.
                </p>
            )";

constexpr auto kPortIDServiceName = "service_name";
constexpr auto kPortIDTimeout = "timeout";

constexpr auto kDefaultServiceName = "/trainer/active_recording";

// The RecordingState value that means the episode's start marker has been published.
constexpr auto kRecordingState = "recording";

constexpr std::chrono::milliseconds kPollPeriod{ 50 };
constexpr std::chrono::duration<double> kServerTimeout{ 5.0 };
constexpr std::chrono::duration<double> kResponseTimeout{ 5.0 };

/** @brief The session's state, or "" when no session is active or the payload has no state. */
std::string stateOf(const std::string& session_json)
{
  if (session_json.empty())
  {
    return "";
  }
  const auto session = nlohmann::json::parse(session_json, nullptr, false);
  if (session.is_discarded() || !session.contains("state") || !session["state"].is_string())
  {
    return "";
  }
  return session["state"].get<std::string>();
}
}  // namespace

namespace kinova_vla_test_sim_behaviors
{
WaitForEpisodeStart::WaitForEpisodeStart(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
  , client_(std::make_unique<moveit_pro::behaviors::RclcppClientInterface<GetActiveRecordingSrv>>(shared_resources))
{
}

BT::PortsList WaitForEpisodeStart::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDServiceName, kDefaultServiceName, "Name of the Trainer active_recording service."),
    BT::InputPort<double>(kPortIDTimeout, "30.0", "Seconds to wait for an episode to open."),
  };
}

BT::KeyValueVector WaitForEpisodeStart::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Color-Cube Stacking" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionWaitForEpisodeStart } };
}

tl::expected<bool, std::string> WaitForEpisodeStart::doWork()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDServiceName),
                                                              getInput<double>(kPortIDTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [service_name, timeout] = ports.value();

  halted_ = false;
  client_->initialize(service_name, kServerTimeout, kResponseTimeout);
  if (!client_->waitForServiceServer())
  {
    return tl::make_unexpected(fmt::format("No Trainer active_recording service on '{}'.", service_name));
  }
  // Nothing here holds a goal, so a halt may interrupt the poll as soon as it arrives.
  notifyCanHalt();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  std::string last_state;
  while (!halted_)
  {
    const auto response = client_->syncSendRequest(GetActiveRecordingSrv::Request{});
    if (!response.has_value())
    {
      return tl::make_unexpected("Failed to read the active recording: " + response.error());
    }
    if (!response.value().status.success)
    {
      return tl::make_unexpected("The Trainer refused to report the active recording: " +
                                 response.value().status.error_message);
    }
    last_state = stateOf(response.value().session_json);
    if (last_state == kRecordingState)
    {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return tl::make_unexpected(
          fmt::format("No recording episode opened within {:g}s; the session's last state was '{}'.", timeout,
                      last_state.empty() ? "no active session" : last_state));
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  return false;
}

tl::expected<void, std::string> WaitForEpisodeStart::doHalt()
{
  halted_ = true;
  client_->cancelRequest();
  return {};
}
}  // namespace kinova_vla_test_sim_behaviors
