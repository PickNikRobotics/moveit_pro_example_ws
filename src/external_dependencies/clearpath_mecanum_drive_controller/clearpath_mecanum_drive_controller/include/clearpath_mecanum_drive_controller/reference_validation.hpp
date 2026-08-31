// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>

namespace clearpath_mecanum_drive_controller::internal
{

enum class ReferenceAction
{
  ROTATE_AND_APPLY,
  WAIT_FOR_REFERENCE,
  REJECT,
};

enum class ReferenceProcessingResult
{
  OK,
  ERROR,
};

struct AxisLimits
{
  bool has_velocity_limits;
  double min_velocity;
  double max_velocity;
  bool has_acceleration_limits;
  double min_acceleration;
  double max_acceleration;
};

[[nodiscard]] inline std::optional<std::string> validate_axis_limits(
  const AxisLimits & limits, const std::string & axis_name)
{
  if (limits.has_velocity_limits &&
      (!std::isfinite(limits.min_velocity) || !std::isfinite(limits.max_velocity) ||
       limits.min_velocity > 0.0 || limits.max_velocity < 0.0 ||
       limits.min_velocity >= limits.max_velocity))
  {
    return axis_name + " velocity limits must be finite, contain zero, and have min < max";
  }
  if (limits.has_acceleration_limits &&
      (!std::isfinite(limits.min_acceleration) || !std::isfinite(limits.max_acceleration) ||
       limits.min_acceleration > 0.0 || limits.max_acceleration < 0.0 ||
       limits.min_acceleration >= limits.max_acceleration))
  {
    return axis_name + " acceleration limits must be finite, contain zero, and have min < max";
  }
  return std::nullopt;
}

[[nodiscard]] inline std::optional<double> limit_axis_reference(
  const double requested, const double previous, const double period_seconds,
  const AxisLimits & limits)
{
  if (!std::isfinite(requested) || !std::isfinite(previous) ||
      !std::isfinite(period_seconds) || period_seconds <= 0.0)
  {
    return std::nullopt;
  }

  double limited = requested;
  if (limits.has_velocity_limits)
  {
    limited = std::clamp(limited, limits.min_velocity, limits.max_velocity);
  }
  if (limits.has_acceleration_limits)
  {
    limited = std::clamp(
      limited, previous + limits.min_acceleration * period_seconds,
      previous + limits.max_acceleration * period_seconds);
  }
  return limited;
}

template <std::size_t CommandCount, typename WriteCommand>
[[nodiscard]] bool write_commands(
  const std::array<double, CommandCount> & commands, WriteCommand && write_command)
{
  bool all_commands_written = true;
  for (std::size_t index = 0; index < CommandCount; ++index)
  {
    all_commands_written = write_command(index, commands[index]) && all_commands_written;
  }
  return all_commands_written;
}

[[nodiscard]] inline ReferenceAction determine_reference_action(
  const std::array<double, 3> & reference)
{
  if (std::all_of(reference.begin(), reference.end(), [](double value) { return std::isnan(value); }))
  {
    return ReferenceAction::WAIT_FOR_REFERENCE;
  }
  if (std::all_of(reference.begin(), reference.end(), [](double value) { return std::isfinite(value); }))
  {
    return ReferenceAction::ROTATE_AND_APPLY;
  }
  return ReferenceAction::REJECT;
}

template <typename TimePoint, typename Duration, typename PrepareReference, typename ApplyReference,
          typename CommitReference, typename StopWheels>
[[nodiscard]] ReferenceProcessingResult process_reference(
  const std::array<double, 3> & reference, const TimePoint & current_time,
  const Duration & command_timeout, TimePoint & last_command_time,
  PrepareReference && prepare_reference, ApplyReference && apply_reference,
  CommitReference && commit_reference, StopWheels && stop_wheels,
  const bool is_new_reference = true)
{
  const ReferenceAction action = determine_reference_action(reference);
  if (action == ReferenceAction::REJECT)
  {
    std::ignore = stop_wheels();
    return ReferenceProcessingResult::ERROR;
  }
  if (action == ReferenceAction::WAIT_FOR_REFERENCE)
  {
    if (current_time < last_command_time || command_timeout < current_time - last_command_time)
    {
      return stop_wheels() ? ReferenceProcessingResult::OK : ReferenceProcessingResult::ERROR;
    }
    return ReferenceProcessingResult::OK;
  }
  if (
    !is_new_reference &&
    (current_time < last_command_time || command_timeout < current_time - last_command_time))
  {
    return stop_wheels() ? ReferenceProcessingResult::OK : ReferenceProcessingResult::ERROR;
  }
  if (!prepare_reference() || !apply_reference())
  {
    std::ignore = stop_wheels();
    return ReferenceProcessingResult::ERROR;
  }
  commit_reference();
  if (is_new_reference)
  {
    last_command_time = current_time;
  }
  return ReferenceProcessingResult::OK;
}

}  // namespace clearpath_mecanum_drive_controller::internal
