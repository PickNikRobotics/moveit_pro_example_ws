// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "clearpath_mecanum_drive_controller/reference_validation.hpp"

#include <array>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace clearpath_mecanum_drive_controller::internal
{
namespace
{

constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();

TEST(ReferenceValidation, FiniteCommandRefreshesTimeoutBeforeIdleStop)
{
  // GIVEN a controller configured long ago and observable production operations.
  constexpr int command_timeout = 50;
  int last_command_time = 0;
  int rotate_calls = 0;
  int apply_calls = 0;
  int commit_calls = 0;
  int stop_calls = 0;
  const auto rotate_reference = [&rotate_calls]() {
    ++rotate_calls;
    return true;
  };
  const auto apply_reference = [&apply_calls]() {
    ++apply_calls;
    return true;
  };
  const auto commit_reference = [&commit_calls]() { ++commit_calls; };
  const auto stop_wheels = [&stop_calls]() {
    ++stop_calls;
    return true;
  };

  // WHEN a finite command is accepted at t=100.
  const ReferenceProcessingResult command_result = process_reference(
    { 1.0, -2.0, 0.5 }, 100, command_timeout, last_command_time, rotate_reference,
    apply_reference, commit_reference, stop_wheels);

  // THEN preparation, application, and commit run and the accepted-command timestamp advances.
  EXPECT_EQ(command_result, ReferenceProcessingResult::OK);
  EXPECT_EQ(last_command_time, 100);
  EXPECT_EQ(rotate_calls, 1);
  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(commit_calls, 1);
  EXPECT_EQ(stop_calls, 0);

  // WHEN the consumed command becomes an all-NaN sentinel before timeout.
  const ReferenceProcessingResult before_timeout_result = process_reference(
    { NAN_VALUE, NAN_VALUE, NAN_VALUE }, 125, command_timeout, last_command_time,
    rotate_reference, apply_reference, commit_reference, stop_wheels);

  // THEN the previous command remains active without repeating command processing.
  EXPECT_EQ(before_timeout_result, ReferenceProcessingResult::OK);
  EXPECT_EQ(last_command_time, 100);
  EXPECT_EQ(rotate_calls, 1);
  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(commit_calls, 1);
  EXPECT_EQ(stop_calls, 0);

  // WHEN another idle cycle occurs after timeout.
  const ReferenceProcessingResult after_timeout_result = process_reference(
    { NAN_VALUE, NAN_VALUE, NAN_VALUE }, 151, command_timeout, last_command_time,
    rotate_reference, apply_reference, commit_reference, stop_wheels);

  // THEN the wheels stop without turning normal idleness into a controller error.
  EXPECT_EQ(after_timeout_result, ReferenceProcessingResult::OK);
  EXPECT_EQ(last_command_time, 100);
  EXPECT_EQ(rotate_calls, 1);
  EXPECT_EQ(apply_calls, 1);
  EXPECT_EQ(commit_calls, 1);
  EXPECT_EQ(stop_calls, 1);
}

TEST(ReferenceValidation, InvalidReferenceStopsWithoutRotationOrInverseKinematics)
{
  // GIVEN mixed-NaN and infinite references.
  const std::array<std::array<double, 3>, 3> invalid_references{ {
    { NAN_VALUE, 0.0, 0.0 },
    { INFINITY_VALUE, 0.0, 0.0 },
    { -INFINITY_VALUE, 0.0, 0.0 },
  } };

  for (const auto & reference : invalid_references)
  {
    int last_command_time = 10;
    int rotate_calls = 0;
    int apply_calls = 0;
    int stop_calls = 0;

    // WHEN the production dispatch processes an invalid command.
    const ReferenceProcessingResult result = process_reference(
      reference, 20, 50, last_command_time,
      [&rotate_calls]() {
        ++rotate_calls;
        return true;
      },
      [&apply_calls]() {
        ++apply_calls;
        return true;
      },
      []() {},
      [&stop_calls]() {
        ++stop_calls;
        return true;
      });

    // THEN it fails closed without refreshing the accepted-command timestamp.
    EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
    EXPECT_EQ(last_command_time, 10);
    EXPECT_EQ(rotate_calls, 0);
    EXPECT_EQ(apply_calls, 0);
    EXPECT_EQ(stop_calls, 1);
  }
}

TEST(ReferenceValidation, RotationFailureStopsBeforeInverseKinematics)
{
  // GIVEN a finite reference whose world-frame rotation fails.
  int last_command_time = 10;
  int apply_calls = 0;
  int stop_calls = 0;

  // WHEN the production dispatch processes the failed rotation.
  const ReferenceProcessingResult result = process_reference(
    { 1.0, 0.0, 0.0 }, 20, 50, last_command_time, []() { return false; },
    [&apply_calls]() {
      ++apply_calls;
      return true;
    },
    []() {},
    [&stop_calls]() {
      ++stop_calls;
      return true;
    });

  // THEN it stops without invoking IK or refreshing the timestamp.
  EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
  EXPECT_EQ(last_command_time, 10);
  EXPECT_EQ(apply_calls, 0);
  EXPECT_EQ(stop_calls, 1);
}

TEST(ReferenceValidation, InverseKinematicsFailureStopsWheels)
{
  // GIVEN a finite reference whose inverse kinematics output is non-finite.
  int last_command_time = 10;
  int stop_calls = 0;

  // WHEN rotation succeeds but command application fails.
  const ReferenceProcessingResult result = process_reference(
    { 1.0, 0.0, 0.0 }, 20, 50, last_command_time, []() { return true; },
    []() { return false; }, []() {}, [&stop_calls]() {
      ++stop_calls;
      return true;
    });

  // THEN the production dispatch fails closed without refreshing the timestamp.
  EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
  EXPECT_EQ(last_command_time, 10);
  EXPECT_EQ(stop_calls, 1);
}

TEST(ReferenceValidation, StopWriteFailureReturnsErrorAfterTimeout)
{
  // GIVEN a controller whose command interfaces reject a timeout stop.
  constexpr int command_timeout = 50;
  int last_command_time = 100;
  const auto unused_operation = []() { return true; };
  const auto stop_wheels = []() { return false; };

  // WHEN an idle reference is processed after the timeout.
  const ReferenceProcessingResult result = process_reference(
    { NAN_VALUE, NAN_VALUE, NAN_VALUE }, 151, command_timeout, last_command_time,
    unused_operation, unused_operation, unused_operation, stop_wheels);

  // THEN the failed safety stop is reported to the controller manager.
  EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
}

TEST(ReferenceTransaction, SuccessfulApplicationCommitsLimitedReference)
{
  // GIVEN a provisional acceleration-limited command and prior coherent history.
  const AxisLimits limits{ false, 0.0, 0.0, true, -1.0, 1.0 };
  double limiter_state = 0.4;
  const std::optional<double> provisional =
    limit_axis_reference(1.0, limiter_state, 0.1, limits);
  ASSERT_TRUE(provisional.has_value());
  int last_command_time = 0;

  // WHEN preparation and all actuator writes succeed.
  const ReferenceProcessingResult result = process_reference(
    { 1.0, 0.0, 0.0 }, 1, 10, last_command_time, []() { return true; },
    []() { return true; }, [&limiter_state, &provisional]() { limiter_state = *provisional; },
    []() { return true; });

  // THEN the applied command becomes the acceleration history for the next update.
  EXPECT_EQ(result, ReferenceProcessingResult::OK);
  EXPECT_DOUBLE_EQ(limiter_state, 0.5);
  const std::optional<double> next = limit_axis_reference(1.0, limiter_state, 0.1, limits);
  ASSERT_TRUE(next.has_value());
  EXPECT_DOUBLE_EQ(*next, 0.6);
}

TEST(ReferenceTransaction, SuccessfulRecoveryStopCommitsZero)
{
  // GIVEN a provisional command whose actuator application will fail.
  const AxisLimits limits{ false, 0.0, 0.0, true, -1.0, 1.0 };
  double limiter_state = 0.4;
  int last_command_time = 0;

  // WHEN application fails but every recovery-stop write succeeds.
  const ReferenceProcessingResult result = process_reference(
    { 1.0, 0.0, 0.0 }, 1, 10, last_command_time, []() { return true; },
    []() { return false; }, [&limiter_state]() { limiter_state = 0.5; },
    [&limiter_state]() {
      limiter_state = 0.0;
      return true;
    });

  // THEN the failed provisional command is never committed and the next step starts from zero.
  EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
  EXPECT_DOUBLE_EQ(limiter_state, 0.0);
  const std::optional<double> next = limit_axis_reference(1.0, limiter_state, 0.1, limits);
  ASSERT_TRUE(next.has_value());
  EXPECT_DOUBLE_EQ(*next, 0.1);
}

TEST(ReferenceTransaction, FailedRecoveryStopPreservesCoherentHistory)
{
  // GIVEN prior coherent history and an actuator transaction that cannot apply or stop fully.
  const AxisLimits limits{ false, 0.0, 0.0, true, -1.0, 1.0 };
  double limiter_state = 0.4;
  int last_command_time = 0;

  // WHEN application and the recovery stop both fail.
  const ReferenceProcessingResult result = process_reference(
    { 1.0, 0.0, 0.0 }, 1, 10, last_command_time, []() { return true; },
    []() { return false; }, [&limiter_state]() { limiter_state = 0.5; },
    []() { return false; });

  // THEN neither the provisional command nor a zero stop is recorded as successfully applied.
  EXPECT_EQ(result, ReferenceProcessingResult::ERROR);
  EXPECT_DOUBLE_EQ(limiter_state, 0.4);
  const std::optional<double> next = limit_axis_reference(1.0, limiter_state, 0.1, limits);
  ASSERT_TRUE(next.has_value());
  EXPECT_DOUBLE_EQ(*next, 0.5);
}

TEST(WriteCommands, RejectedWriteReturnsFalseAndAttemptsEveryWheel)
{
  // GIVEN wheel commands and a command interface that rejects the second write.
  const std::array<double, 4> commands{};
  std::array<double, 4> attempted_commands{};
  std::size_t write_calls = 0;

  // WHEN the production write wrapper applies the commands.
  const bool result = write_commands(
    commands, [&attempted_commands, &write_calls](const std::size_t index, const double value) {
      attempted_commands[index] = value;
      ++write_calls;
      return index != 1;
    });

  // THEN it reports failure, but every wheel is still sent its command.
  EXPECT_FALSE(result);
  EXPECT_EQ(write_calls, commands.size());
  EXPECT_EQ(attempted_commands, commands);
}

TEST(WriteCommands, AcceptedWritesReturnTrue)
{
  // GIVEN a complete set of wheel-stop commands.
  const std::array<double, 4> commands{};

  // WHEN every command interface accepts its write.
  const bool result = write_commands(
    commands, [](const std::size_t, const double) { return true; });

  // THEN the production command-write wrapper reports success.
  EXPECT_TRUE(result);
}

TEST(AxisLimits, RejectsInvalidEnabledBounds)
{
  // GIVEN enabled limits that are non-finite, inverted, or exclude zero.
  const std::array<AxisLimits, 4> invalid_limits{ {
    { true, NAN_VALUE, 1.0, false, 0.0, 0.0 },
    { true, 1.0, 2.0, false, 0.0, 0.0 },
    { false, 0.0, 0.0, true, -1.0, INFINITY_VALUE },
    { false, 0.0, 0.0, true, 1.0, 2.0 },
  } };

  for (const AxisLimits & limits : invalid_limits)
  {
    // WHEN configuration validates the axis bounds.
    const std::optional<std::string> error = validate_axis_limits(limits, "linear.x");

    // THEN malformed safety limits are rejected.
    EXPECT_TRUE(error.has_value());
  }
}

TEST(AxisLimits, ClampsVelocityAndAcceleration)
{
  // GIVEN symmetric velocity and acceleration limits and a prior stopped command.
  const AxisLimits limits{ true, -1.3, 1.3, true, -1.0, 1.0 };

  // WHEN a large positive command is limited over a 100 ms control period.
  const std::optional<double> first = limit_axis_reference(10.0, 0.0, 0.1, limits);

  // THEN acceleration is limited before the velocity ceiling is reached.
  ASSERT_TRUE(first.has_value());
  EXPECT_DOUBLE_EQ(*first, 0.1);

  // WHEN the prior command is already at the velocity ceiling.
  const std::optional<double> saturated = limit_axis_reference(10.0, 1.3, 0.1, limits);

  // THEN the command remains within the configured velocity bound.
  ASSERT_TRUE(saturated.has_value());
  EXPECT_DOUBLE_EQ(*saturated, 1.3);
}

TEST(AxisLimits, HandlesAsymmetricNegativeAndExactBoundaryRequests)
{
  // GIVEN asymmetric velocity and acceleration limits.
  const AxisLimits limits{ true, -0.7, 1.3, true, -2.0, 1.0 };

  // WHEN a large negative request is limited from rest.
  const std::optional<double> limited_negative = limit_axis_reference(-10.0, 0.0, 0.1, limits);

  // THEN the asymmetric negative acceleration bound determines the next reference.
  ASSERT_TRUE(limited_negative.has_value());
  EXPECT_DOUBLE_EQ(*limited_negative, -0.2);

  // WHEN the request and prior state are exactly on the positive velocity boundary.
  const std::optional<double> limited_boundary = limit_axis_reference(1.3, 1.3, 0.1, limits);

  // THEN the exact boundary remains valid and unchanged.
  ASSERT_TRUE(limited_boundary.has_value());
  EXPECT_DOUBLE_EQ(*limited_boundary, 1.3);
}

TEST(AxisLimits, RejectsInvalidControlPeriod)
{
  // GIVEN valid limits but a nonpositive or non-finite control period.
  const AxisLimits limits{ true, -1.0, 1.0, true, -1.0, 1.0 };

  // WHEN limiting is attempted without a valid elapsed duration.
  // THEN the command path fails closed.
  EXPECT_FALSE(limit_axis_reference(0.5, 0.0, 0.0, limits).has_value());
  EXPECT_FALSE(limit_axis_reference(0.5, 0.0, NAN_VALUE, limits).has_value());
}

}  // namespace
}  // namespace clearpath_mecanum_drive_controller::internal
