// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef CLEARPATH_MECANUM_DRIVE_CONTROLLER__PARAMETER_VALIDATORS_HPP_
#define CLEARPATH_MECANUM_DRIVE_CONTROLLER__PARAMETER_VALIDATORS_HPP_

#include <cmath>
#include <string>

#include "rclcpp/parameter.hpp"
#include "tl/expected.hpp"

namespace clearpath_mecanum_drive_controller
{

[[nodiscard]] inline tl::expected<void, std::string> finite_positive(
  const rclcpp::Parameter & parameter)
{
  const double value = parameter.as_double();
  if (!std::isfinite(value) || value <= 0.0)
  {
    return tl::make_unexpected(parameter.get_name() + " must be finite and greater than zero");
  }
  return {};
}

[[nodiscard]] inline tl::expected<void, std::string> finite_non_negative(
  const rclcpp::Parameter & parameter)
{
  const double value = parameter.as_double();
  if (!std::isfinite(value) || value < 0.0)
  {
    return tl::make_unexpected(parameter.get_name() + " must be finite and non-negative");
  }
  return {};
}

[[nodiscard]] inline tl::expected<void, std::string> finite(
  const rclcpp::Parameter & parameter)
{
  if (!std::isfinite(parameter.as_double()))
  {
    return tl::make_unexpected(parameter.get_name() + " must be finite");
  }
  return {};
}

[[nodiscard]] inline tl::expected<void, std::string> finite_non_negative_array(
  const rclcpp::Parameter & parameter)
{
  for (const double value : parameter.as_double_array())
  {
    if (!std::isfinite(value) || value < 0.0)
    {
      return tl::make_unexpected(
        parameter.get_name() + " values must be finite and non-negative");
    }
  }
  return {};
}

}  // namespace clearpath_mecanum_drive_controller

#endif  // CLEARPATH_MECANUM_DRIVE_CONTROLLER__PARAMETER_VALIDATORS_HPP_
