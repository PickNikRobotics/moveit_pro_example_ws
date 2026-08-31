// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>

namespace robotiq_controllers::internal
{
struct ReactivationResult
{
  bool success;
  std::string message;
};

[[nodiscard]] ReactivationResult simulated_reactivation();
}  // namespace robotiq_controllers::internal
