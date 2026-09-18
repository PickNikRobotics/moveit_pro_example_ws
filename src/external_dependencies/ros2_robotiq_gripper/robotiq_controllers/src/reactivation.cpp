// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "robotiq_controllers/internal/reactivation.hpp"

namespace robotiq_controllers::internal
{
ReactivationResult simulated_reactivation()
{
  return { true, "Gripper reactivation is not required in simulation." };
}
}  // namespace robotiq_controllers::internal
