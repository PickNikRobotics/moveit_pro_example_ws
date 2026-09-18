// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "clearpath_mecanum_drive_controller/clearpath_mecanum_drive_controller.hpp"

#ifndef ROS_DISTRO_JAZZY
#error "The installed Clearpath controller target must export its Jazzy public-header ABI definition."
#endif

int main()
{
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  static_cast<void>(controller);
  return 0;
}
