// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <hangar_sim_behaviors/localization_gates.hpp>

#include <istream>
#include <string>
#include <vector>

/**
 * @file
 * @brief Plain-text carriers for the gate's calibration data.
 *
 * @details The threshold in localization_gates.hpp is a measurement, so it has to be reproducible,
 * which means the calibration has to run the *shipped* scoring code over the *published* grid. Both
 * halves matter:
 *
 *  - Shipped code: calibrate_scan_match_gate links localization_gates.cpp, the same translation unit
 *    ScanMatchResidual uses. A separate offline reimplementation would be calibrating a different
 *    function that happens to look similar.
 *  - Published grid: the dump script captures whatever map_server actually put on /map, rather than
 *    re-deriving it from the .pgm with a second loader that then has to be proved equivalent
 *    cell for cell. Capturing removes the question instead of answering it.
 */
namespace hangar_sim_behaviors::calibration
{
/// One captured measurement: a scan and the pose the robot truly held when it was taken.
struct ScanSample
{
  std::string label;
  localization::ScanGeometry geometry;
  std::vector<float> ranges;
  double truth_x = 0.0;
  double truth_y = 0.0;
  double truth_yaw = 0.0;
};

/// A grid as captured from /map, in the dump format written by dump_localization_calibration_data.py.
struct GridDump
{
  localization::GridInfo info;
  std::vector<std::int8_t> data;
};

/// @throws std::runtime_error with a message naming what was wrong.
GridDump readGridDump(std::istream& input);

/// @throws std::runtime_error with a message naming what was wrong.
std::vector<ScanSample> readScanSamples(std::istream& input);

}  // namespace hangar_sim_behaviors::calibration
