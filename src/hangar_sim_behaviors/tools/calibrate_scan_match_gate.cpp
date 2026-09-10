// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

/**
 * @file
 * @brief Measure the fit-to-map gate's threshold against a specific map. Offline, no ROS.
 *
 * @details A gate whose threshold was measured on a different map is not a gate. This tool is what
 * makes re-measuring cheap enough to actually do after a map rebuild:
 *
 *   ros2 run hangar_sim dump_localization_calibration_data.py --output-dir /tmp/calib
 *   ros2 run hangar_sim_behaviors calibrate_scan_match_gate /tmp/calib/grid.txt /tmp/calib/scans.txt
 *
 * It reports three numbers that matter and one that decides:
 *   - the WORST score any true pose gets (the floor a threshold must sit below);
 *   - the score of deliberately wrong poses at a few offsets (what a threshold must reject);
 *   - the STRONGEST alias found anywhere on the map -- the best-scoring pose that is nowhere near
 *     the truth, found by sweeping the whole free space rather than by guessing where to look;
 *   - the separation between the first and the third, which is the margin the threshold lives in.
 *
 * A narrow separation is a finding to report, not a number to split the difference on. It means the
 * map has structure that repeats, and the honest response is a tighter click and a shorter drive,
 * not a lower threshold.
 */

#include <hangar_sim_behaviors/calibration_io.hpp>
#include <hangar_sim_behaviors/localization_gates.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace localization = hangar_sim_behaviors::localization;
namespace calibration = hangar_sim_behaviors::calibration;

/// A candidate pose and what it scored.
struct Scored
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double inlier_fraction = 0.0;
  int beams_used = 0;
};

struct SweepSettings
{
  int max_beams = localization::kMaxBeams;
  double min_range = localization::kLaserMinRange;
  double max_range = localization::kLaserMaxRange;
  double max_obstacle_distance = localization::kMaxObstacleDistance;
  double inlier_distance = localization::kInlierDistance;
  /// A candidate this close to the truth is the truth, not an alias.
  double alias_keepout_m = 2.0;
  /// Coarse sweep stride over the map, metres, and its yaw stride, radians.
  double coarse_stride_m = 0.40;
  double coarse_yaw_stride = M_PI / 18.0;  // 10 degrees
  /// Local refinement around each coarse winner.
  double fine_stride_m = 0.05;
  double fine_span_m = 0.40;
  double fine_yaw_stride = M_PI / 180.0;
  double fine_yaw_span = 10.0 * M_PI / 180.0;
};

Scored score(const localization::DistanceField& field, const calibration::ScanSample& sample,
             const SweepSettings& settings, double x, double y, double yaw)
{
  const auto stats =
      localization::computeScanResidual(field, sample.geometry, sample.ranges, settings.max_beams, settings.min_range,
                                        settings.max_range, settings.inlier_distance, x, y, yaw);
  return Scored{ x, y, yaw, stats.inlier_fraction, stats.beams_used };
}

double wrapAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/// Cells the sweep may stand on: free, and not inside a wall.
std::vector<std::pair<double, double>> freePositions(const calibration::GridDump& grid, double stride)
{
  std::vector<std::pair<double, double>> positions;
  const int step = std::max(1, static_cast<int>(std::lround(stride / grid.info.resolution)));
  // Cell centre out of the grid's own frame and back into the map frame -- the exact inverse of the
  // rotate-then-floor DistanceField::at does. Every map this workspace ships has origin_yaw == 0,
  // but the two halves of this tool have to agree on the convention or the sweep stands its
  // candidates somewhere other than the cells it says it is scoring.
  const double cos_yaw = std::cos(grid.info.origin_yaw);
  const double sin_yaw = std::sin(grid.info.origin_yaw);
  for (int row = 0; row < grid.info.height; row += step)
  {
    for (int column = 0; column < grid.info.width; column += step)
    {
      const auto index =
          static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.info.width) + static_cast<std::size_t>(column);
      // Only known-free cells. Standing the robot in unknown space would invent aliases that no
      // click could ever produce.
      if (grid.data[index] != 0)
      {
        continue;
      }
      const double local_x = (static_cast<double>(column) + 0.5) * grid.info.resolution;
      const double local_y = (static_cast<double>(row) + 0.5) * grid.info.resolution;
      positions.emplace_back(grid.info.origin_x + cos_yaw * local_x - sin_yaw * local_y,
                             grid.info.origin_y + sin_yaw * local_x + cos_yaw * local_y);
    }
  }
  return positions;
}

/**
 * @brief The best-scoring pose anywhere on the map that is at least alias_keepout_m from the truth.
 *
 * @return Nothing when no candidate was ever accepted -- a sample whose returns are all NaN, or all
 *         outside the range filters, scores every candidate at zero. Reporting the zero-initialised
 *         placeholder instead would name a pose at the map-frame origin that the sweep never stood
 *         at, and the local refinement would then hunt a 0.8 m box around a point that need not
 *         even be on the grid.
 */
std::optional<Scored> strongestAlias(const localization::DistanceField& field, const calibration::GridDump& grid,
                                     const calibration::ScanSample& sample, const SweepSettings& settings)
{
  const auto positions = freePositions(grid, settings.coarse_stride_m);
  std::optional<Scored> best;
  for (const auto& [x, y] : positions)
  {
    const double dx = x - sample.truth_x;
    const double dy = y - sample.truth_y;
    if (std::hypot(dx, dy) < settings.alias_keepout_m)
    {
      continue;
    }
    for (double yaw = -M_PI; yaw < M_PI; yaw += settings.coarse_yaw_stride)
    {
      const auto candidate = score(field, sample, settings, x, y, yaw);
      // A pose that explains none of the scan is not an alias, it is just a pose. Requiring a
      // positive score is what makes "nothing came back" distinguishable from "the strongest
      // wrong pose on this map happens to score zero", which no real map and scan produce.
      if (candidate.inlier_fraction > 0.0 && (!best.has_value() || candidate.inlier_fraction > best->inlier_fraction))
      {
        best = candidate;
      }
    }
  }
  if (!best.has_value())
  {
    return std::nullopt;
  }

  // Refine locally: the coarse stride can miss the peak by a fifth of a metre, which on this
  // measurement is the difference between a comfortable margin and a reported one.
  Scored refined = *best;
  for (double dx = -settings.fine_span_m; dx <= settings.fine_span_m; dx += settings.fine_stride_m)
  {
    for (double dy = -settings.fine_span_m; dy <= settings.fine_span_m; dy += settings.fine_stride_m)
    {
      const double x = best->x + dx;
      const double y = best->y + dy;
      if (std::hypot(x - sample.truth_x, y - sample.truth_y) < settings.alias_keepout_m)
      {
        continue;
      }
      for (double dyaw = -settings.fine_yaw_span; dyaw <= settings.fine_yaw_span; dyaw += settings.fine_yaw_stride)
      {
        const auto candidate = score(field, sample, settings, x, y, wrapAngle(best->yaw + dyaw));
        if (candidate.inlier_fraction > refined.inlier_fraction)
        {
          refined = candidate;
        }
      }
    }
  }
  return refined;
}

/// The BEST score over that same ring -- the number a threshold has to reject, not merely beat.
Scored bestOverOffsetRing(const localization::DistanceField& field, const calibration::ScanSample& sample,
                          const SweepSettings& settings, double offset_m, double offset_yaw)
{
  Scored best;
  constexpr int kBearings = 16;
  for (int bearing_index = 0; bearing_index < kBearings; ++bearing_index)
  {
    const double bearing = 2.0 * M_PI * static_cast<double>(bearing_index) / static_cast<double>(kBearings);
    for (const double yaw_sign : { -1.0, 1.0 })
    {
      const auto candidate =
          score(field, sample, settings, sample.truth_x + offset_m * std::cos(bearing),
                sample.truth_y + offset_m * std::sin(bearing), wrapAngle(sample.truth_yaw + yaw_sign * offset_yaw));
      if (candidate.inlier_fraction > best.inlier_fraction)
      {
        best = candidate;
      }
    }
  }
  return best;
}

std::string percent(double fraction)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "%.1f%%", fraction * 100.0);
  return buffer;
}

/// Parse a positive number from the command line, refusing the silent zero std::ato* returns.
template <typename Number>
bool parsePositive(const char* text, const char* what, Number& out)
{
  try
  {
    const double value = std::stod(text);
    if (!(value > 0.0))
    {
      std::cerr << what << " must be greater than zero, got '" << text << "'\n";
      return false;
    }
    out = static_cast<Number>(value);
    return true;
  }
  catch (const std::exception&)
  {
    std::cerr << what << " is not a number: '" << text << "'\n";
    return false;
  }
}
}  // namespace

int main(int argc, char** argv)
{
  // The scoring constants in localization_gates.hpp are the defaults, but the numbers the shipped
  // gate actually runs with are the input-port defaults of the "Refine Localization In Place
  // Subtree" Objective, which is their single home. When those are re-measured they change there,
  // not here, so a re-measurement has to be able to pass them in -- otherwise this tool quietly
  // reports a band for a gate that is no longer the one shipping, which is the exact failure it
  // exists to prevent.
  if (argc < 3)
  {
    std::cerr << "usage: calibrate_scan_match_gate <grid.txt> <scans.txt>\n"
                 "                                 [max_beams] [inlier_distance] [max_obstacle_distance]\n"
                 "  grid.txt               occupancy grid as captured from /map\n"
                 "  scans.txt              scan samples with their MuJoCo ground-truth poses\n"
                 "  max_beams              beams scored per pose\n"
                 "  inlier_distance        endpoint-to-obstacle distance counted as a fit, metres\n"
                 "  max_obstacle_distance  distance at which the likelihood field saturates, metres\n"
                 "\n"
                 "The three overrides default to the constants in localization_gates.hpp. Pass the\n"
                 "values from the ScanMatchResidual ports in refine_localization_in_place_subtree.xml\n"
                 "whenever they differ, or this measures a gate that is not the one shipping.\n";
    return EXIT_FAILURE;
  }

  SweepSettings settings;
  if (argc >= 4 && !parsePositive(argv[3], "max_beams", settings.max_beams))
  {
    return EXIT_FAILURE;
  }
  if (argc >= 5 && !parsePositive(argv[4], "inlier_distance", settings.inlier_distance))
  {
    return EXIT_FAILURE;
  }
  if (argc >= 6 && !parsePositive(argv[5], "max_obstacle_distance", settings.max_obstacle_distance))
  {
    return EXIT_FAILURE;
  }
  // The field saturates at max_obstacle_distance, so an inlier band at or above it calls every
  // beam an inlier and every pose on the map scores 100%. That is not a gate, and the separation
  // this tool would print for it is meaningless rather than merely wrong.
  if (settings.inlier_distance >= settings.max_obstacle_distance)
  {
    std::cerr << "inlier_distance (" << settings.inlier_distance << " m) must be below max_obstacle_distance ("
              << settings.max_obstacle_distance << " m); above it every beam counts as a fit.\n";
    return EXIT_FAILURE;
  }

  calibration::GridDump grid;
  std::vector<calibration::ScanSample> samples;
  try
  {
    std::ifstream grid_file(argv[1]);
    if (!grid_file)
    {
      std::cerr << "cannot open grid file: " << argv[1] << "\n";
      return EXIT_FAILURE;
    }
    grid = calibration::readGridDump(grid_file);

    std::ifstream scan_file(argv[2]);
    if (!scan_file)
    {
      std::cerr << "cannot open scans file: " << argv[2] << "\n";
      return EXIT_FAILURE;
    }
    samples = calibration::readScanSamples(scan_file);
  }
  catch (const std::exception& error)
  {
    std::cerr << "could not read the calibration data: " << error.what() << "\n";
    return EXIT_FAILURE;
  }

  if (samples.empty())
  {
    std::cerr << "no scan samples to calibrate against\n";
    return EXIT_FAILURE;
  }

  const auto field = localization::buildDistanceField(grid.info, grid.data, settings.max_obstacle_distance);
  if (!field.valid())
  {
    std::cerr << "the captured grid did not produce a usable distance field\n";
    return EXIT_FAILURE;
  }

  std::cout << "grid: " << grid.info.width << " x " << grid.info.height << " at " << grid.info.resolution
            << " m, origin (" << grid.info.origin_x << ", " << grid.info.origin_y << ")\n"
            << "settings: max_beams " << settings.max_beams << ", inlier_distance " << settings.inlier_distance
            << " m, range [" << settings.min_range << ", " << settings.max_range << "] m, field truncated at "
            << settings.max_obstacle_distance << " m\n"
            << "samples: " << samples.size() << "\n\n";

  double worst_true = 1.0;
  std::string worst_true_label;
  double best_alias = 0.0;
  std::string best_alias_label;
  std::optional<Scored> best_alias_pose;
  double best_near_miss_10cm = 0.0;
  double best_near_miss_25cm = 0.0;
  double best_near_miss_50cm = 0.0;

  std::cout << std::left << std::setw(22) << "sample" << std::setw(10) << "beams" << std::setw(10) << "truth"
            << std::setw(12) << "0.10m/1deg" << std::setw(12) << "0.25m/3deg" << std::setw(12) << "0.50m/5deg"
            << "best alias\n";
  std::cout << std::string(90, '-') << "\n";

  for (const auto& sample : samples)
  {
    const auto truth = score(field, sample, settings, sample.truth_x, sample.truth_y, sample.truth_yaw);
    // The worst a wrong pose can look is not what a gate must beat; the BEST is. Report that.
    const auto near_10 = bestOverOffsetRing(field, sample, settings, 0.10, 1.0 * M_PI / 180.0);
    const auto near_25 = bestOverOffsetRing(field, sample, settings, 0.25, 3.0 * M_PI / 180.0);
    const auto near_50 = bestOverOffsetRing(field, sample, settings, 0.50, 5.0 * M_PI / 180.0);
    const auto alias = strongestAlias(field, grid, sample, settings);

    if (truth.inlier_fraction < worst_true)
    {
      worst_true = truth.inlier_fraction;
      worst_true_label = sample.label;
    }
    if (alias.has_value() && (!best_alias_pose.has_value() || alias->inlier_fraction > best_alias))
    {
      best_alias = alias->inlier_fraction;
      best_alias_label = sample.label;
      best_alias_pose = alias;
    }
    best_near_miss_10cm = std::max(best_near_miss_10cm, near_10.inlier_fraction);
    best_near_miss_25cm = std::max(best_near_miss_25cm, near_25.inlier_fraction);
    best_near_miss_50cm = std::max(best_near_miss_50cm, near_50.inlier_fraction);

    std::cout << std::left << std::setw(22) << sample.label << std::setw(10) << truth.beams_used << std::setw(10)
              << percent(truth.inlier_fraction) << std::setw(12) << percent(near_10.inlier_fraction) << std::setw(12)
              << percent(near_25.inlier_fraction) << std::setw(12) << percent(near_50.inlier_fraction)
              << (alias.has_value() ? percent(alias->inlier_fraction) : std::string("none")) << "\n";
  }

  const double separation = worst_true - best_alias;
  std::cout << "\n"
            << "worst true pose            " << percent(worst_true) << "  (" << worst_true_label << ")\n"
            << "best pose 0.10 m / 1 deg   " << percent(best_near_miss_10cm) << "\n"
            << "best pose 0.25 m / 3 deg   " << percent(best_near_miss_25cm) << "\n"
            << "best pose 0.50 m / 5 deg   " << percent(best_near_miss_50cm) << "\n";

  if (!best_alias_pose.has_value())
  {
    // Not "0%": nothing was found at all. Printing a fraction here, with a pose to go beside it,
    // would read as a measured result for a sweep that never accepted a single candidate.
    std::cout << "strongest alias on the map none found\n\n"
                 "NOTHING SCORED. No candidate anywhere in the free space beat zero, so there is no alias\n"
                 "measurement and no separation to report. That is a data problem, not a map finding: check\n"
                 "that the scan samples carry usable returns inside the range filters and that the grid and\n"
                 "the scans came from the same run.\n";
    return EXIT_FAILURE;
  }

  std::cout << "strongest alias on the map " << percent(best_alias) << "  (" << best_alias_label << " at " << std::fixed
            << std::setprecision(2) << best_alias_pose->x << ", " << best_alias_pose->y << ", yaw "
            << best_alias_pose->yaw << ")\n"
            << "separation                 " << std::setprecision(1) << (separation * 100.0) << " points\n\n";

  if (separation <= 0.0)
  {
    std::cout << "NO THRESHOLD SEPARATES THEM. Somewhere on this map a wrong pose explains the scan at least as\n"
                 "well as the right one. Do not pick a number out of the overlap: that is a map finding, and the\n"
                 "answer is a tighter click bound or a short drive, not a lower gate.\n";
  }
  else
  {
    std::cout << "A threshold must sit strictly between " << percent(best_alias) << " and " << percent(worst_true)
              << ".\n";
  }
  return EXIT_SUCCESS;
}
