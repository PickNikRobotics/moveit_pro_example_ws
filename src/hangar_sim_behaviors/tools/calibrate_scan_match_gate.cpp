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
 * It reports:
 *   - the WORST score any true pose gets;
 *   - the score of deliberately wrong poses on rings from 0.10 m out to the drift limit -- the ones
 *     beyond the refinement's own output are what a threshold must REJECT, the tightest one is a
 *     near-miss it must ACCEPT, and those pull the threshold in opposite directions;
 *   - the STRONGEST alias found anywhere on the map -- the best-scoring pose that is nowhere near
 *     the truth, found by sweeping the whole free space rather than by guessing where to look;
 *   - the band between the worst pose that must be rejected and the tightest one that must be
 *     accepted, which is the window the threshold has to live in.
 *
 * The rings matter as much as the map-wide sweep, because that sweep only starts at
 * `alias_keepout_m`. Everything closer than that is the region the drift gate is explicitly allowed
 * to move the belief into, so a band computed from the sweep alone names a safe threshold over
 * poses it never evaluated.
 *
 * WHAT THIS IS NOT. It is a SAMPLE, not a proof. The rings visit six discrete radii, sixteen
 * bearings and three yaw values at one magnitude per ring, so most of the accept region is never
 * evaluated: a pose 0.90 m out with a 20 degree heading error is inside the drift limit, the gate
 * can accept it, and no ring and no alias candidate scores it. Read the output as evidence about a
 * lattice of poses plus a swept search beyond the keepout, and do not read a clean band as a
 * guarantee that nothing admissible scores higher.
 *
 * A narrow band is a finding to report, not a number to split the difference on. It means the map
 * has structure that repeats, and the honest response is a tighter click and a shorter drive, not a
 * lower threshold.
 */

#include <hangar_sim_behaviors/calibration_io.hpp>
#include <hangar_sim_behaviors/localization_gates.hpp>

#include <algorithm>
#include <array>
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
  /// MINIMUM distance from the truth for a map-wide sweep candidate to count as an alias.
  ///
  /// The VALUE is fine. This is a floor on how far away a candidate has to be before the sweep is
  /// willing to call it an alias rather than a rediscovery of the truth, and kDriftLimit is a
  /// sensible floor: past the drift limit, a pose is unambiguously somewhere else.
  ///
  /// What does NOT follow from the value is any claim of coverage. Because it is a floor,
  /// `strongestAlias` skips everything CLOSER than this, so the map-wide sweep says nothing at all
  /// about the region inside it -- which is exactly the region the drift gate admits. The offset
  /// rings are what sample that region, and their maximum is folded into the reported band. Read
  /// this field as "where the alias search begins", never as "everything inside is accounted for".
  double alias_keepout_m = localization::kDriftLimit;
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

/// Map-frame centre of a grid cell. Every map this workspace ships has origin_yaw == 0, but the two
/// directions of this mapping have to agree on the convention or the sweep stands its candidates
/// somewhere other than the cells it says it is scoring.
std::pair<double, double> cellCentre(const calibration::GridDump& grid, int column, int row)
{
  const double cos_yaw = std::cos(grid.info.origin_yaw);
  const double sin_yaw = std::sin(grid.info.origin_yaw);
  const double local_x = (static_cast<double>(column) + 0.5) * grid.info.resolution;
  const double local_y = (static_cast<double>(row) + 0.5) * grid.info.resolution;
  return { grid.info.origin_x + cos_yaw * local_x - sin_yaw * local_y,
           grid.info.origin_y + sin_yaw * local_x + cos_yaw * local_y };
}

/**
 * @brief Can the robot actually stand here? True only for a KNOWN-FREE cell inside the grid.
 *
 * The one rule for where a candidate pose may be placed, so that every part of this tool agrees.
 * Standing the robot in a wall or in unknown space invents aliases no click could ever produce,
 * and since the reported band is what an operator sets min_inlier_fraction from, an invented alias
 * mis-sets the shipped gate. The rotate-then-floor here is the exact inverse of `cellCentre`.
 */
bool isKnownFree(const calibration::GridDump& grid, double x, double y)
{
  const double cos_yaw = std::cos(grid.info.origin_yaw);
  const double sin_yaw = std::sin(grid.info.origin_yaw);
  const double relative_x = x - grid.info.origin_x;
  const double relative_y = y - grid.info.origin_y;
  const double local_x = cos_yaw * relative_x + sin_yaw * relative_y;
  const double local_y = -sin_yaw * relative_x + cos_yaw * relative_y;
  const int column = static_cast<int>(std::floor(local_x / grid.info.resolution));
  const int row = static_cast<int>(std::floor(local_y / grid.info.resolution));
  if (column < 0 || column >= grid.info.width || row < 0 || row >= grid.info.height)
  {
    return false;
  }
  const auto index =
      static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.info.width) + static_cast<std::size_t>(column);
  return grid.data[index] == 0;
}

/// Cells the sweep may stand on: free, and not inside a wall.
std::vector<std::pair<double, double>> freePositions(const calibration::GridDump& grid, double stride)
{
  std::vector<std::pair<double, double>> positions;
  const int step = std::max(1, static_cast<int>(std::lround(stride / grid.info.resolution)));
  for (int row = 0; row < grid.info.height; row += step)
  {
    for (int column = 0; column < grid.info.width; column += step)
    {
      const auto [x, y] = cellCentre(grid, column, row);
      if (!isKnownFree(grid, x, y))
      {
        continue;
      }
      positions.emplace_back(x, y);
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
      // The SAME free-cell rule the coarse sweep applies. Without it the refinement could walk a
      // candidate into a wall or into unknown space and report it as the strongest alias on the
      // map -- a pose the robot cannot occupy and no click could produce.
      if (!isKnownFree(grid, x, y) || std::hypot(x - sample.truth_x, y - sample.truth_y) < settings.alias_keepout_m)
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

/**
 * @brief A ring of deliberately-wrong poses at a fixed offset from the truth.
 *
 * `must_reject` says which end of the band the ring constrains. A ring the gate has to REJECT sets
 * a floor the threshold must sit above, and the number that matters there is the BEST the ring can
 * score. A ring the gate has to ACCEPT -- the near-miss the refinement itself legitimately returns
 * -- sets a ceiling the threshold must sit below, and there the number that matters is the WORST.
 * Taking the best of a must-accept ring would publish a ceiling looser than the gate can honour.
 */
struct OffsetRing
{
  double offset_m;
  double offset_yaw;
  bool must_reject;
  const char* label;
};

/**
 * @brief Score one ring, reducing with `keep_best` or its opposite.
 *
 * Includes the ZERO-yaw member alongside the +/- yaw ones. A pure translation with the heading
 * still correct routinely outscores the same offset with a yaw error, so a ring that only tried
 * +/- yaw under-reported what a wrong pose at that radius can look like -- and it is the maximum
 * over the ring that a rejecting threshold has to beat.
 */
Scored reduceOverOffsetRing(const localization::DistanceField& field, const calibration::ScanSample& sample,
                            const SweepSettings& settings, const OffsetRing& ring, bool keep_best)
{
  std::optional<Scored> chosen;
  constexpr int kBearings = 16;
  for (int bearing_index = 0; bearing_index < kBearings; ++bearing_index)
  {
    const double bearing = 2.0 * M_PI * static_cast<double>(bearing_index) / static_cast<double>(kBearings);
    for (const double yaw_sign : { -1.0, 0.0, 1.0 })
    {
      const auto candidate = score(field, sample, settings, sample.truth_x + ring.offset_m * std::cos(bearing),
                                   sample.truth_y + ring.offset_m * std::sin(bearing),
                                   wrapAngle(sample.truth_yaw + yaw_sign * ring.offset_yaw));
      const bool better = !chosen.has_value() || (keep_best ? candidate.inlier_fraction > chosen->inlier_fraction :
                                                              candidate.inlier_fraction < chosen->inlier_fraction);
      if (better)
      {
        chosen = candidate;
      }
    }
  }
  return chosen.value_or(Scored{});
}

/**
 * @brief The rings, spanning from inside the refinement's own output out to the drift limit.
 *
 * The outer rings exist because the map-wide alias sweep starts at `alias_keepout_m` and so never
 * scores the region the drift gate admits. Without them the report named a safe band over poses it
 * had not evaluated: a wrong pose 0.9 m from the truth is inside the drift limit, so the gate can
 * accept it, and nothing in the report spoke to it.
 */
constexpr std::array<OffsetRing, 6> kOffsetRings = {
  OffsetRing{ 0.10, 1.0 * M_PI / 180.0, false, "0.10m" },  OffsetRing{ 0.25, 3.0 * M_PI / 180.0, true, "0.25m" },
  OffsetRing{ 0.50, 5.0 * M_PI / 180.0, true, "0.50m" },   OffsetRing{ 0.75, 7.0 * M_PI / 180.0, true, "0.75m" },
  OffsetRing{ 1.00, 9.0 * M_PI / 180.0, true, "1.00m" },   OffsetRing{ 1.15, 10.0 * M_PI / 180.0, true, "1.15m" },
};

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
            << "alias sweep: every free cell at least " << settings.alias_keepout_m
            << " m from the truth. That is a FLOOR, so the sweep says nothing about\n"
            << "  anything nearer; the offset rings below SAMPLE the region inside it, out to the drift limit\n"
            << "samples: " << samples.size() << "\n\n";

  double worst_true = 1.0;
  std::string worst_true_label;
  double best_alias = 0.0;
  std::string best_alias_label;
  std::optional<Scored> best_alias_pose;
  // One reduced figure per ring: the best a must-reject ring managed, the worst a must-accept ring
  // managed. Both are the number that constrains the threshold from that ring's side.
  std::array<double, kOffsetRings.size()> ring_result{};
  for (std::size_t i = 0; i < kOffsetRings.size(); ++i)
  {
    ring_result[i] = kOffsetRings[i].must_reject ? 0.0 : 1.0;
  }

  std::cout << std::left << std::setw(22) << "sample" << std::setw(8) << "beams" << std::setw(9) << "truth";
  for (const auto& ring : kOffsetRings)
  {
    std::cout << std::setw(9) << ring.label;
  }
  std::cout << "best alias\n";
  std::cout << std::string(105, '-') << "\n";

  for (const auto& sample : samples)
  {
    const auto truth = score(field, sample, settings, sample.truth_x, sample.truth_y, sample.truth_yaw);
    const auto alias = strongestAlias(field, grid, sample, settings);

    if (worst_true_label.empty() || truth.inlier_fraction < worst_true)
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

    std::cout << std::left << std::setw(22) << sample.label << std::setw(8) << truth.beams_used << std::setw(9)
              << percent(truth.inlier_fraction);
    for (std::size_t i = 0; i < kOffsetRings.size(); ++i)
    {
      const auto scored = reduceOverOffsetRing(field, sample, settings, kOffsetRings[i], kOffsetRings[i].must_reject);
      ring_result[i] = kOffsetRings[i].must_reject ? std::max(ring_result[i], scored.inlier_fraction) :
                                                     std::min(ring_result[i], scored.inlier_fraction);
      std::cout << std::setw(9) << percent(scored.inlier_fraction);
    }
    std::cout << (alias.has_value() ? percent(alias->inlier_fraction) : std::string("none")) << "\n";
  }

  // The floor a threshold must clear is the worst offender anywhere it can be admitted: the
  // strongest alias beyond the keepout, OR the strongest must-reject ring inside it. Leaving the
  // rings out published a band over poses the drift gate can accept but nothing had scored.
  double worst_reject = best_alias;
  const char* worst_reject_label = "strongest alias";
  double tightest_accept = worst_true;
  const char* tightest_accept_label = "worst true pose";
  for (std::size_t i = 0; i < kOffsetRings.size(); ++i)
  {
    if (kOffsetRings[i].must_reject)
    {
      if (ring_result[i] > worst_reject)
      {
        worst_reject = ring_result[i];
        worst_reject_label = kOffsetRings[i].label;
      }
    }
    else if (ring_result[i] < tightest_accept)
    {
      tightest_accept = ring_result[i];
      tightest_accept_label = kOffsetRings[i].label;
    }
  }

  // The window is bounded by the two CONSTRAINTS, not by the true pose. The tightest must-accept
  // ring is the real ceiling: a threshold above it rejects refinements the loop legitimately
  // returns, even though it still sits comfortably under the true pose's score.
  const double separation = tightest_accept - worst_reject;
  std::cout << "\n" << "worst true pose            " << percent(worst_true) << "  (" << worst_true_label << ")\n";
  for (std::size_t i = 0; i < kOffsetRings.size(); ++i)
  {
    std::cout << (kOffsetRings[i].must_reject ? "best pose  " : "worst pose ") << std::left << std::setw(15)
              << kOffsetRings[i].label << "  " << percent(ring_result[i])
              << (kOffsetRings[i].must_reject ? "  must be REJECTED\n" : "  must be ACCEPTED\n");
  }

  if (!best_alias_pose.has_value())
  {
    // Not "0%": nothing was found at all. Printing a fraction here, with a pose to go beside it,
    // would read as a measured result for a sweep that never accepted a single candidate.
    std::cout << "strongest alias on the map none found\n\n"
                 "NOTHING SCORED. No candidate anywhere in the free space beat zero, so there is no alias\n"
                 "measurement and no band to report. That is a data problem, not a map finding: check\n"
                 "that the scan samples carry usable returns inside the range filters and that the grid and\n"
                 "the scans came from the same run.\n";
    return EXIT_FAILURE;
  }

  std::cout << "strongest alias on the map " << percent(best_alias) << "  (" << best_alias_label << " at " << std::fixed
            << std::setprecision(2) << best_alias_pose->x << ", " << best_alias_pose->y << ", yaw "
            << best_alias_pose->yaw << ")\n"
            << "window                     " << std::setprecision(1) << (separation * 100.0) << " points\n\n";

  if (separation <= 0.0)
  {
    std::cout << "NO THRESHOLD SATISFIES BOTH CONSTRAINTS. The tightest pose the gate must ACCEPT ("
              << percent(tightest_accept) << ", " << tightest_accept_label << ")\n"
              << "scores no better than the worst pose it must REJECT (" << percent(worst_reject) << ", "
              << worst_reject_label << ").\n"
              << "Do not pick a number out of the overlap: that is a map and scan-density finding, and the answer\n"
                 "is a tighter click bound or a short drive, not a lower gate.\n";
    return EXIT_SUCCESS;
  }

  std::cout << "A threshold must sit strictly between " << percent(worst_reject) << " and "
            << percent(tightest_accept) << ".\n"
            << "Lower edge: the worst pose that must be REJECTED (" << worst_reject_label
            << "). Upper edge: the tightest pose\n"
            << "that must be ACCEPTED (" << tightest_accept_label
            << "). Those are opposite constraints, not a single direction,\n"
            << "so read the ring rows rather than averaging them. Note the true pose's own score is NOT the upper\n"
               "edge: a threshold under it can still reject the near-misses this refinement legitimately returns.\n\n"
            << "The lower edge folds in BOTH the map-wide alias sweep (which starts " << std::fixed
            << std::setprecision(2) << settings.alias_keepout_m << " m out) and the offset\n"
            << "rings inside that keepout, which run to the drift limit. Those rings are a SAMPLE of the accept\n"
               "region -- six radii, sixteen bearings, three yaw values each -- not a proof about all of it. A\n"
               "pose at an unsampled offset and heading inside the drift limit can be admitted without appearing\n"
               "anywhere above.\n";
  return EXIT_SUCCESS;
}
