// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/localization_gates.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>

namespace hangar_sim_behaviors::localization
{
namespace
{
/// Row-major linear index, matching beluga's BaseLinearGrid2::index_at.
std::size_t indexAt(const GridInfo& info, std::size_t column, std::size_t row)
{
  return row * static_cast<std::size_t>(info.width) + column;
}
}  // namespace

double seedReachRadius(double xy_std_dev)
{
  return kSeedReachSigmas * xy_std_dev;
}

bool DistanceField::valid() const
{
  return info.width > 0 && info.height > 0 && info.resolution > 0.0 &&
         distance.size() == static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
}

float DistanceField::at(double x, double y) const
{
  if (!valid())
  {
    return static_cast<float>(max_distance);
  }
  // Into the grid's own frame first: beluga holds the grid origin as an SE2 and applies its inverse
  // before indexing. Every map this workspace ships has origin_yaw == 0, so the rotation is usually
  // the identity, but a rotated map would silently mis-index without it.
  const double dx = x - info.origin_x;
  const double dy = y - info.origin_y;
  const double cos_yaw = std::cos(info.origin_yaw);
  const double sin_yaw = std::sin(info.origin_yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  // beluga's cell_near: floor(local / resolution). NOT nav2's round-to-nearest-centre.
  const int column = static_cast<int>(std::floor(local_x / info.resolution));
  const int row = static_cast<int>(std::floor(local_y / info.resolution));
  if (column < 0 || column >= info.width || row < 0 || row >= info.height)
  {
    return static_cast<float>(max_distance);
  }
  return distance[indexAt(info, static_cast<std::size_t>(column), static_cast<std::size_t>(row))];
}

DistanceField buildDistanceField(const GridInfo& info, const std::vector<std::int8_t>& occupancy, double max_distance)
{
  DistanceField field;
  if (info.width <= 0 || info.height <= 0 || info.resolution <= 0.0 || max_distance <= 0.0 ||
      occupancy.size() != static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height))
  {
    return field;
  }

  const auto width = static_cast<std::size_t>(info.width);
  const auto height = static_cast<std::size_t>(info.height);
  const std::size_t cell_count = width * height;

  // beluga works in squared distances throughout and only takes the root when it turns the field
  // into likelihoods, so the comparisons below are squared too.
  const double squared_max = max_distance * max_distance;
  const double squared_resolution = info.resolution * info.resolution;

  // Squared distance between two cell centroids, in metres^2. beluga computes this from
  // coordinates_at(), which places a centroid at (index + 0.5) * resolution; the half-cell offset
  // is common to both cells and cancels, leaving the cell-index delta.
  const auto squared_distance = [width, squared_resolution](std::size_t from, std::size_t to) {
    const auto from_column = static_cast<std::ptrdiff_t>(from % width);
    const auto from_row = static_cast<std::ptrdiff_t>(from / width);
    const auto to_column = static_cast<std::ptrdiff_t>(to % width);
    const auto to_row = static_cast<std::ptrdiff_t>(to / width);
    const auto d_column = static_cast<double>(to_column - from_column);
    const auto d_row = static_cast<double>(to_row - from_row);
    return (d_column * d_column + d_row * d_row) * squared_resolution;
  };

  // beluga's neighborhood4, in its own push order: +x, +y, -x, -y. Ties in the queue break on that
  // order, so keeping it keeps the field identical rather than merely equivalent.
  const auto for_each_neighbour = [width, height](std::size_t index, const auto& visit) {
    const std::size_t column = index % width;
    const std::size_t row = index / width;
    if (column + 1 < width)
    {
      visit(index + 1);
    }
    if (row + 1 < height)
    {
      visit(index + width);
    }
    if (column > 0)
    {
      visit(index - 1);
    }
    if (row > 0)
    {
      visit(index - width);
    }
  };

  std::vector<double> squared(cell_count, squared_max);
  std::vector<bool> visited(cell_count, false);

  struct IndexPair
  {
    std::size_t nearest_obstacle_index;
    std::size_t index;
  };
  const auto compare = [&squared](const IndexPair& first, const IndexPair& second) {
    return squared[first.index] > squared[second.index];
  };
  std::priority_queue<IndexPair, std::vector<IndexPair>, decltype(compare)> queue{ compare };

  for (std::size_t index = 0; index < cell_count; ++index)
  {
    // beluga's trinary is_occupied() is an equality test against 100, not a threshold against
    // occupied_thresh. For a `mode: trinary` map_server -- which is what hangar_map.yaml asks for --
    // the two agree, because map_server has already collapsed every cell to 0 / 100 / -1. For a
    // `scale` map they disagree completely, and mirroring beluga is the point of this file.
    if (occupancy[index] == kOccupiedValue)
    {
      visited[index] = true;
      squared[index] = 0.0;
      queue.push(IndexPair{ index, index });
    }
  }

  while (!queue.empty())
  {
    const IndexPair parent = queue.top();
    queue.pop();
    for_each_neighbour(parent.index, [&](std::size_t neighbour) {
      if (visited[neighbour])
      {
        return;
      }
      visited[neighbour] = true;
      const double distance_squared = squared_distance(parent.nearest_obstacle_index, neighbour);
      if (distance_squared < squared_max)
      {
        squared[neighbour] = distance_squared;
        queue.push(IndexPair{ parent.nearest_obstacle_index, neighbour });
      }
      // Past the truncation beluga neither records nor expands, so the wavefront stops here. The
      // cell keeps its initial squared_max, which is exactly what the filter sees.
    });
  }

  field.info = info;
  field.max_distance = max_distance;
  field.distance.resize(cell_count);
  std::transform(squared.begin(), squared.end(), field.distance.begin(),
                 [max_distance](double value) { return static_cast<float>(std::min(std::sqrt(value), max_distance)); });
  return field;
}

std::vector<std::size_t> takeEvenlyIndices(std::size_t range_count, std::size_t max_beams)
{
  std::vector<std::size_t> indices;
  if (range_count == 0 || max_beams == 0)
  {
    return indices;
  }
  if (max_beams >= range_count)
  {
    // beluga hands back every element rather than padding.
    indices.resize(range_count);
    std::iota(indices.begin(), indices.end(), std::size_t{ 0 });
    return indices;
  }

  indices.reserve(max_beams);
  const std::size_t divisor = max_beams - 1;
  const std::size_t span = range_count - 1;
  for (std::size_t position = 0; position < max_beams; ++position)
  {
    if (position == 0)
    {
      indices.push_back(0);
      continue;
    }
    // Ceiling division, as beluga's compute_offset does it. The last position lands exactly on
    // range_count - 1, so the end of the scan is always scored.
    const std::size_t numerator = position * span;
    indices.push_back(numerator / divisor + ((numerator % divisor == 0) ? 0 : 1));
  }
  return indices;
}

ResidualStats computeScanResidual(const DistanceField& field, const ScanGeometry& geometry,
                                  const std::vector<float>& ranges, int max_beams, double min_range, double max_range,
                                  double inlier_distance, double x, double y, double yaw)
{
  ResidualStats stats;
  stats.median_residual = field.valid() ? field.max_distance : kMaxObstacleDistance;
  if (!field.valid() || ranges.empty() || max_beams < 1)
  {
    return stats;
  }

  // beluga clamps the scan's own limits with the configured ones and keeps everything in between,
  // both ends inclusive.
  const double effective_min = std::max(geometry.range_min, min_range);
  const double effective_max = std::min(geometry.range_max, max_range);

  const auto indices = takeEvenlyIndices(ranges.size(), static_cast<std::size_t>(max_beams));
  stats.beams_selected = static_cast<int>(indices.size());

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<float> residuals;
  residuals.reserve(indices.size());
  for (const std::size_t index : indices)
  {
    const double range = static_cast<double>(ranges[index]);
    // beluga filters on isnan alone; an infinite return is dropped by the max_range test below,
    // which is how a no-return beam gets excluded.
    if (std::isnan(range) || range < effective_min || range > effective_max)
    {
      continue;
    }
    const double bearing = geometry.angle_min + static_cast<double>(index) * geometry.angle_increment;
    const double sensor_x = range * std::cos(bearing);
    const double sensor_y = range * std::sin(bearing);
    const double hit_x = x + cos_yaw * sensor_x - sin_yaw * sensor_y;
    const double hit_y = y + sin_yaw * sensor_x + cos_yaw * sensor_y;
    residuals.push_back(field.at(hit_x, hit_y));
  }

  stats.beams_used = static_cast<int>(residuals.size());
  if (residuals.empty())
  {
    return stats;
  }

  const auto inliers = std::count_if(residuals.begin(), residuals.end(), [inlier_distance](float residual) {
    return static_cast<double>(residual) < inlier_distance;
  });
  stats.inlier_fraction = static_cast<double>(inliers) / static_cast<double>(residuals.size());

  const std::size_t middle = residuals.size() / 2;
  std::nth_element(residuals.begin(), residuals.begin() + static_cast<std::ptrdiff_t>(middle), residuals.end());
  stats.median_residual = static_cast<double>(residuals[middle]);
  return stats;
}

}  // namespace hangar_sim_behaviors::localization
