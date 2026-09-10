// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/calibration_io.hpp>

#include <fmt/format.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace hangar_sim_behaviors::calibration
{
namespace
{
/// Read the next non-blank, non-comment line, or return false at end of input.
bool nextLine(std::istream& input, std::string& line)
{
  while (std::getline(input, line))
  {
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#')
    {
      continue;
    }
    return true;
  }
  return false;
}

/// Read a "<key> <value>" line and check the key is the one expected.
template <typename T>
T expectKeyed(std::istream& input, const std::string& key)
{
  std::string line;
  if (!nextLine(input, line))
  {
    throw std::runtime_error(fmt::format("expected '{}' but the input ended", key));
  }
  std::istringstream stream(line);
  std::string found;
  T value{};
  if (!(stream >> found >> value) || found != key)
  {
    throw std::runtime_error(fmt::format("expected '{} <value>', got '{}'", key, line));
  }
  return value;
}
}  // namespace

GridDump readGridDump(std::istream& input)
{
  GridDump dump;
  dump.info.width = expectKeyed<int>(input, "width");
  dump.info.height = expectKeyed<int>(input, "height");
  dump.info.resolution = expectKeyed<double>(input, "resolution");
  dump.info.origin_x = expectKeyed<double>(input, "origin_x");
  dump.info.origin_y = expectKeyed<double>(input, "origin_y");
  dump.info.origin_yaw = expectKeyed<double>(input, "origin_yaw");

  std::string line;
  if (!nextLine(input, line) || line.find("data") == std::string::npos)
  {
    throw std::runtime_error("expected a 'data' marker before the cell values");
  }

  if (dump.info.width <= 0 || dump.info.height <= 0)
  {
    throw std::runtime_error(fmt::format("grid dimensions must be positive, got {} x {}", dump.info.width,
                                         dump.info.height));
  }
  const auto expected =
      static_cast<std::size_t>(dump.info.width) * static_cast<std::size_t>(dump.info.height);
  dump.data.reserve(expected);
  int value = 0;
  while (input >> value)
  {
    dump.data.push_back(static_cast<std::int8_t>(value));
  }
  if (dump.data.size() != expected)
  {
    throw std::runtime_error(fmt::format("grid says {} x {} = {} cells but carries {}", dump.info.width,
                                         dump.info.height, expected, dump.data.size()));
  }
  return dump;
}

std::vector<ScanSample> readScanSamples(std::istream& input)
{
  std::vector<ScanSample> samples;
  std::string line;
  while (nextLine(input, line))
  {
    if (line.find("scan") != 0)
    {
      throw std::runtime_error(fmt::format("expected a 'scan' marker to open a sample, got '{}'", line));
    }
    ScanSample sample;
    {
      std::string found;
      std::istringstream stream(line);
      stream >> found >> sample.label;
    }
    sample.geometry.angle_min = expectKeyed<double>(input, "angle_min");
    sample.geometry.angle_increment = expectKeyed<double>(input, "angle_increment");
    sample.geometry.range_min = expectKeyed<double>(input, "range_min");
    sample.geometry.range_max = expectKeyed<double>(input, "range_max");
    sample.truth_x = expectKeyed<double>(input, "truth_x");
    sample.truth_y = expectKeyed<double>(input, "truth_y");
    sample.truth_yaw = expectKeyed<double>(input, "truth_yaw");

    if (!nextLine(input, line))
    {
      throw std::runtime_error("sample ended before its ranges");
    }
    std::istringstream stream(line);
    std::string found;
    std::size_t count = 0;
    if (!(stream >> found >> count) || found != "ranges")
    {
      throw std::runtime_error(fmt::format("expected 'ranges <count> <values...>', got '{}'", line));
    }
    sample.ranges.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
      std::string token;
      if (!(stream >> token))
      {
        throw std::runtime_error(fmt::format("sample '{}' promised {} ranges but carries {}", sample.label, count,
                                             sample.ranges.size()));
      }
      // "inf" and "nan" are written literally so a no-return stays a no-return through the round
      // trip; writing them as a large finite number would quietly turn them into scored beams.
      sample.ranges.push_back(std::stof(token));
    }
    samples.push_back(std::move(sample));
  }
  return samples;
}

}  // namespace hangar_sim_behaviors::calibration
