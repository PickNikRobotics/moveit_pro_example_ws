// Copyright 2026 PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <hardware_interface/component_parser.hpp>
#include <picknik_mujoco_ros/cameras.hpp>
#include <picknik_mujoco_ros/lidar_noise.hpp>
#include <rclcpp/logger.hpp>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: market_sim_test_livox_sensor_probe SCENE_XML EXPANDED_URDF\n";
    return 2;
  }
  const auto plugin = ament_index_cpp::get_package_prefix("mujoco_package_uri") +
                      "/bin/mujoco_plugin/libmujoco_package_uri_resource_provider.so";
  mj_loadPluginLibrary(plugin.c_str());
  const auto decoders = ament_index_cpp::get_package_prefix("picknik_mujoco") + "/bin/mujoco_plugin/";
  mj_loadPluginLibrary((decoders + "libobj_decoder.so").c_str());
  mj_loadPluginLibrary((decoders + "libstl_decoder.so").c_str());
  char error[1024] = {};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadXML(argv[1], nullptr, error, sizeof(error)),
                                                            mj_deleteModel);
  if (!model)
  {
    std::cerr << error << '\n';
    return 1;
  }
  // The real consumers interpret both scene and ros2_control declaration, including noise.
  auto cameras = picknik_mujoco_ros::extract_cameras(model.get());
  std::ifstream urdf_file(argv[2]);
  if (!urdf_file)
  {
    std::cerr << "Could not read expanded URDF: " << argv[2] << '\n';
    return 1;
  }
  const std::string urdf{ std::istreambuf_iterator<char>(urdf_file), std::istreambuf_iterator<char>() };
  const auto hardware = hardware_interface::parse_control_resources_from_urdf(urdf);
  for (const auto& component : hardware)
  {
    picknik_mujoco_ros::apply_camera_sensor_overrides(cameras, component.sensors, rclcpp::get_logger("livox_probe"));
  }
  const auto found =
      std::find_if(cameras.begin(), cameras.end(), [](const auto& camera) { return camera.name == "livox_mid360"; });
  if (found == cameras.end() || !found->lidar_params.has_value())
  {
    std::cerr << "No native livox_mid360 lidar camera\n";
    return 1;
  }
  const auto& config = *found;
  const auto& lidar = config.lidar_params.value();
  std::cout << "config " << config.width << ' ' << config.height << ' ' << config.fov_y << ' ' << lidar.fov_x << ' '
            << lidar.range_min << ' ' << lidar.range_max << ' ' << lidar.noise.range_stddev << ' '
            << lidar.noise.dropout_probability << '\n';
  for (const auto& camera : cameras)
  {
    if (!camera.geom_groups.has_value())
      continue;
    std::cout << "groups " << camera.name;
    for (const auto enabled : camera.geom_groups.value())
      std::cout << ' ' << static_cast<int>(enabled);
    std::cout << '\n';
  }

  // A synthetic spherical depth shell at r=3 m. Exercise the installed sweep stitcher
  // and radial noise implementation, rather than looking for parameter strings in XML.
  picknik_mujoco_ros::LidarSweep sweep;
  const int width = model->vis.global.offwidth;
  const int height = model->vis.global.offheight;
  for (std::size_t index = 0; index < lidar.num_images; ++index)
  {
    picknik_mujoco_ros::LidarTile tile;
    tile.width = width;
    tile.height = height;
    tile.z_buffer.resize(static_cast<std::size_t>(width) * height);
    for (int row = 0; row < height; ++row)
    {
      for (int col = 0; col < width; ++col)
      {
        const double x = (col - width * 0.5) / lidar.render_f;
        const double y = (row - height * 0.5) / lidar.render_f;
        const double depth = 3.0 / std::sqrt(1.0 + x * x + y * y);
        const double z_buffer = (1.0 - config.z_near / depth) / (1.0 - config.z_near / config.z_far);
        tile.z_buffer[static_cast<std::size_t>(row) * width + col] = static_cast<float>(z_buffer);
      }
    }
    sweep.tiles.push_back(std::move(tile));
  }
  auto noise_params = lidar.noise;
  noise_params.seed = 360;
  noise_params.dropout_probability = 0.0f;  // Check geometric coverage before intentional missing returns.
  picknik_mujoco_ros::LidarNoiseGenerator noise(noise_params);
  sensor_msgs::msg::PointCloud2 cloud;
  picknik_mujoco_ros::construct_point_cloud_from_lidar_sweep(config, sweep, noise, cloud);
  constexpr std::size_t first_row = 63;
  constexpr std::size_t last_row = 123;
  std::size_t finite = 0;
  double sum = 0.0;
  double sum_sq = 0.0;
  double min_elevation = 90.0;
  double max_elevation = -90.0;
  for (std::size_t point = first_row * cloud.width; point < last_row * cloud.width; ++point)
  {
    float xyz[3];
    std::memcpy(xyz, cloud.data.data() + point * cloud.point_step, sizeof(xyz));
    const double range = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    if (!std::isfinite(range))
      continue;
    ++finite;
    const double residual = range - 3.0;
    sum += residual;
    sum_sq += residual * residual;
    const double elevation = std::atan2(-xyz[1], std::hypot(xyz[0], xyz[2])) * 180.0 / mjPI;
    min_elevation = std::min(min_elevation, elevation);
    max_elevation = std::max(max_elevation, elevation);
  }
  const double mean = sum / static_cast<double>(finite);
  std::cout << "shell " << finite << ' ' << mean << ' ' << std::sqrt(sum_sq / static_cast<double>(finite) - mean * mean)
            << ' ' << min_elevation << ' ' << max_elevation << '\n';
  noise_params.dropout_probability = lidar.noise.dropout_probability;
  picknik_mujoco_ros::LidarNoiseGenerator dropout_noise(noise_params);
  picknik_mujoco_ros::construct_point_cloud_from_lidar_sweep(config, sweep, dropout_noise, cloud);
  std::size_t valid_with_dropouts = 0;
  for (std::size_t point = first_row * cloud.width; point < last_row * cloud.width; ++point)
  {
    float x = 0.0f;
    std::memcpy(&x, cloud.data.data() + point * cloud.point_step, sizeof(x));
    valid_with_dropouts += std::isfinite(x);
  }
  std::cout << "dropouts " << valid_with_dropouts << '\n';
  return 0;
}
