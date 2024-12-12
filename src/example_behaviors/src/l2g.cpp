#include <future>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_ml/onnx_model.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tl_expected/expected.hpp>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <example_behaviors/l2g.hpp>

namespace moveit_pro_ml
{
  class L2GModel
  {
  public:
    L2GModel(const std::filesystem::path& onnx_file)
      : model{std::make_shared<ONNXTensorModel>(onnx_file)}
    {
    }

    [[nodiscard]] std::vector<float> predict(const std::vector<std::array<float, 3>>& points) const
    {
      // move image into tensor
      const int num_points = points.size();
      const auto point_data = model->create_dynamic_tensor<float>(model->dynamic_inputs.at("point_cloud"),
                                                                  {1, num_points, 3});
      std::copy_n(points.data()->data(), num_points * 3, point_data);

      auto pred = model->predict_base(model->inputs, model->dynamic_inputs);

      // copy out predicted_grasps data
      const auto shape = pred.at("predicted_grasps").onnx_shape;
      const auto predicted_grasps_data = model->get_tensor_data<float>(pred.at("predicted_grasps"));

      // output format [center_x, center_y, center_z, qw, qx, qy, qz, grasp_width] x N
      return {predicted_grasps_data, predicted_grasps_data + get_size(shape)};
    }

    std::shared_ptr<ONNXTensorModel> model;
  };
} // namespace moveit_pro_ml


namespace
{
  constexpr auto kPortPointCloud = "point_cloud";
  constexpr auto kPortPointCloudDefault = "{point_cloud}";
  constexpr auto kPortGrasps = "grasps";
  constexpr auto kPortGraspsDefault = "{grasps}";
  constexpr auto kNumberOfPoints = 5000;
  constexpr auto kMaxGrasps = 10;
} // namespace

namespace example_behaviors
{
  L2GBehavior::L2GBehavior(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
    : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
  {
    const std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("example_behaviors");
    const std::filesystem::path onnx_file = package_path / "models" / "l2g.onnx";
    l2g_ = std::make_unique<moveit_pro_ml::L2GModel>(onnx_file);
  }

  BT::PortsList L2GBehavior::providedPorts()
  {
    return {
      BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortPointCloud, kPortPointCloudDefault,
                                                   "The point cloud to grasp. The point cloud should be approximately fit in a 0.22 meter cube."),
      BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortGrasps, kPortGraspsDefault,
                                                                          "The grasp poses in a vector of <code>geometry_msgs::msg::PoseStamped</code> messages.")
    };
  }

  tl::expected<bool, std::string> L2GBehavior::doWork()
  {
    const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::PointCloud2>(kPortPointCloud));

    // Check that all required input data ports were set.
    if (!ports.has_value())
    {
      auto error_message = fmt::format("Failed to get required values from input data ports:\n{}", ports.error());
      return tl::make_unexpected(error_message);
    }
    const auto& [point_cloud_msg] = ports.value();

    const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(point_cloud_msg, *cloud);
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::Indices index;
    pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, index);
    if (filtered_cloud->size() < kNumberOfPoints)
    {
      auto error_message = fmt::format("Point cloud must have at least {} points in it.", 5000);
      return tl::make_unexpected(error_message);
    }
    double fraction = std::min(static_cast<double>(kNumberOfPoints) / filtered_cloud->points.size(), 1.0);
    auto downsampled_cloud = moveit_studio::point_cloud_tools::downsampleRandom(filtered_cloud, fraction);

    // Subtract centroid from the point cloud
    pcl::PointXYZ centroid;
    computeCentroid(*downsampled_cloud, centroid);

    std::vector<std::array<float, 3>> points_centered;
    points_centered.reserve(downsampled_cloud->points.size());
    for (auto& point : downsampled_cloud->points)
    {
      points_centered.push_back({point.x - centroid.x, point.y - centroid.y, point.z - centroid.z});
    }

    // Filter out points outside the bounds defines by L2G.
    std::vector<std::array<float, 3>> points;
    points.reserve(downsampled_cloud->points.size());
    for (auto& point : points_centered)
    {
      if (abs(point[0]) > .22 / 2.0 || abs(point[1]) > .22 / 2.0 || abs(point[2]) > .22)
      {
        continue;
      }
      points.push_back({2.0f * point[0] / .22f, 2.0f * point[1] / .22f, point[2] / .22f});
    }

    // Run the network
    std::vector<float> grasps;
    try
    {
      grasps = l2g_->predict(points);
    }
    catch (const std::invalid_argument& e)
    {
      return tl::make_unexpected(fmt::format("Invalid argument: {}", e.what()));
    }

    // Copy of grasps into the output vector
    std::vector<geometry_msgs::msg::PoseStamped> grasps_pose;
    for (size_t i = 0; i < grasps.size(); i += 8)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = point_cloud_msg.header;
      pose.header.stamp = shared_resources_->node->now();

      pose.pose.position.x = grasps[i + 0] + centroid.x;
      pose.pose.position.y = grasps[i + 1] + centroid.y;
      pose.pose.position.z = grasps[i + 2] + centroid.z;

      pose.pose.orientation.w = grasps[i + 3];
      pose.pose.orientation.x = grasps[i + 4];
      pose.pose.orientation.y = grasps[i + 5];
      pose.pose.orientation.z = grasps[i + 6];
      grasps_pose.push_back(pose);

      // break after kMaxGrasps grasps have been added
      if (i >= 8*kMaxGrasps)
      {
        break;
      }
    }

    setOutput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortGrasps, grasps_pose);

    return true;
  }

  BT::KeyValueVector L2GBehavior::metadata()
  {
    return {
      {
        "description",
        "Generate a vector grasps from a point cloud using the L2G network."
      }
    };
  }
} // namespace example_behaviors
