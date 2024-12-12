#include <future>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <example_behaviors/l2g.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tl_expected/expected.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>


#include "moveit_pro_ml/onnx_model.hpp"

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
      int num_points = points.size();
      const auto point_data = model->create_dynamic_tensor<float>(model->dynamic_inputs.at("point_cloud"),
                                                                  {1, num_points, 3});
      std::copy_n(points.data()->data(), num_points * 3, point_data);

      auto pred = model->predict_base(model->inputs, model->dynamic_inputs);

      // copy mask out and return reference
      auto shape = pred.at("predicted_grasps").onnx_shape;
      const auto predicted_grasps_data = model->get_tensor_data<float>(pred.at("predicted_grasps"));
      // const auto grasp_scores_data = model->get_tensor_data<float>(pred.at("grasp_scores"));

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
                                             "The Image to run segmentation on."),
      BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortGrasps, kPortGraspsDefault,
                                                                          "The masks contained in a vector of <code>moveit_studio_vision_msgs::msg::Mask2D</code> messages.")
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
    shared_resources_->logger->publishWarnMessage("Pointcloud size: " + filtered_cloud->size());
    double fraction = std::min(10000.0 / filtered_cloud->points.size(), 1.0);
    auto downsampled_cloud = moveit_studio::point_cloud_tools::downsampleRandom(filtered_cloud, fraction);
    shared_resources_->logger->publishWarnMessage("Downsampled pointcloud size: " + downsampled_cloud->size());

    try
    {
      std::vector<std::array<float, 3>> points;
      points.reserve(downsampled_cloud->points.size());
      for (auto& point : downsampled_cloud->points)
      {
        points.push_back({point.x, point.y, point.z});
      }

      const auto grasps = l2g_->predict(points);
      std::vector<geometry_msgs::msg::PoseStamped> grasps_pose;
      for (size_t i = 0; i < grasps.size(); i += 7)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = point_cloud_msg.header;
        pose.header.stamp = shared_resources_->node->now();

        pose.pose.position.x = (grasps[i + 0] + grasps[i + 3]) / 2.0;
        pose.pose.position.y = (grasps[i + 1] + grasps[i + 4]) / 2.0;
        pose.pose.position.z = (grasps[i + 2] + grasps[i + 5]) / 2.0;
        grasps_pose.push_back(pose);

        if (i >= 70)
        {
          break;
        }
      }

      setOutput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortGrasps, grasps_pose);
    }
    catch (const std::invalid_argument& e)
    {
      return tl::make_unexpected(fmt::format("Invalid argument: {}", e.what()));
    }

    return true;
  }

  BT::KeyValueVector L2GBehavior::metadata()
  {
    return {
      {
        "description",
        "Generate a grasp from a point cloud and output as a <code>geometry_msgs/PoseStamped</code> message."
      }
    };
  }
} // namespace sam2_segmentation
