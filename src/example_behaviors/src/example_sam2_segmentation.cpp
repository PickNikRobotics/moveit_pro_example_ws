#include <fmt/format.h>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <example_behaviors/example_sam2_segmentation.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_ml/data_types.hpp>
#include <moveit_pro_ml/sam2_segment.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tl_expected/expected.hpp>

namespace
{
constexpr auto kPortImage = "image";
constexpr auto kPortImageDefault = "{image}";
constexpr auto kPortPoint = "pixel_coords";
constexpr auto kPortPointDefault = "{pixel_coords}";
constexpr auto kPortMasks = "masks2d";
constexpr auto kPortMasksDefault = "{masks2d}";
constexpr auto kPortModelPackage = "model_package";
constexpr auto kPortModelPackageDefault = "moveit_pro_sam2";
constexpr auto kPortBundleManifest = "model_bundle_manifest";
constexpr auto kPortBundleManifestDefault = "models/model.yaml";
constexpr auto kPortRuntimeId = "runtime_id";
constexpr auto kPortRuntimeIdDefault = "onnxruntime";

constexpr auto kImageInferenceWidth = 1024;
constexpr auto kImageInferenceHeight = 1024;

/// Number of bytes per pixel each supported ROS encoding stores.
constexpr size_t kRgb8Channels = 3;
constexpr size_t kRgba8Channels = 4;
}  // namespace

namespace example_behaviors
{
namespace
{
/**
 * @brief Convert a ROS image message to the NHWC tensor the SAM2 pipeline consumes.
 * @details The source may carry an alpha channel; the tensor is always three-channel RGB with
 * values normalized to [0.0, 1.0].
 */
tl::expected<moveit_pro_ml::data::Tensor<float, moveit_pro_ml::data::format::NHWC>, std::string>
toImageTensor(const sensor_msgs::msg::Image& image_msg)
{
  namespace data = moveit_pro_ml::data;

  const size_t source_channels = image_msg.encoding == "rgb8" ? kRgb8Channels : kRgba8Channels;

  // The tensor's strong dimension types reject a non-positive extent by throwing, so catch an empty
  // image here and report it through the same error channel as every other malformed message.
  if (image_msg.height == 0 || image_msg.width == 0)
  {
    return tl::make_unexpected(
        fmt::format("Image message has a zero extent ({}x{})", image_msg.width, image_msg.height));
  }

  // Rows are `step` bytes apart, not necessarily tightly packed: a publisher may pad each row. Walking
  // by pixel would read padding as image data once step exceeds one row of pixels.
  const size_t row_bytes = static_cast<size_t>(image_msg.width) * source_channels;
  if (image_msg.step < row_bytes)
  {
    return tl::make_unexpected(fmt::format("Image message row stride {} is smaller than one {} row of {} pixels ({} "
                                           "bytes)",
                                           image_msg.step, image_msg.encoding, image_msg.width, row_bytes));
  }
  if (image_msg.data.size() < static_cast<size_t>(image_msg.height) * image_msg.step)
  {
    return tl::make_unexpected(fmt::format("Image message declares {}x{} {} with row stride {} but carries only {} "
                                           "bytes",
                                           image_msg.width, image_msg.height, image_msg.encoding, image_msg.step,
                                           image_msg.data.size()));
  }

  std::vector<float> values(static_cast<size_t>(image_msg.height) * image_msg.width * kRgb8Channels);
  for (size_t row = 0; row < image_msg.height; ++row)
  {
    for (size_t column = 0; column < image_msg.width; ++column)
    {
      const size_t source = row * image_msg.step + column * source_channels;
      const size_t destination = (row * image_msg.width + column) * kRgb8Channels;
      for (size_t channel = 0; channel < kRgb8Channels; ++channel)
      {
        values[destination + channel] = static_cast<float>(image_msg.data[source + channel]) / 255.0f;
      }
    }
  }

  auto tensor = data::Tensor<float, data::format::NHWC>::create(
      std::move(values), data::Batch{ 1 }, data::Channels{ static_cast<int64_t>(kRgb8Channels) },
      data::Extent{ data::Height{ static_cast<int64_t>(image_msg.height) },
                    data::Width{ static_cast<int64_t>(image_msg.width) } });
  if (!tensor.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to build the SAM2 input tensor: {}", tensor.error()));
  }
  return std::move(tensor).value();
}

/// @brief Convert a single-channel mask of probabilities to a ROS mask message.
moveit_studio_vision_msgs::msg::Mask2D
toMaskMessage(const moveit_pro_ml::data::Tensor<float, moveit_pro_ml::data::format::HW>& mask,
              const std_msgs::msg::Header& header)
{
  sensor_msgs::msg::Image mask_image_msg;
  mask_image_msg.header = header;
  mask_image_msg.height = static_cast<uint32_t>(mask.height());
  mask_image_msg.width = static_cast<uint32_t>(mask.width());
  mask_image_msg.encoding = "mono8";
  mask_image_msg.step = mask_image_msg.width;
  mask_image_msg.data.resize(mask.data.size());
  for (size_t i = 0; i < mask.data.size(); ++i)
  {
    mask_image_msg.data[i] = mask.data[i] > 0.5f ? 255 : 0;
  }

  moveit_studio_vision_msgs::msg::Mask2D mask_msg;
  mask_msg.pixels = std::move(mask_image_msg);
  mask_msg.x = 0;
  mask_msg.y = 0;
  return mask_msg;
}
}  // namespace

ExampleSAM2Segmentation::ExampleSAM2Segmentation(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList ExampleSAM2Segmentation::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::Image>(kPortImage, kPortImageDefault, "The Image to run segmentation on."),
           BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint, kPortPointDefault,
                                                                        "The input points, as a vector of "
                                                                        "<code>geometry_msgs/PointStamped</code> "
                                                                        "messages to be used for segmentation."),
           BT::InputPort<std::string>(kPortModelPackage, kPortModelPackageDefault,
                                      "ROS package containing the SAM 2 model bundle."),
           BT::InputPort<std::string>(kPortBundleManifest, kPortBundleManifestDefault,
                                      "Path to the SAM 2 model bundle manifest, relative to the model package's share "
                                      "directory. The manifest names every graph the pipeline loads."),
           BT::InputPort<std::string>(kPortRuntimeId, kPortRuntimeIdDefault,
                                      "Which <code>runtimes:</code> section of the bundle manifest to load."),

           BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(
               kPortMasks, kPortMasksDefault,
               "The masks contained in a vector of <code>moveit_studio_vision_msgs::msg::Mask2D</code> messages.") };
}

tl::expected<void, std::string> ExampleSAM2Segmentation::ensureLoaded(const std::filesystem::path& bundle_manifest,
                                                                      const std::string& runtime_id)
{
  if (sam2_.has_value())
  {
    if (bundle_manifest != loaded_bundle_manifest_ || runtime_id != loaded_runtime_id_)
    {
      return tl::make_unexpected(
          fmt::format("The SAM 2 model latched to bundle '{}' (runtime '{}') on its first run and cannot be reloaded "
                      "with bundle '{}' (runtime '{}'). Stop the Objective, change the ports, and run it again.",
                      loaded_bundle_manifest_.string(), loaded_runtime_id_, bundle_manifest.string(), runtime_id));
    }
    return {};
  }

  auto model = moveit_pro_ml::SAM2Segment::load(
      { .bundle_manifest = bundle_manifest, .runtime = moveit_pro_ml::model::RuntimeId{ runtime_id } });
  if (!model.has_value())
  {
    return tl::make_unexpected(
        fmt::format("Failed to load the SAM 2 model bundle '{}': {}", bundle_manifest.string(), model.error().message));
  }
  sam2_ = std::move(model).value();
  loaded_bundle_manifest_ = bundle_manifest;
  loaded_runtime_id_ = runtime_id;
  return {};
}

tl::expected<bool, std::string> ExampleSAM2Segmentation::doWork()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::Image>(kPortImage),
      getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint), getInput<std::string>(kPortModelPackage),
      getInput<std::string>(kPortBundleManifest), getInput<std::string>(kPortRuntimeId));

  // Check that all required input data ports were set.
  if (!ports.has_value())
  {
    auto error_message = fmt::format("Failed to get required values from input data ports:\n{}", ports.error());
    return tl::make_unexpected(error_message);
  }
  const auto& [image_msg, points_2d, model_package, bundle_manifest, runtime_id] = ports.value();

  if (image_msg.encoding != "rgb8" && image_msg.encoding != "rgba8")
  {
    auto error_message =
        fmt::format("Invalid image message format. Expected `(rgb8, rgba8)` got :\n{}", image_msg.encoding);
    return tl::make_unexpected(error_message);
  }

  std::filesystem::path manifest_path;
  try
  {
    manifest_path =
        std::filesystem::path{ ament_index_cpp::get_package_share_directory(model_package) } / bundle_manifest;
  }
  catch (const ament_index_cpp::PackageNotFoundError& e)
  {
    return tl::make_unexpected(fmt::format("Model package '{}' was not found: {}", model_package, e.what()));
  }

  if (const auto loaded = ensureLoaded(manifest_path, runtime_id); !loaded.has_value())
  {
    return tl::make_unexpected(loaded.error());
  }

  auto image_tensor = toImageTensor(image_msg);
  if (!image_tensor.has_value())
  {
    return tl::make_unexpected(image_tensor.error());
  }

  std::vector<moveit_pro_ml::data::Point> point_prompts;
  point_prompts.reserve(points_2d.size());
  for (auto const& point : points_2d)
  {
    // Assume all points are the same label
    point_prompts.push_back({ { kImageInferenceWidth * static_cast<float>(point.point.x),
                                kImageInferenceHeight * static_cast<float>(point.point.y) },
                              { 1.0f } });
  }

  const auto result =
      sam2_->predict({ .image = std::move(image_tensor).value(), .point_prompts = std::move(point_prompts) });
  if (!result.has_value())
  {
    return tl::make_unexpected(fmt::format("SAM 2 segmentation failed: {}", result.error().message));
  }

  setOutput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortMasks,
                                                                 { toMaskMessage(result->mask, image_msg.header) });

  return true;
}

BT::KeyValueVector ExampleSAM2Segmentation::metadata()
{
  return { { "description", "Segments a ROS image message using the provided points represented as a vector of "
                            "<code>geometry_msgs/PointStamped</code> messages." } };
}
}  // namespace example_behaviors
