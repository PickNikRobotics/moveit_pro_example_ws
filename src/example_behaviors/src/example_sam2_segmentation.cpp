#include <fmt/format.h>
#include <future>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <example_behaviors/example_sam2_segmentation.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_ml/onnx_sam2.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tl_expected/expected.hpp>

namespace
{
constexpr auto kPortImage = "image";
constexpr auto kPortImageDefault = "{image}";
constexpr auto kPortPoint = "pixel_coords";
constexpr auto kPortPointDefault = "{pixel_coords}";
constexpr auto kPortMasks = "masks2d";
constexpr auto kPortMasksDefault = "{masks2d}";

constexpr auto kImageInferenceWidth = 1024;
constexpr auto kImageInferenceHeight = 1024;
}  // namespace

namespace example_behaviors
{
// Convert a ROS image message to the ONNX image format used by the SAM 2 model
void set_onnx_image_from_ros_image(const sensor_msgs::msg::Image& image_msg, moveit_pro_ml::ONNXImage& onnx_image)
{
  onnx_image.shape = { 1, image_msg.height, image_msg.width, 3 };
  onnx_image.data.resize(image_msg.height * image_msg.width * 3);
  const int stride = image_msg.encoding != "rgb8" ? 3 : 4;
  for (size_t i = 0; i < onnx_image.data.size(); i += stride)
  {
    onnx_image.data[i] = static_cast<float>(image_msg.data[i]) / 255.0f;
    onnx_image.data[i + 1] = static_cast<float>(image_msg.data[i + 1]) / 255.0f;
    onnx_image.data[i + 2] = static_cast<float>(image_msg.data[i + 2]) / 255.0f;
  }
}

// Converts a single channel ONNX image mask to a ROS mask message.
void set_ros_mask_from_onnx_mask(const moveit_pro_ml::ONNXImage& onnx_image, sensor_msgs::msg::Image& mask_image_msg,
                                 moveit_studio_vision_msgs::msg::Mask2D& mask_msg)
{
  mask_image_msg.height = static_cast<uint32_t>(onnx_image.shape[0]);
  mask_image_msg.width = static_cast<uint32_t>(onnx_image.shape[1]);
  mask_image_msg.encoding = "mono8";
  mask_image_msg.data.resize(mask_image_msg.height * mask_image_msg.width);
  mask_image_msg.step = mask_image_msg.width;
  for (size_t i = 0; i < onnx_image.data.size(); ++i)
  {
    mask_image_msg.data[i] = onnx_image.data[i] > 0.5 ? 255 : 0;
  }
  mask_msg.pixels = mask_image_msg;
  mask_msg.x = 0;
  mask_msg.y = 0;
}

ExampleSAM2Segmentation::ExampleSAM2Segmentation(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
  const std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("example_behaviors");
  const std::filesystem::path encoder_onnx_file = package_path / "models" / "sam2_hiera_large_encoder.onnx";
  const std::filesystem::path decoder_onnx_file = package_path / "models" / "decoder.onnx";
  sam2_ = std::make_unique<moveit_pro_ml::SAM2>(encoder_onnx_file, decoder_onnx_file);
}

BT::PortsList ExampleSAM2Segmentation::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::Image>(kPortImage, kPortImageDefault, "The Image to run segmentation on."),
           BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint, kPortPointDefault,
                                                                        "The input points, as a vector of "
                                                                        "<code>geometry_msgs/PointStamped</code> "
                                                                        "messages to be used for segmentation."),

           BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(
               kPortMasks, kPortMasksDefault,
               "The masks contained in a vector of <code>moveit_studio_vision_msgs::msg::Mask2D</code> messages.") };
}

tl::expected<bool, std::string> ExampleSAM2Segmentation::doWork()
{
  const auto ports =
      moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortImage),
                                                  getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint));

  // Check that all required input data ports were set.
  if (!ports.has_value())
  {
    auto error_message = fmt::format("Failed to get required values from input data ports:\n{}", ports.error());
    return tl::make_unexpected(error_message);
  }
  const auto& [image_msg, points_2d] = ports.value();

  if (image_msg.encoding != "rgb8" && image_msg.encoding != "rgba8")
  {
    auto error_message =
        fmt::format("Invalid image message format. Expected `(rgb8, rgba8)` got :\n{}", image_msg.encoding);
    return tl::make_unexpected(error_message);
  }

  // Create ONNX formatted image tensor from ROS image
  set_onnx_image_from_ros_image(image_msg, onnx_image_);

  std::vector<moveit_pro_ml::PointPrompt> point_prompts;
  for (auto const& point : points_2d)
  {
    // Assume all points are the same label
    point_prompts.push_back({ { kImageInferenceWidth * static_cast<float>(point.point.x),
                                kImageInferenceHeight * static_cast<float>(point.point.y) },
                              { 1.0f } });
  }

  try
  {
    const auto masks = sam2_->predict(onnx_image_, point_prompts);

    mask_image_msg_.header = image_msg.header;
    set_ros_mask_from_onnx_mask(masks, mask_image_msg_, mask_msg_);

    setOutput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortMasks, { mask_msg_ });
  }
  catch (const std::invalid_argument& e)
  {
    return tl::make_unexpected(fmt::format("Invalid argument: {}", e.what()));
  }

  return true;
}

BT::KeyValueVector ExampleSAM2Segmentation::metadata()
{
  return { { "description", "Segments a ROS image message using the provided points represented as a vector of "
                            "<code>geometry_msgs/PointStamped</code> messages." } };
}
}  // namespace example_behaviors
