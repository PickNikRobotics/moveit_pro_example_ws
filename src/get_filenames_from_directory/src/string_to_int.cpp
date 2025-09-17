#include <spdlog/spdlog.h>
#include <get_filenames_from_directory/string_to_int.hpp>
#include <stdexcept>
#include <string>

namespace
{

constexpr auto kPortIDString = "string_to_convert";
constexpr auto kPortIDOutput = "result";
}  // namespace

namespace string_to_int
{
StringToInt::StringToInt(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList StringToInt::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.
  return BT::PortsList({ BT::InputPort<std::string>(kPortIDString, "{string}", "The string to convert to an int"),
                         BT::OutputPort<int>(kPortIDOutput, "{result}", "The input string converted to an int") });
}

BT::KeyValueVector StringToInt::metadata()
{
  // TODO: Define your behavior here.
  return { { "description", "Convert a String to an Int" }, { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus StringToInt::tick()
{
  auto const input_string = getInput<std::string>(kPortIDString);
  if (const auto error = moveit_studio::behaviors::maybe_error(input_string); error)
  {
    spdlog::error("Failed to get required value from input data port: {}", error.value());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    setOutput(kPortIDOutput, std::stoi(input_string.value()));
    return BT::NodeStatus::SUCCESS;
  }
  catch (std::invalid_argument)
  {
    spdlog::error("Could not convert {} to an int", input_string.value());
    return BT::NodeStatus::FAILURE;
  }
  catch (std::out_of_range)
  {
    spdlog::error("{} is outside of the range representable by an int", input_string.value());
    return BT::NodeStatus::FAILURE;
  }
  catch (...)
  {
    // std::stoi only throws the above two, should not be possible to reach this
    spdlog::error("Unknown error");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace string_to_int
