#include <spdlog/spdlog.h>
#include <get_filenames_from_directory/get_filenames_from_directory.hpp>

namespace
{

constexpr auto kPortIDDirectory = "directory_path";
constexpr auto kPortIDFileType = "file_type";
constexpr auto kPortIDOutput = "result";
}  // namespace

namespace get_filenames_from_directory
{
GetFilenamesFromDirectory::GetFilenamesFromDirectory(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList GetFilenamesFromDirectory::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.
  return BT::PortsList(
      { BT::InputPort<std::string>(kPortIDDirectory, "", "The path to the directory of interest"),
        BT::InputPort<std::string>(kPortIDFileType, ".yaml", "The optional file type to filter."),
        BT::OutputPort<std::vector<std::string>>(kPortIDOutput, "{result}", "Filenames in the given directory.") });
}

BT::KeyValueVector GetFilenamesFromDirectory::metadata()
{
  // TODO: Define your behavior here.
  return { { "description", "Gets a vector of all filenames in a given directory "
                            "(optionally with a given filetype)" },
           { "subcategory", "User Created Behaviors" } };
}

std::string to_lower(const std::string& s)
{
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
  return result;
}

std::vector<std::string> get_filenames(std::string const& dir_path, std::string const& extension = "")
{
  std::vector<std::string> filenames;
  auto const lower_extension = to_lower(extension);
  for (const auto& entry : std::filesystem::directory_iterator(dir_path))
  {
    if (entry.is_regular_file())
    {
      if (extension.empty() || to_lower(entry.path().extension().string()) == lower_extension)
      {
        filenames.push_back(entry.path().string());
      }
      else
      {
        if (!extension.empty())
        {
          spdlog::debug("Invalid extension: {}, expected {}", to_lower(entry.path().extension().string()),
                        lower_extension);
        }
      }
    }
  }
  return filenames;
}

BT::NodeStatus GetFilenamesFromDirectory::tick()
{
  auto const directory = getInput<std::string>(kPortIDDirectory);
  auto const file_type = getInput<std::string>(kPortIDFileType);
  if (const auto error = moveit_studio::behaviors::maybe_error(directory); error)
  {
    spdlog::error("Failed to get required value from input data port: {}", error.value());
    return BT::NodeStatus::FAILURE;
  }

  std::string extension = "";
  if (file_type.has_value())
  {
    extension = file_type.value();
  }
  setOutput(kPortIDOutput, get_filenames(directory.value(), extension));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace get_filenames_from_directory
