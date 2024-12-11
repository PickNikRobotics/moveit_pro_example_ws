#include <graspnet_example_behavior/graspnet_example_behavior.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>

namespace
{
  inline constexpr auto kDescriptionGraspNetExampleBehavior = R"(
                <p>
                    To do
                </p>  
                  )";
  constexpr auto kPortIDPointCloud = "point_cloud";
  constexpr auto kPortIDPCDTopicName = "pcd_topic";
  constexpr auto kPortIDStampedPose = "stamped_pose";
  constexpr auto kPortIDNumSamples = "num_samples";
} //namespace

namespace graspnet_example_behavior
{
GraspnetExampleBehavior::GraspnetExampleBehavior(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList GraspnetExampleBehavior::providedPorts()
{
  // TODO(...)
  return { BT::InputPort<std::string>(kPortIDPCDTopicName, "/pcd_pointcloud_captures",
                                      "Topic the pcd formatted point cloud is published to."),
          BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}",
                                                        "Point cloud in sensor_msgs::msg::PointCloud2 format."),
          BT::InputPort<int>(kPortIDNumSamples, "{num_samples}", "Number of samples (poses) to take from GraspNet."),
          BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDStampedPose, "{pose_stamped_vector}",
                                                           "The geometry_msgs/PoseStamped message output.") };
}

BT::KeyValueVector GraspnetExampleBehavior::metadata()
{
  
  return { { "subcategory", "Grasping"}, {"description", kDescriptionGraspNetExampleBehavior} };
}

BT::NodeStatus GraspnetExampleBehavior::tick()
{
  using namespace moveit_studio::behaviors;
  // Return SUCCESS once the work has been completed.
  const auto ports = getRequiredInputs(getInput<std::string>(kPortIDPCDTopicName), 
                                        getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud),
                                        getInput<int>(kPortIDNumSamples));
  
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pc_topic, pointcloud, num_samples] = ports.value();
  std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vector;

  std::random_device r;
  std::mt19937 el(r());
  std::uniform_real_distribution<double> uniform_dist(-0.75, 0.75);

  geometry_msgs::msg::PoseStamped to_edit;
  for (int i=0; i<num_samples; ++i) 
  {
    to_edit.header.frame_id = pointcloud.header.frame_id;
    to_edit.header.stamp = shared_resources_->node->now();

    to_edit.pose.position.x = uniform_dist(el);
    to_edit.pose.position.y = uniform_dist(el);
    to_edit.pose.position.z = uniform_dist(el);

    to_edit.pose.orientation.x = 0.0;
    to_edit.pose.orientation.y = 0.0;
    to_edit.pose.orientation.z = 0.0;
    to_edit.pose.orientation.w = 1.0;

    pose_stamped_vector.push_back(to_edit);
  }

  setOutput(kPortIDStampedPose, pose_stamped_vector);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace graspnet_example_behavior
