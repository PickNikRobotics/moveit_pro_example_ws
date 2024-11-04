#include <example_behaviors/hello_world.hpp>

namespace example_behaviors
{
HelloWorld::HelloWorld(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList HelloWorld::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList({});
}

BT::KeyValueVector HelloWorld::metadata()
{
  return { { "subcategory", "Example" }, { "description", "Log a message that says \"Hello, world!\"." } };
}

BT::NodeStatus HelloWorld::tick()
{
  // Do HelloWorld's useful work.
  // Setting the third argument to false ensures the message will be shown immediately
  shared_resources_->logger->publishInfoMessage(name(), "Hello, world!");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace example_behaviors
