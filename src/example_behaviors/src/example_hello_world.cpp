#include <example_behaviors/example_hello_world.hpp>

namespace example_behaviors
{
ExampleHelloWorld::ExampleHelloWorld(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ExampleHelloWorld::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList({});
}

BT::KeyValueVector ExampleHelloWorld::metadata()
{
  return { { "subcategory", "Example Behaviors" }, { "description", "Log a message that says \"Hello, world!\"." } };
}

BT::NodeStatus ExampleHelloWorld::tick()
{
  // Do ExampleHelloWorld's useful work.
  // Setting the third argument to false ensures the message will be shown immediately
  shared_resources_->logger->publishInfoMessage(name(), "Hello, world!");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace example_behaviors
