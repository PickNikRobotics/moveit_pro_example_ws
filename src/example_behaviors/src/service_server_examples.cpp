#include <example_behaviors/service_server_examples.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace example_behaviors
{

TriggerServer::TriggerServer(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceServerBase(name, config, shared_resources)
{
}

BT::KeyValueVector TriggerServer::metadata()
{
  return { { "subcategory", "Examples" }, { "description", "Trigger service server example" } };
}

void TriggerServer::processRequest([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr& request,
                                       const std_srvs::srv::Trigger::Response::SharedPtr& response)
{
  response->success = true;
}

SetBoolServer::SetBoolServer(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceServerBase(name, config, shared_resources)
{
}
// Override to provide additional ports
BT::PortsList SetBoolServer::provideAdditionalPorts() 
{
      std::cout << "Providing additional ports for SetBoolServer" << std::endl;

  return {
    BT::OutputPort<bool>("result", "{result}", "A result for the behavior to put on the blackboard"),
  };
}

BT::KeyValueVector SetBoolServer::metadata()
{
  return { { "subcategory", "Examples" }, { "description", "SetBool service server example" } };
}

void SetBoolServer::processRequest([[maybe_unused]] const std_srvs::srv::SetBool::Request::SharedPtr& request,
                                       const std_srvs::srv::SetBool::Response::SharedPtr& response)
{
  setOutput("result", request->data);

  response->success = true;
}

} // namespace example_behaviors
