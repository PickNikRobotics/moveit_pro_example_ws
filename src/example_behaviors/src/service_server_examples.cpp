#include <example_behaviors/service_server_examples.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>

constexpr auto kInputTriggerTopicName = "my_trigger_service";
constexpr auto kInputSetBoolTopicName = "my_set_bool_service";

namespace example_behaviors
{

TriggerServer::TriggerServer(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceServerBase(name, config, shared_resources, kInputTriggerTopicName)
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
  : ServiceServerBase(name, config, shared_resources, kInputSetBoolTopicName)
{
}

BT::PortsList SetBoolServer::providedPorts()
{
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
