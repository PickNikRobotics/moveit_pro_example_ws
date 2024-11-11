#ifndef SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_IMPL_HPP_
#define SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_IMPL_HPP_

#include "service_server_base.hpp"
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <spdlog/spdlog.h>

namespace example_behaviors
{

// Template definition for the ServiceServerBase class with a specific ServiceType
template<typename ServiceType>
ServiceServerBase<ServiceType>::ServiceServerBase(
    const std::string& name, // The name of the behavior node
    const BT::NodeConfiguration& config, // Configuration for the behavior tree node
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) // Shared resources across behaviors
  : AsyncBehaviorBase(name, config, shared_resources),
    cancel_behavior_(false), data_available_(false)
{
}

// If the behavior is halted, we need to stop our service server
template<typename ServiceType>
void ServiceServerBase<ServiceType>::onHalted()
{
  // Set the cancel flag to true to indicate that the behavior should be canceled
  cancel_behavior_.store(true);
  // Notify doWork to continue
  cv_.notify_one();
  // Call the base class's onHalted method for any additional cleanup
  AsyncBehaviorBase::onHalted();
}

template<typename ServiceType>
BT::KeyValueVector ServiceServerBase<ServiceType>::metadata()
{
  return {};
}

template<typename ServiceType>
BT::PortsList ServiceServerBase<ServiceType>::providedPorts()
{
  return {};
}
// This task runs asynchronously once the behavior is ticked
// The behavior will return RUNNING until this thread completes
template<typename ServiceType>
tl::expected<bool, std::string> ServiceServerBase<ServiceType>::doWork()
{
  const auto service_name = this->getInput<std::string>(kInputServiceName);

  if (!service_name.has_value())
  {
    spdlog::warn("Port {} not defined. Creating service with topic {}", kInputServiceName, kDefaultTopicName);
  }
  const auto topic_name = service_name.has_value() ? service_name.value() : kDefaultTopicName;
  // Create a service server for the given ServiceType using the shared node
  service_ = shared_resources_->node->create_service<ServiceType>(
      topic_name,
      std::bind(&ServiceServerBase<ServiceType>::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Mutex to keep the service callback from accessing shared memory
  std::unique_lock<std::mutex> lock(mutex_);

  // Wait until a service callback to completes, or the behavior is halted
  while (!data_available_.load() && !cancel_behavior_.load())
  {
    // Release the mutex so serviceCallback can execute
    cv_.wait(lock);
  }
  // If a service was called, we want to reset this flag so the behavior can be ticked again
  data_available_.store(false);

  if (cancel_behavior_.load())
  {
    return tl::make_unexpected("Behavior canceled");
  }

  // Derived classes should process the data and set outputs as necessary
  return true;
}

template<typename ServiceType>
void ServiceServerBase<ServiceType>::serviceCallback(const typename ServiceType::Request::SharedPtr& request,
                                                    const typename ServiceType::Response::SharedPtr& response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // To be implemented in derived classes
    processRequest(request, response);
    data_available_.store(true);
  }
  // Notify doWork to continue
  cv_.notify_one();
}

template<typename ServiceType>
BT::NodeStatus ServiceServerBase<ServiceType>::onRunning()
{
  // This node returns success after a service call, but should reset so it can be ticked again
  const auto node_status = AsyncBehaviorBase::onRunning();
  // If the behavior has succeeded, reset its internal state before returning
  if (node_status == BT::NodeStatus::SUCCESS) resetStatus();
  return node_status;
}

} // namespace example_behaviors

#endif // SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_IMPL_HPP_
