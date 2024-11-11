#pragma once
#ifndef SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_HPP_
#define SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <tl_expected/expected.hpp>
#include <mutex>
#include <condition_variable>

constexpr auto kInputServiceName = "service_name";
namespace example_behaviors
{
/**
 * @brief Template base class for a ROS2 service server behavior node.
 * @tparam ServiceType The type of the ROS2 service (e.g., std_srvs::srv::Trigger)
 */
template<typename ServiceType>
class ServiceServerBase : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  /**
   * @brief Constructor for the ServiceServerBase behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the MoveIt Pro's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  ServiceServerBase(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the behavior. Marked as final so trhat it is not overridden, and derived classes include the base class's ports
   * @return BT::PortsList containing the behavior's ports.
   */
  BT::PortsList providedPorts() const
  {
      std::cout << "Providing providedPorts ports" << std::endl;
      BT::PortsList ports = getBasePorts();
      BT::PortsList derived_ports = provideAdditionalPorts();
      ports.insert(derived_ports.begin(), derived_ports.end());
      return ports;
  }

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Override onRunning so the node resets itself upon success.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Override onHalted to handle additional thread-breaking logic.
   */
  void onHalted() override;

protected:
  /**
   * @brief Process the service request. This function should be implemented by derived classes.
   * @param request The request received.
   * @param response The response to send back.
   */
  virtual void processRequest(const typename ServiceType::Request::SharedPtr& request,
                              const typename ServiceType::Response::SharedPtr& response) = 0;

  /**
   * @brief Derived classes can add additional ports using this method
   */
  virtual BT::PortsList provideAdditionalPorts();

private:
  /**
   * @brief Atomic bool for cancelling the thread when halted.
   */
  std::atomic<bool> cancel_behavior_;

  /**
   * @brief Atomic bool for ending the thread when data is available.
   */
  std::atomic<bool> data_available_;

  /**
   * @brief Wait for a service client to call, then process the data.
   */
  tl::expected<bool, std::string> doWork() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member. */
  std::shared_future<tl::expected<bool, std::string>> future_;
  /** @brief Thread syenchronization */
  std::mutex mutex_;
  std::condition_variable cv_;

  typename rclcpp::Service<ServiceType>::SharedPtr service_;

  /**
   * @brief The service callback function to process the incoming request.
   * @param request The request received.
   * @param response The response to send back.
   */
  void serviceCallback(const typename ServiceType::Request::SharedPtr& request,
                       const typename ServiceType::Response::SharedPtr& response);

  /**
   * @brief Provide the base ports to be used in all classes
   */
  BT::PortsList getBasePorts() const
  {
      std::cout << "Providing getBasePorts ports" << std::endl;
    return {
      BT::InputPort<std::string>(kInputServiceName, "/service_server", "Name of the service to advertise.")
    };
  }
};

}  // namespace example_behaviors

#include "service_server_base_impl.hpp"

#endif  // SERVICE_SERVER_BASE__SERVICE_SERVER_BASE_HPP_
