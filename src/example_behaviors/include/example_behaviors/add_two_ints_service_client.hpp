#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using moveit_studio::behaviors::BehaviorContext;
using moveit_studio::behaviors::ServiceClientBehaviorBase;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

namespace example_behaviors
{
class AddTwoIntsServiceClient final : public ServiceClientBehaviorBase<AddTwoInts>
{
public:
  AddTwoIntsServiceClient(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<BehaviorContext>& shared_resources);

  /** @brief Implementation of the required providedPorts() function for the hello_world Behavior. */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

private:
  /** @brief User-provided function to get the name of the service when initializing the service client. */
  tl::expected<std::string, std::string> getServiceName() override;

  /**
   * @brief User-provided function to create the service request.
   * @return Returns a service request message. If not successful, returns an error message. Note that the criteria for
   * success or failure is defined by the user's implementation of this function.
   */
  tl::expected<AddTwoInts::Request, std::string> createRequest() override;

  /** @brief Optional user-provided function to process the service response after the service has finished. */
  tl::expected<bool, std::string> processResponse(const AddTwoInts::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
