// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_studio_kinova_pstop_manager/protective_stop_manager_node.hpp>

namespace moveit_studio::kinova_pstop_manager
{
constexpr auto kNodeName = "protective_stop_manager";
static const auto kLogger = rclcpp::get_logger(kNodeName);
constexpr auto kSwitchControllerService = "/controller_manager/switch_controller";
constexpr auto kRecoveryServiceName = "recover_from_protective_stop";
constexpr auto kFaultStatusTopic = "~/robot_fault_status";
constexpr auto kServiceCallTimeout = std::chrono::duration<double>(1.0);

constexpr auto kFaultStatusPeriod = std::chrono::duration<double>(0.02);
constexpr auto kWatchdogPeriodSec = 0.1;

constexpr auto kFaultTopic = "/fault_controller/internal_fault";
constexpr auto kResetFault = "/fault_controller/reset_fault";

constexpr auto kParameterControllersDefaultActive = "controllers_default_active";
constexpr auto kParameterControllersDefaultNotActive = "controllers_default_not_active";

ProtectiveStopManager::ProtectiveStopManager(const rclcpp::NodeOptions& options)
  : rclcpp::Node("protective_stop_manager", options)
  , reentrant_callback_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  , recovery_service_(create_service<std_srvs::srv::Trigger>(
        kRecoveryServiceName,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) { recoverFromProtectiveStop(request, response); },
        rmw_qos_profile_services_default, reentrant_callback_group_))
  , switch_controller_client_(create_client<SwitchController>(
        kSwitchControllerService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , fault_status_publisher_(create_publisher<moveit_studio_agent_msgs::msg::FaultStatus>(kFaultStatusTopic, 1))
  , fault_status_timer_(create_wall_timer(kFaultStatusPeriod, [this] { this->publishFaultStatus(); }))
  , in_fault_(false)
  , last_fault_status_update_(0, 0, this->now().get_clock_type())
  , fault_ctrl_sub_(create_subscription<example_interfaces::msg::Bool>(
        kFaultTopic, rclcpp::QoS(1),
        [&](const example_interfaces::msg::Bool::ConstSharedPtr msg) {
          in_fault_ = msg->data;
          last_fault_status_update_ = this->now();
        }))
  , fault_reset_client_(create_client<example_interfaces::srv::Trigger>(kResetFault, rmw_qos_profile_services_default,
                                                                        reentrant_callback_group_))
{
  declare_parameter<std::vector<std::string>>(kParameterControllersDefaultActive, std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>(kParameterControllersDefaultNotActive, std::vector<std::string>{});

  // Retrieve list of controllers which should be active by default
  active_controller_names = get_parameter(kParameterControllersDefaultActive).as_string_array();

  // Set of all controllers = default-active controllers + default-inactive controllers
  all_controller_names = get_parameter(kParameterControllersDefaultNotActive).as_string_array();
  all_controller_names.insert(all_controller_names.end(), active_controller_names.cbegin(),
                              active_controller_names.cend());

  // remove fault controller because it is needed for the protective stop manager to work
  // fault controller exposes reset_fault service and internal_fault topic
  auto iter = std::find_if(all_controller_names.begin(), all_controller_names.end(),
                           [&](const std::string& name) { return name == "fault_controller"; });
  if (iter != all_controller_names.end())
  {
    all_controller_names.erase(iter);
  }
}

void ProtectiveStopManager::recoverFromProtectiveStop(const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
                                                      std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // Deactivate currently-active controllers.
  // This is needed to prevent any controllers from commanding the robot hardware to a stale position when the Kinova
  // enabled again
  RCLCPP_INFO_STREAM(kLogger, "Deactivating all controllers...");
  if (indicateUnavailableService(switch_controller_client_, response))
  {
    return;
  }
  auto deactivate_all_controllers_request = std::make_shared<SwitchController::Request>();
#ifdef ROS_DISTRO_JAZZY
  deactivate_all_controllers_request->deactivate_controllers = all_controller_names;
#else
  deactivate_all_controllers_request->stop_controllers = all_controller_names;
#endif
  // Use BEST_EFFORT strictness because some controllers may already be inactive, and attempting to stop them while
  // using STRICT strictness would introduce an unnecessary error.
  deactivate_all_controllers_request->strictness = SwitchController::Request::BEST_EFFORT;
  auto deactivate_all_controllers_future =
      switch_controller_client_->async_send_request(deactivate_all_controllers_request);
  if (deactivate_all_controllers_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") +
                     switch_controller_client_->get_service_name() +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  if (!deactivate_all_controllers_future.get()->ok)
  {
    const auto msg = "Failed to deactivate robot controllers.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // Reactivate controllers set in the site config as active by default.
  // This resets the state of the robot controllers to match the state it was in at startup.
  RCLCPP_INFO_STREAM(kLogger, "Reactivating default controllers...");
  auto activate_default_controllers_request = std::make_shared<SwitchController::Request>();
#ifdef ROS_DISTRO_JAZZY
  activate_default_controllers_request->activate_controllers = active_controller_names;
#else
  activate_default_controllers_request->start_controllers = active_controller_names;
#endif
  // BEST_EFFORT is good enough to put the system back into an operational state without throwing unnecessary errors.
  activate_default_controllers_request->strictness = SwitchController::Request::BEST_EFFORT;
  auto activate_default_controllers_future =
      switch_controller_client_->async_send_request(activate_default_controllers_request);
  if (activate_default_controllers_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") +
                     switch_controller_client_->get_service_name() +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  if (!activate_default_controllers_future.get()->ok)
  {
    const auto msg = "Failed to activate default robot controllers.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // reset fault
  RCLCPP_INFO_STREAM(kLogger, "Resetting fault on the robot...");
  if (indicateUnavailableService(fault_reset_client_, response))
  {
    return;
  }

  auto reset_fault_request = std::make_shared<example_interfaces::srv::Trigger::Request>();
  auto reset_fault_future = fault_reset_client_->async_send_request(reset_fault_request);
  if (reset_fault_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") + fault_reset_client_->get_service_name() +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  if (!reset_fault_future.get()->success)
  {
    const auto msg = "Failed to reset robot fault.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
}

bool ProtectiveStopManager::indicateUnavailableService(rclcpp::ClientBase::SharedPtr client,
                                                       std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (client->service_is_ready())
  {
    return false;
  }

  const auto msg = std::string("`") + client->get_service_name() + "` service is not ready.";
  RCLCPP_ERROR_STREAM(kLogger, msg);
  if (response)
  {
    response->success = false;
    response->message = msg;
  }
  return true;
}

void ProtectiveStopManager::publishFaultStatus()
{
  auto msg = moveit_studio_agent_msgs::msg::FaultStatus();
  msg.status = in_fault_ ? moveit_studio_agent_msgs::msg::FaultStatus::RECOVERABLE_FAULT :
                           moveit_studio_agent_msgs::msg::FaultStatus::NORMAL;

  msg.status = ((this->now() - last_fault_status_update_).seconds() < kWatchdogPeriodSec) ?
                   msg.status :
                   moveit_studio_agent_msgs::msg::FaultStatus::NONRECOVERABLE_FAULT;

  fault_status_publisher_->publish(msg);
}

bool ProtectiveStopManager::isRobotInFault()
{
  return in_fault_;
}
}  // namespace moveit_studio::kinova_pstop_manager

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<moveit_studio::kinova_pstop_manager::ProtectiveStopManager>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  rclcpp::shutdown();
}
