// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "robstride_sdk/can_bus.hpp"
#include "robstride_sdk/robstride_motor.hpp"
#include "robstride_sdk/robstride_protocol.hpp"

#include "robstride_interfaces/msg/robstride_state.hpp"
#include "robstride_interfaces/srv/get_data_from_robstride.hpp"
#include "robstride_interfaces/srv/set_data_to_robstride.hpp"
#include "robstride_interfaces/srv/set_zero_robstride.hpp"
#include "robstride_interfaces/srv/reboot_robstride.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "realtime_tools/realtime_publisher.hpp"

namespace robstride_hardware_interface
{

struct JointHandle
{
  std::string joint_name;
  // <gpio> block holding this motor's device configuration.
  std::string gpio_name;
  uint8_t id;
  std::string can_interface;
  robstride_sdk::ActuatorType actuator_type;
  robstride_sdk::RunMode control_mode = robstride_sdk::RunMode::RUN_MODE_MOTION;
  // +1 or -1: joint frame = direction * motor frame (position/velocity/effort).
  double direction = 1.0;
  // Reported position is wrapped into [wrap_center - pi, wrap_center + pi):
  // the middle of the URDF limits, or 0 for a joint without them. A motor
  // that power-cycles may come back a whole turn off, and this folds it back.
  double wrap_center = 0.0;
  // No URDF position limits: targets take the shortest way round.
  bool continuous = false;
  robstride_sdk::RobstrideMotor * motor;  // owned by RobstrideHardware::motors_
};

// ros2_control SystemInterface for RobStride actuators over SocketCAN.
// Owns one CanBus per distinct can_interface, so multi-bus rigs work.
class RobstrideHardware : public hardware_interface::SystemInterface, public rclcpp::Node
{
public:
  RobstrideHardware();
  ~RobstrideHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_;

  std::vector<JointHandle> joints_;
  std::unordered_map<std::string, std::unique_ptr<robstride_sdk::CanBus>> buses_;
  std::vector<std::unique_ptr<robstride_sdk::RobstrideMotor>> motors_;

  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_effort_;
  std::vector<double> hw_cmd_position_;
  std::vector<double> hw_cmd_velocity_;
  std::vector<double> hw_cmd_effort_;

  // <gpio> interfaces: runtime-writable gains and per-motor health.
  std::vector<double> hw_cmd_kp_;
  std::vector<double> hw_cmd_kd_;
  std::vector<double> hw_state_temperature_;
  std::vector<double> hw_state_run_state_;
  std::vector<double> hw_state_fault_bits_;

  // Gain changes parked by SetDataCallback (service thread) and applied by
  // write() (RT thread), which owns hw_cmd_kp_/hw_cmd_kd_.
  std::vector<std::atomic<double>> pending_kp_;
  std::vector<std::atomic<double>> pending_kd_;
  std::vector<std::atomic<bool>> has_pending_kp_;
  std::vector<std::atomic<bool>> has_pending_kd_;

  uint8_t master_id_ = 0xFD;
  double error_timeout_ms_ = 4000.0;
  // Each motor's own CAN_TIMEOUT watchdog; <= 0 disables it.
  double motor_can_timeout_ms_ = 4000.0;
  // While inactive the buses are closed, so missing feedback is expected
  // and read() must not escalate it.
  bool is_active_ = false;
  // true: hold the remaining joints and ignore commands when one is lost.
  // false: return ERROR so the whole component is deactivated.
  bool freeze_on_joint_loss_ = true;
  // true: on_deactivate() leaves torque engaged, so a clean stop holds the
  // pose instead of dropping the arm. Only actually holds while the motor's
  // own CAN_TIMEOUT watchdog is disabled; see on_init().
  bool hold_torque_on_deactivate_ = false;
  // Latched on the first loss, cleared only by a fresh activation.
  bool frozen_ = false;
  // Positions held while frozen, captured at the moment of the loss.
  std::vector<double> freeze_position_;
  // Whole turns (joint frame) dropped when wrapping each joint's position;
  // write() adds them back so targets land in the motor's own frame.
  std::vector<double> wrap_offset_;
  // Joint confirmed enabled since torque was last requested. Leaving the
  // enabled state after that means the motor rebooted or faulted, not that
  // it is still waiting for its first Enable.
  std::vector<bool> seen_enabled_;
  // When write() last sent this joint its re-enable burst. Rate limited to
  // stay inside the bus budget; see kRecoveryResendNs.
  std::vector<uint64_t> last_recovery_ns_;
  // Set by SetTorqueCallback (service thread), consumed by write(), which
  // sends the actual Enable/Disable frames.
  std::atomic<bool> torque_enabled_desired_{true};

  using RobstrideStateMsg = robstride_interfaces::msg::RobstrideState;
  using StatePublisher = realtime_tools::RealtimePublisher<RobstrideStateMsg>;
  rclcpp::Publisher<RobstrideStateMsg>::SharedPtr state_pub_;
  std::unique_ptr<StatePublisher> state_pub_uni_ptr_;

  rclcpp::Service<robstride_interfaces::srv::GetDataFromRobstride>::SharedPtr get_data_srv_;
  rclcpp::Service<robstride_interfaces::srv::SetDataToRobstride>::SharedPtr set_data_srv_;
  rclcpp::Service<robstride_interfaces::srv::SetZeroRobstride>::SharedPtr set_zero_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;
  rclcpp::Service<robstride_interfaces::srv::RebootRobstride>::SharedPtr reboot_srv_;

  // Services are spun on their own thread: spinning inside read() would
  // stall the RT loop for the duration of any slow callback.
  rclcpp::executors::SingleThreadedExecutor::SharedPtr service_executor_;
  std::thread service_executor_thread_;

  void GetDataCallback(
    const std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Response> response);
  void SetDataCallback(
    const std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Response> response);
  void SetZeroCallback(
    const std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Response> response);
  void SetTorqueCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void RebootCallback(
    const std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Response> response);

  // Joint-frame position of joint i from a raw motor reading, wrapped into
  // the joint's window. Records the dropped turns in wrap_offset_.
  double WrapMotorPosition(size_t i, double motor_position);
  // Motor-frame target for a joint-frame position target.
  double MotorTargetPosition(size_t i, double joint_position) const;

  JointHandle * FindJointById(uint8_t id);
  const hardware_interface::ComponentInfo * FindGpio(const std::string & name) const;
};

}  // namespace robstride_hardware_interface

#endif  // ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_
