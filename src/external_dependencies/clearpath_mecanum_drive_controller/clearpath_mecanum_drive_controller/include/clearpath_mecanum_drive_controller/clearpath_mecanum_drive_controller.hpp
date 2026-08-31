// Modified by PickNik Inc., 2026.
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CLEARPATH_MECANUM_DRIVE_CONTROLLER__CLEARPATH_MECANUM_DRIVE_CONTROLLER_HPP_
#define CLEARPATH_MECANUM_DRIVE_CONTROLLER__CLEARPATH_MECANUM_DRIVE_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "clearpath_mecanum_drive_controller/odometry.hpp"
#include "clearpath_mecanum_drive_controller/visibility_control.h"
#include <clearpath_mecanum_drive_controller/clearpath_mecanum_drive_controller_parameters.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "control_msgs/msg/mecanum_drive_controller_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
namespace clearpath_mecanum_drive_controller
{
// name constants for state interfaces
static constexpr size_t NR_STATE_ITFS = 4;

// name constants for command interfaces
static constexpr size_t NR_CMD_ITFS = 4;

// name constants for reference interfaces
static constexpr size_t NR_REF_ITFS = 3;

class MecanumDriveController : public controller_interface::ChainableControllerInterface
{
public:
  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  MecanumDriveController();

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

#ifdef ROS_DISTRO_JAZZY
  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#else
  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;
#endif

  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerReferenceUnstampedMsg = geometry_msgs::msg::Twist;
  using OdomStateMsg = nav_msgs::msg::Odometry;
  using TfStateMsg = tf2_msgs::msg::TFMessage;
  using ControllerStateMsg = control_msgs::msg::MecanumDriveControllerState;

private:
  std::shared_ptr<clearpath_mecanum_drive_controller::ParamListener> param_listener_;
  clearpath_mecanum_drive_controller::Params params_;

  // used for chained controller
  std::vector<std::string> state_joint_names_;

  // Names of the references, ex: high level vel commands from MoveIt, Nav2, etc.
  // used for preceding controller
  std::vector<std::string> reference_names_;

  // Command subscribers and Controller State, odom state, tf state publishers
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerReferenceUnstampedMsg>::SharedPtr ref_unstamped_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceUnstampedMsg>> input_ref_unstamped_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);
  rclcpp::Duration cmd_timeout_ = rclcpp::Duration::from_seconds(1.0);
  rclcpp::Time cmd_timestamp_;
  int64_t last_applied_reference_stamp_nanoseconds_{ 0 };
  int64_t current_reference_stamp_nanoseconds_{ 0 };
  bool has_last_applied_reference_stamp_{ false };
  bool current_reference_is_new_{ true };

  using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_;
  std::unique_ptr<OdomStatePublisher> rt_odom_state_publisher_;

  using TfStatePublisher = realtime_tools::RealtimePublisher<TfStateMsg>;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
  std::unique_ptr<TfStatePublisher> rt_tf_odom_state_publisher_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  Odometry odometry_;

  bool use_stamped_vel_ = true;

  // callback for topic interface
  CLEARATH_MECANUM_DRIVE_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<const ControllerReferenceMsg> msg);
  void reference_unstamped_callback(const std::shared_ptr<const ControllerReferenceUnstampedMsg> msg);

  using WheelVelocities = std::array<double, NR_STATE_ITFS>;
  bool update_reference_interfaces(const rclcpp::Time & time);
  std::optional<WheelVelocities> read_wheel_velocities();
  bool limit_reference(const rclcpp::Duration & period);
  bool rotate_reference_into_body_frame();
  bool write_wheel_commands_from_reference();
  bool stop_wheels();

  double velocity_in_center_frame_linear_x_;   // [m/s]
  double velocity_in_center_frame_linear_y_;   // [m/s]
  double velocity_in_center_frame_angular_z_;  // [rad/s]
  std::array<double, NR_REF_ITFS> last_limited_reference_{};
};

}  // namespace clearpath_mecanum_drive_controller

#endif  // CLEARPATH_MECANUM_DRIVE_CONTROLLER__CLEARPATH_MECANUM_DRIVE_CONTROLLER_HPP_
