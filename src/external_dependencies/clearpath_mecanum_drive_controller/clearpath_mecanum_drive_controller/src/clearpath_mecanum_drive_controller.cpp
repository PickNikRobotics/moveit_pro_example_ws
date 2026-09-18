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

#include "clearpath_mecanum_drive_controller/clearpath_mecanum_drive_controller.hpp"
#include "clearpath_mecanum_drive_controller/reference_validation.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

using ControllerReferenceMsg =
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg;

using ControllerReferenceUnstampedMsg =
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceUnstampedMsg;

// called from RT control loop
void reset_controller_reference_unstamped_msg(
  const std::shared_ptr<ControllerReferenceUnstampedMsg> & msg)
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}


}  // namespace

namespace clearpath_mecanum_drive_controller
{
MecanumDriveController::MecanumDriveController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<clearpath_mecanum_drive_controller::ParamListener>(get_node());
  }
  catch (const std::runtime_error & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joint_names.empty())
  {
    state_joint_names_ = params_.state_joint_names;
  }
  else
  {
    state_joint_names_ = params_.command_joint_names;
  }

  if (params_.command_joint_names.size() != NR_CMD_ITFS ||
      state_joint_names_.size() != NR_STATE_ITFS)
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Exactly %zu command joints and %zu state joints are required, but got %zu and %zu.",
      NR_CMD_ITFS, NR_STATE_ITFS,
      params_.command_joint_names.size(), state_joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_FATAL(get_node()->get_logger(), "The interface name must not be empty.");
    return CallbackReturn::FAILURE;
  }
  if (!std::isfinite(params_.kinematics.wheels_radius) ||
      params_.kinematics.wheels_radius <= 0.0 ||
      !std::isfinite(params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis) ||
      params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis <= 0.0)
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Wheel radius and center-projection sum must be finite and greater than zero.");
    return CallbackReturn::FAILURE;
  }

  const std::array<std::pair<std::string, internal::AxisLimits>, NR_REF_ITFS> axis_limits{ {
    { "linear.x",
      { params_.linear.x.has_velocity_limits, params_.linear.x.min_velocity,
        params_.linear.x.max_velocity, params_.linear.x.has_acceleration_limits,
        params_.linear.x.min_acceleration, params_.linear.x.max_acceleration } },
    { "linear.y",
      { params_.linear.y.has_velocity_limits, params_.linear.y.min_velocity,
        params_.linear.y.max_velocity, params_.linear.y.has_acceleration_limits,
        params_.linear.y.min_acceleration, params_.linear.y.max_acceleration } },
    { "angular.z",
      { params_.angular.z.has_velocity_limits, params_.angular.z.min_velocity,
        params_.angular.z.max_velocity, params_.angular.z.has_acceleration_limits,
        params_.angular.z.min_acceleration, params_.angular.z.max_acceleration } },
  } };
  for (const auto & [axis_name, limits] : axis_limits)
  {
    if (const std::optional<std::string> error = internal::validate_axis_limits(limits, axis_name))
    {
      RCLCPP_FATAL(get_node()->get_logger(), "%s", error->c_str());
      return CallbackReturn::FAILURE;
    }
  }

  odometry_.init(
    get_node()->now(), {params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y, params_.kinematics.base_frame_offset.theta});

  if (params_.body_frame_control != params_.body_frame_yaw_joint.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "The `body_frame_yaw_joint` parameter must be set if and only if the "
      "`body_frame_control` parameter is false.");
    return CallbackReturn::FAILURE;
  }

  // Set wheel params for the odometry computation
  odometry_.setWheelsParams(
    params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis,
    params_.kinematics.wheels_radius);

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  use_stamped_vel_ = params_.use_stamped_vel;
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  cmd_timeout_ = rclcpp::Duration::from_seconds(params_.command_timeout);
  cmd_timestamp_ = get_node()->now();
  if (use_stamped_vel_)
  {
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/cmd_vel", subscribers_qos,
    std::bind(&MecanumDriveController::reference_callback, this, std::placeholders::_1));

    std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    ref_unstamped_subscriber_ = get_node()->create_subscription<ControllerReferenceUnstampedMsg>(
      "~/cmd_vel_unstamped", subscribers_qos,
      std::bind(&MecanumDriveController::reference_unstamped_callback, this, std::placeholders::_1));
    std::shared_ptr<ControllerReferenceUnstampedMsg> msg = std::make_shared<ControllerReferenceUnstampedMsg>();
    reset_controller_reference_unstamped_msg(msg);
    input_ref_unstamped_.writeFromNonRT(msg);
  }

  try
  {
    // Odom state publisher
    odom_s_publisher_ =
      get_node()->create_publisher<OdomStateMsg>("~/odom", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_s_publisher_);
  }
  catch (const std::runtime_error & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  auto & pose_covariance = rt_odom_state_publisher_->msg_.pose.covariance;
  auto & twist_covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    pose_covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    twist_covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }
  rt_odom_state_publisher_->unlock();

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<TfStateMsg>("~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_s_publisher_);
  }
  catch (const std::runtime_error & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  try
  {
    // controller State publisher
    controller_s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::runtime_error & e)
  {
    fprintf(
      stderr,
      "Exception thrown during publisher creation at configure stage "
      "with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumDriveController::reference_callback(const std::shared_ptr<const ControllerReferenceMsg> msg)
{
  // Copy message to allow mutation (setting timestamp, resetting values)
  auto mutable_msg = std::make_shared<ControllerReferenceMsg>(*msg);

  // if no timestamp provided use current time for command timestamp
  if (mutable_msg->header.stamp.sec == 0 && mutable_msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command "
      "timestamp.");
    mutable_msg->header.stamp = get_node()->now();
  }
  const auto age_of_last_command = get_node()->now() - mutable_msg->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.writeFromNonRT(mutable_msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(mutable_msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
    // Replace any older buffered motion. The update loop consumes this stale sample through its
    // timeout path, commands zero wheel velocity, and then resets the buffer to the idle sentinel.
    input_ref_.writeFromNonRT(mutable_msg);
  }
}

void MecanumDriveController::reference_unstamped_callback(const std::shared_ptr<const ControllerReferenceUnstampedMsg> msg)
{
  auto mutable_msg = std::make_shared<ControllerReferenceUnstampedMsg>(*msg);
  input_ref_unstamped_.writeFromNonRT(mutable_msg);
}

controller_interface::InterfaceConfiguration
MecanumDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.command_joint_names.size());
  for (const auto & joint : params_.command_joint_names)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joint_names_.size());

  for (const auto & joint : state_joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }
  if (!params_.body_frame_yaw_joint.empty())
  {
    state_interfaces_config.names.push_back(params_.body_frame_yaw_joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
MecanumDriveController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces_.resize(NR_REF_ITFS, std::numeric_limits<double>::quiet_NaN());
  reference_interfaces.reserve(reference_interfaces_.size());

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(),
    params_.reference_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool MecanumDriveController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn MecanumDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_limited_reference_.fill(0.0);
  has_last_applied_reference_stamp_ = false;
  current_reference_is_new_ = true;
  // Set default value in command
  if(use_stamped_vel_){
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
  }
  else
  {
    reset_controller_reference_unstamped_msg(*(input_ref_unstamped_.readFromRT()));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return stop_wheels() ? controller_interface::CallbackReturn::SUCCESS :
                         controller_interface::CallbackReturn::ERROR;
}

#ifdef ROS_DISTRO_JAZZY
controller_interface::return_type MecanumDriveController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
#else
controller_interface::return_type MecanumDriveController::update_reference_from_subscribers()
#endif
{
  // Move functionality to the `update_and_write_commands` because of the missing arguments in
  // humble - otherwise issues with multiple time-sources might happen when working with simulators

  return controller_interface::return_type::OK;
}

bool MecanumDriveController::update_reference_interfaces(const rclcpp::Time & time)
{
  if (is_in_chained_mode())
  {
    current_reference_is_new_ = true;
    return internal::determine_reference_action(
             { reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2] }) !=
           internal::ReferenceAction::REJECT;
  }

  if (use_stamped_vel_)
  {
    auto current_ref = *(input_ref_.readFromRT());
    const auto age_of_last_command = time - current_ref->header.stamp;
    const internal::ReferenceAction reference_action = internal::determine_reference_action(
      { current_ref->twist.linear.x, current_ref->twist.linear.y,
        current_ref->twist.angular.z });
    if (reference_action == internal::ReferenceAction::WAIT_FOR_REFERENCE)
    {
      current_reference_is_new_ = false;
      return true;
    }
    if (reference_action == internal::ReferenceAction::REJECT)
    {
      return false;
    }

    if (age_of_last_command < rclcpp::Duration::from_seconds(0))
    {
      reset_controller_reference_msg(current_ref, get_node());
      return false;
    }

    current_reference_stamp_nanoseconds_ = rclcpp::Time(current_ref->header.stamp).nanoseconds();
    current_reference_is_new_ =
      !has_last_applied_reference_stamp_ ||
      current_reference_stamp_nanoseconds_ != last_applied_reference_stamp_nanoseconds_;
    if (
      !current_reference_is_new_ &&
      (time < cmd_timestamp_ || cmd_timeout_ < time - cmd_timestamp_))
    {
      reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
      reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
      reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();
      reset_controller_reference_msg(current_ref, get_node());
      return true;
    }

    if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
    {
      reference_interfaces_[0] = current_ref->twist.linear.x;
      reference_interfaces_[1] = current_ref->twist.linear.y;
      reference_interfaces_[2] = current_ref->twist.angular.z;
      if (ref_timeout_ != rclcpp::Duration::from_seconds(0))
      {
        return true;
      }
    }
    else
    {
      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      reference_interfaces_[2] = 0.0;
    }
    reset_controller_reference_msg(current_ref, get_node());
    return true;
  }

  auto current_ref = *(input_ref_unstamped_.readFromRT());
  current_reference_is_new_ = true;
  const internal::ReferenceAction reference_action = internal::determine_reference_action({ current_ref->linear.x, current_ref->linear.y, current_ref->angular.z });
  if (reference_action == internal::ReferenceAction::WAIT_FOR_REFERENCE)
  {
    return true;
  }
  if (reference_action == internal::ReferenceAction::REJECT)
  {
    return false;
  }
  reference_interfaces_[0] = current_ref->linear.x;
  reference_interfaces_[1] = current_ref->linear.y;
  reference_interfaces_[2] = current_ref->angular.z;
  reset_controller_reference_unstamped_msg(current_ref);
  return true;
}

std::optional<MecanumDriveController::WheelVelocities>
MecanumDriveController::read_wheel_velocities()
{
#ifdef ROS_DISTRO_JAZZY
  const std::optional<double> front_left = state_interfaces_[0].get_optional<double>();
  const std::optional<double> back_left = state_interfaces_[1].get_optional<double>();
  const std::optional<double> back_right = state_interfaces_[2].get_optional<double>();
  const std::optional<double> front_right = state_interfaces_[3].get_optional<double>();
  if (!front_left || !back_left || !back_right || !front_right)
  {
    return std::nullopt;
  }
  const WheelVelocities velocities{ *front_left, *back_left, *back_right, *front_right };
#else
  const WheelVelocities velocities{
    state_interfaces_[0].get_value(), state_interfaces_[1].get_value(),
    state_interfaces_[2].get_value(), state_interfaces_[3].get_value()
  };
#endif
  if (!std::all_of(
        velocities.begin(), velocities.end(), [](double value) { return std::isfinite(value); }))
  {
    return std::nullopt;
  }
  return velocities;
}

bool MecanumDriveController::rotate_reference_into_body_frame()
{
  if (params_.body_frame_control || params_.body_frame_yaw_joint.empty() ||
      state_interfaces_.size() < 5)
  {
    return true;
  }
#ifdef ROS_DISTRO_JAZZY
  const std::optional<double> theta_optional = state_interfaces_[4].get_optional<double>();
  if (!theta_optional)
  {
    return false;
  }
  const double theta = *theta_optional;
#else
  const double theta = state_interfaces_[4].get_value();
#endif
  if (!std::isfinite(theta))
  {
    return false;
  }
  const double rotated_x =
    std::cos(-theta) * reference_interfaces_[0] - std::sin(-theta) * reference_interfaces_[1];
  const double rotated_y =
    std::sin(-theta) * reference_interfaces_[0] + std::cos(-theta) * reference_interfaces_[1];
  if (!std::isfinite(rotated_x) || !std::isfinite(rotated_y))
  {
    return false;
  }
  reference_interfaces_[0] = rotated_x;
  reference_interfaces_[1] = rotated_y;
  return true;
}

bool MecanumDriveController::limit_reference(const rclcpp::Duration & period)
{
  const std::array<internal::AxisLimits, NR_REF_ITFS> limits{ {
    { params_.linear.x.has_velocity_limits, params_.linear.x.min_velocity,
      params_.linear.x.max_velocity, params_.linear.x.has_acceleration_limits,
      params_.linear.x.min_acceleration, params_.linear.x.max_acceleration },
    { params_.linear.y.has_velocity_limits, params_.linear.y.min_velocity,
      params_.linear.y.max_velocity, params_.linear.y.has_acceleration_limits,
      params_.linear.y.min_acceleration, params_.linear.y.max_acceleration },
    { params_.angular.z.has_velocity_limits, params_.angular.z.min_velocity,
      params_.angular.z.max_velocity, params_.angular.z.has_acceleration_limits,
      params_.angular.z.min_acceleration, params_.angular.z.max_acceleration },
  } };

  std::array<double, NR_REF_ITFS> limited_reference{};
  for (std::size_t index = 0; index < limited_reference.size(); ++index)
  {
    const std::optional<double> limited = internal::limit_axis_reference(
      reference_interfaces_[index], last_limited_reference_[index], period.seconds(), limits[index]);
    if (!limited)
    {
      return false;
    }
    limited_reference[index] = *limited;
  }
  std::copy(limited_reference.begin(), limited_reference.end(), reference_interfaces_.begin());
  return true;
}

bool MecanumDriveController::stop_wheels()
{
  const std::array<double, NR_CMD_ITFS> stop_commands{};
  const bool stopped = internal::write_commands(stop_commands, [this](const size_t index, const double value) {
#ifdef ROS_DISTRO_JAZZY
    return command_interfaces_[index].set_value(value);
#else
    command_interfaces_[index].set_value(value);
    return true;
#endif
  });
  if (stopped)
  {
    last_limited_reference_.fill(0.0);
  }
  return stopped;
}

bool MecanumDriveController::write_wheel_commands_from_reference()
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, params_.kinematics.base_frame_offset.theta);
  const tf2::Matrix3x3 rotation_from_base_to_center(quaternion);
  const tf2::Vector3 velocity_in_base_frame_w_r_t_center_frame =
    rotation_from_base_to_center *
    tf2::Vector3(reference_interfaces_[0], reference_interfaces_[1], 0.0);
  const tf2::Vector3 linear_trans_from_base_to_center(
    params_.kinematics.base_frame_offset.x, params_.kinematics.base_frame_offset.y, 0.0);

  velocity_in_center_frame_linear_x_ =
    velocity_in_base_frame_w_r_t_center_frame.x() +
    linear_trans_from_base_to_center.y() * reference_interfaces_[2];
  velocity_in_center_frame_linear_y_ =
    velocity_in_base_frame_w_r_t_center_frame.y() -
    linear_trans_from_base_to_center.x() * reference_interfaces_[2];
  velocity_in_center_frame_angular_z_ = reference_interfaces_[2];

  const double w_front_left_vel =
    1.0 / params_.kinematics.wheels_radius *
    (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ -
     params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
       velocity_in_center_frame_angular_z_);
  const double w_back_left_vel =
    1.0 / params_.kinematics.wheels_radius *
    (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ -
     params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
       velocity_in_center_frame_angular_z_);
  const double w_back_right_vel =
    1.0 / params_.kinematics.wheels_radius *
    (velocity_in_center_frame_linear_x_ - velocity_in_center_frame_linear_y_ +
     params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
       velocity_in_center_frame_angular_z_);
  const double w_front_right_vel =
    1.0 / params_.kinematics.wheels_radius *
    (velocity_in_center_frame_linear_x_ + velocity_in_center_frame_linear_y_ +
     params_.kinematics.sum_of_robot_center_projection_on_X_Y_axis *
       velocity_in_center_frame_angular_z_);

  if (
    !std::isfinite(w_front_left_vel) || !std::isfinite(w_back_left_vel) ||
    !std::isfinite(w_back_right_vel) || !std::isfinite(w_front_right_vel))
  {
    return false;
  }

  const std::array<double, NR_CMD_ITFS> wheel_commands{
    w_front_left_vel, w_back_left_vel, w_back_right_vel, w_front_right_vel
  };
  return internal::write_commands(
    wheel_commands, [this](const size_t index, const double value) {
#ifdef ROS_DISTRO_JAZZY
      return command_interfaces_[index].set_value(value);
#else
      command_interfaces_[index].set_value(value);
      return true;
#endif
    });
}

controller_interface::return_type MecanumDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto clear_reference_interfaces = [this]() {
    std::fill(
      reference_interfaces_.begin(), reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
  };
  const auto consume_reference = [this, &clear_reference_interfaces]() {
    clear_reference_interfaces();
    if (is_in_chained_mode())
    {
      return;
    }
    if (use_stamped_vel_)
    {
      reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
    }
    else
    {
      reset_controller_reference_unstamped_msg(*(input_ref_unstamped_.readFromRT()));
    }
  };
  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0)
  {
    std::ignore = stop_wheels();
    consume_reference();
    return controller_interface::return_type::ERROR;
  }

  if (!update_reference_interfaces(time))
  {
    std::ignore = stop_wheels();
    consume_reference();
    return controller_interface::return_type::ERROR;
  }

  // FORWARD KINEMATICS (odometry).
  const std::optional<WheelVelocities> wheel_velocities = read_wheel_velocities();
  if (!wheel_velocities)
  {
    std::ignore = stop_wheels();
    consume_reference();
    return controller_interface::return_type::ERROR;
  }
  const auto & [wheel_front_left_vel, wheel_back_left_vel, wheel_back_right_vel,
    wheel_front_right_vel] = *wheel_velocities;

  // Estimate twist (using joint information) and integrate.
  if (!odometry_.update(
    wheel_front_left_vel, wheel_back_left_vel, wheel_back_right_vel, wheel_front_right_vel,
    period_seconds))
  {
    std::ignore = stop_wheels();
    consume_reference();
    return controller_interface::return_type::ERROR;
  }

  const internal::ReferenceProcessingResult processing_result = internal::process_reference(
    { reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2] },
    time, cmd_timeout_, cmd_timestamp_,
    [this, &period]() { return rotate_reference_into_body_frame() && limit_reference(period); },
    [this]() { return write_wheel_commands_from_reference(); },
    [this]() {
      std::copy(
        reference_interfaces_.begin(), reference_interfaces_.end(),
        last_limited_reference_.begin());
      if (use_stamped_vel_ && current_reference_is_new_)
      {
        last_applied_reference_stamp_nanoseconds_ = current_reference_stamp_nanoseconds_;
        has_last_applied_reference_stamp_ = true;
      }
    },
    [this]() { return stop_wheels(); }, current_reference_is_new_);
  if (processing_result == internal::ReferenceProcessingResult::ERROR)
  {
    consume_reference();
    return controller_interface::return_type::ERROR;
  }

  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getRz());

  // Populate odom message and publish
  if (rt_odom_state_publisher_->trylock())
  {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.getX();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.getY();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.getVx();
    rt_odom_state_publisher_->msg_.twist.twist.linear.y = odometry_.getVy();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.getWz();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x = odometry_.getX();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y = odometry_.getY();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = get_node()->now();
    controller_state_publisher_->msg_.front_left_wheel_velocity = wheel_front_left_vel;
    controller_state_publisher_->msg_.back_left_wheel_velocity = wheel_back_left_vel;
    controller_state_publisher_->msg_.back_right_wheel_velocity = wheel_back_right_vel;
    controller_state_publisher_->msg_.front_right_wheel_velocity = wheel_front_right_vel;
    controller_state_publisher_->msg_.reference_velocity.linear.x = reference_interfaces_[0];
    controller_state_publisher_->msg_.reference_velocity.linear.y = reference_interfaces_[1];
    controller_state_publisher_->msg_.reference_velocity.angular.z = reference_interfaces_[2];
    controller_state_publisher_->unlockAndPublish();
  }

  clear_reference_interfaces();

  return controller_interface::return_type::OK;
}

}  // namespace clearpath_mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  clearpath_mecanum_drive_controller::MecanumDriveController,
  controller_interface::ChainableControllerInterface)
