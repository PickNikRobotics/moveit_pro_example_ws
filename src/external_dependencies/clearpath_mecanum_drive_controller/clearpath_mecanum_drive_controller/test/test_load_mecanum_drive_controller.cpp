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

#include <gmock/gmock.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include "clearpath_mecanum_drive_controller/clearpath_mecanum_drive_controller.hpp"
#include "clearpath_mecanum_drive_controller/clearpath_mecanum_drive_controller_parameters.hpp"
#include "clearpath_mecanum_drive_controller/odometry.hpp"
#include "clearpath_mecanum_drive_controller/parameter_validators.hpp"
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace allocation_test
{
thread_local std::size_t monitoring_depth = 0;
thread_local std::size_t allocation_count = 0;
std::size_t new_handler_call_count = 0;

void record_new_handler_call()
{
  ++new_handler_call_count;
  std::set_new_handler(nullptr);
}

void record_allocation()
{
  if (monitoring_depth > 0)
  {
    ++allocation_count;
  }
}

void* allocate(const std::size_t size)
{
  while (true)
  {
    if (void* const pointer = std::malloc(size == 0 ? 1 : size))
    {
      return pointer;
    }
    const std::new_handler handler = std::get_new_handler();
    if (handler == nullptr)
    {
      throw std::bad_alloc();
    }
    handler();
  }
}

void* allocate_aligned(const std::size_t size, const std::align_val_t alignment)
{
  while (true)
  {
    void* pointer = nullptr;
    if (posix_memalign(&pointer, static_cast<std::size_t>(alignment), size == 0 ? 1 : size) == 0)
    {
      return pointer;
    }
    const std::new_handler handler = std::get_new_handler();
    if (handler == nullptr)
    {
      throw std::bad_alloc();
    }
    handler();
  }
}

class ScopedMonitor
{
public:
  ScopedMonitor() : baseline_(allocation_count)
  {
    ++monitoring_depth;
  }

  ScopedMonitor(const ScopedMonitor&) = delete;
  ScopedMonitor& operator=(const ScopedMonitor&) = delete;
  ScopedMonitor(ScopedMonitor&&) = delete;
  ScopedMonitor& operator=(ScopedMonitor&&) = delete;

  ~ScopedMonitor()
  {
    if (active_)
    {
      --monitoring_depth;
    }
  }

  std::size_t stop()
  {
    if (active_)
    {
      --monitoring_depth;
      active_ = false;
    }
    return allocation_count - baseline_;
  }

private:
  const std::size_t baseline_;
  bool active_{ true };
};
}  // namespace allocation_test

void* operator new(const std::size_t size)
{
  allocation_test::record_allocation();
  return allocation_test::allocate(size);
}

void * operator new[](const std::size_t size)
{
  return ::operator new(size);
}

void * operator new(const std::size_t size, const std::nothrow_t &) noexcept
{
  try
  {
    return ::operator new(size);
  }
  catch (...)
  {
    return nullptr;
  }
}

void * operator new[](const std::size_t size, const std::nothrow_t &) noexcept
{
  return ::operator new(size, std::nothrow);
}

void* operator new(const std::size_t size, const std::align_val_t alignment)
{
  allocation_test::record_allocation();
  return allocation_test::allocate_aligned(size, alignment);
}

void* operator new[](const std::size_t size, const std::align_val_t alignment)
{
  return ::operator new(size, alignment);
}

void* operator new(const std::size_t size, const std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  try
  {
    return ::operator new(size, alignment);
  }
  catch (...)
  {
    return nullptr;
  }
}

void* operator new[](const std::size_t size, const std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  return ::operator new(size, alignment, std::nothrow);
}

void operator delete(void * const pointer) noexcept
{
  std::free(pointer);
}

void operator delete[](void * const pointer) noexcept
{
  ::operator delete(pointer);
}

void operator delete(void * const pointer, const std::size_t) noexcept
{
  ::operator delete(pointer);
}

void operator delete[](void * const pointer, const std::size_t) noexcept
{
  ::operator delete(pointer);
}

void operator delete(void * const pointer, const std::align_val_t) noexcept
{
  std::free(pointer);
}

void operator delete[](void * const pointer, const std::align_val_t alignment) noexcept
{
  ::operator delete(pointer, alignment);
}

void operator delete(void * const pointer, const std::size_t, const std::align_val_t alignment) noexcept
{
  ::operator delete(pointer, alignment);
}

void operator delete[](void * const pointer, const std::size_t, const std::align_val_t alignment) noexcept
{
  ::operator delete(pointer, alignment);
}

void operator delete(void* const pointer, const std::nothrow_t&) noexcept
{
  ::operator delete(pointer);
}

void operator delete[](void* const pointer, const std::nothrow_t&) noexcept
{
  ::operator delete(pointer);
}

void operator delete(void* const pointer, const std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  ::operator delete(pointer, alignment);
}

void operator delete[](void* const pointer, const std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  ::operator delete(pointer, alignment);
}

#define EXPECT_NO_MALLOC(statement)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    allocation_test::ScopedMonitor allocation_monitor;                                                                \
    statement;                                                                                                         \
    EXPECT_EQ(allocation_monitor.stop(), 0U) << "Real-time update path allocated heap memory";                         \
  } while (false)

TEST(AllocationMonitor, NestedScopesPreserveOuterCount)
{
  // GIVEN an active allocation monitor containing another monitor.
  allocation_test::ScopedMonitor outer_monitor;
  void* const first_allocation = ::operator new(1);
  allocation_test::ScopedMonitor inner_monitor;

  // WHEN allocations occur inside and after the nested scope.
  void* const inner_allocation = ::operator new(1);
  const std::size_t inner_count = inner_monitor.stop();
  void* const final_allocation = ::operator new(1);
  const std::size_t outer_count = outer_monitor.stop();
  ::operator delete(first_allocation);
  ::operator delete(inner_allocation);
  ::operator delete(final_allocation);

  // THEN the inner count is scoped and the outer count includes all allocations.
  EXPECT_EQ(inner_count, 1U);
  EXPECT_EQ(outer_count, 3U);
}

TEST(AllocationMonitor, AlignedNothrowAllocationIsCounted)
{
  // GIVEN an allocation monitor on the current thread.
  allocation_test::ScopedMonitor monitor;

  // WHEN aligned storage is allocated through the nothrow replacement overload.
  void* const allocation = ::operator new(64, std::align_val_t{ 64 }, std::nothrow);
  const std::size_t allocation_count = monitor.stop();

  // THEN the allocation is valid and counted exactly once.
  ASSERT_NE(allocation, nullptr);
  EXPECT_EQ(allocation_count, 1U);
  ::operator delete(allocation, std::align_val_t{ 64 }, std::nothrow);
}

TEST(AllocationMonitor, AllocationFailureInvokesNewHandler)
{
  // GIVEN an active new handler that records one invocation and then clears itself.
  allocation_test::new_handler_call_count = 0;
  const std::new_handler previous_handler = std::set_new_handler(allocation_test::record_new_handler_call);

  // WHEN an impossible allocation is requested.
  volatile std::size_t impossible_size = std::numeric_limits<std::size_t>::max();
  using NewFunction = void* (*)(std::size_t);
  NewFunction volatile allocate = static_cast<NewFunction>(&::operator new);
  void* allocation = nullptr;
  bool threw_bad_alloc = false;
  try
  {
    allocation = allocate(impossible_size);
  }
  catch (const std::bad_alloc&)
  {
    threw_bad_alloc = true;
  }
  std::set_new_handler(previous_handler);
  ::operator delete(allocation);

  // THEN the replacement follows the standard handler contract before failing.
  EXPECT_TRUE(threw_bad_alloc);
  EXPECT_EQ(allocation_test::new_handler_call_count, 1U);
}

namespace
{

std::vector<rclcpp::Parameter> valid_controller_parameters()
{
  return {
    rclcpp::Parameter(
      "command_joint_names",
      std::vector<std::string>{ "wheel_0", "wheel_1", "wheel_2", "wheel_3" }),
    rclcpp::Parameter(
      "state_joint_names",
      std::vector<std::string>{ "wheel_0", "wheel_1", "wheel_2", "wheel_3" }),
    rclcpp::Parameter("interface_name", "velocity"),
    rclcpp::Parameter("kinematics.wheels_radius", 0.1),
    rclcpp::Parameter("kinematics.sum_of_robot_center_projection_on_X_Y_axis", 0.5),
    rclcpp::Parameter("body_frame_control", true),
    rclcpp::Parameter("enable_odom_tf", false),
  };
}

void apply_parameter_replacements(
  std::vector<rclcpp::Parameter> & parameters,
  const std::vector<rclcpp::Parameter> & replacements)
{
  for (const rclcpp::Parameter & replacement : replacements)
  {
    const auto parameter = std::find_if(
      parameters.begin(), parameters.end(), [&replacement](const rclcpp::Parameter & candidate) {
        return candidate.get_name() == replacement.get_name();
      });
    if (parameter == parameters.end())
    {
      parameters.push_back(replacement);
    }
    else
    {
      *parameter = replacement;
    }
  }
}

void expect_configuration_result(
  const std::vector<rclcpp::Parameter> & replacements,
  const controller_interface::CallbackReturn expected_result)
{
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  apply_parameter_replacements(parameters, replacements);

  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "invalid_configuration_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  const controller_interface::return_type init_result = controller.init(controller_params);
  EXPECT_EQ(init_result, controller_interface::return_type::OK);
  if (init_result == controller_interface::return_type::OK)
  {
    EXPECT_EQ(
      controller.on_configure(rclcpp_lifecycle::State{}),
      expected_result);
  }
  rclcpp::shutdown();
}

}  // namespace

TEST(TestLoadMecanumDriveController, when_loading_controller_expect_no_exception)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

#ifdef ROS_DISTRO_JAZZY
  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf,
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
      rclcpp::get_logger("test_controller_manager")),
    executor, "test_controller_manager");
#else
  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf),
    executor, "test_controller_manager");
#endif

  ASSERT_NE(
    cm.load_controller(
      "test_mecanum_drive_controller",
      "clearpath_mecanum_drive_controller/MecanumDriveController"),
    nullptr);

  rclcpp::shutdown();
}

#ifdef ROS_DISTRO_JAZZY
namespace
{

std::vector<hardware_interface::CommandInterface::SharedPtr> assign_controller_interfaces(
  clearpath_mecanum_drive_controller::MecanumDriveController & controller,
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> * state_interface_handles,
  const std::optional<double> yaw = std::nullopt, const double wheel_velocity = 0.0,
  const std::optional<std::array<double, clearpath_mecanum_drive_controller::NR_STATE_ITFS>> &
    wheel_velocities = std::nullopt)
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interface_handles;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_CMD_ITFS; ++index)
  {
    hardware_interface::CommandInterface::SharedPtr command_interface =
      std::make_shared<hardware_interface::CommandInterface>(
        "wheel_" + std::to_string(index), "velocity");
    command_interfaces.emplace_back(command_interface, []() {});
    command_interface_handles.push_back(std::move(command_interface));
  }

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_STATE_ITFS; ++index)
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = "velocity";
    interface_info.data_type = "double";
    std::shared_ptr<hardware_interface::StateInterface> state_interface =
      std::make_shared<hardware_interface::StateInterface>(hardware_interface::InterfaceDescription{
        "wheel_" + std::to_string(index), interface_info });
    EXPECT_TRUE(state_interface->set_value(wheel_velocities ? (*wheel_velocities)[index] : wheel_velocity));
    state_interfaces.emplace_back(state_interface, []() {});
    state_interface_handles->push_back(std::move(state_interface));
  }
  if (yaw)
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = "position";
    interface_info.data_type = "double";
    std::shared_ptr<hardware_interface::StateInterface> yaw_interface =
      std::make_shared<hardware_interface::StateInterface>(
        hardware_interface::InterfaceDescription{ "base_yaw", interface_info });
    EXPECT_TRUE(yaw_interface->set_value(*yaw));
    state_interfaces.emplace_back(std::move(yaw_interface), []() {});
  }

  controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  return command_interface_handles;
}

void publish_stamped_reference(
  clearpath_mecanum_drive_controller::MecanumDriveController & controller,
  const clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg &
    reference)
{
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> controller_node = controller.get_node();
  const std::shared_ptr<rclcpp::Node> publisher_node =
    std::make_shared<rclcpp::Node>("stamped_reference_publisher");
  auto qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  qos.best_effort();
  const auto publisher = publisher_node->create_publisher<
    clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg>(
    "/" + std::string(controller_node->get_name()) + "/cmd_vel", qos);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_node->get_node_base_interface());

  // Publish repeatedly while spinning so discovery and delivery complete without a timed sleep.
  for (std::size_t attempt = 0; attempt < 10; ++attempt)
  {
    publisher->publish(reference);
    executor.spin_some();
  }

  executor.remove_node(controller_node->get_node_base_interface());
  EXPECT_GT(publisher->get_subscription_count(), 0U);
}

}  // namespace

TEST(MecanumDriveControllerDeactivation, RejectedStopWriteReturnsError)
{
  // GIVEN a controller whose second wheel command interface cannot be locked for writing.
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  std::array<double, clearpath_mecanum_drive_controller::NR_CMD_ITFS> command_values{};
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interface_handles;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interface_handles.reserve(command_values.size());
  command_interfaces.reserve(command_values.size());
  for (std::size_t index = 0; index < command_values.size(); ++index)
  {
    hardware_interface::CommandInterface::SharedPtr command_interface =
      std::make_shared<hardware_interface::CommandInterface>(
        "wheel_" + std::to_string(index), "velocity", &command_values[index]);
    command_interfaces.emplace_back(command_interface, []() {});
    command_interface_handles.push_back(std::move(command_interface));
  }
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  std::unique_lock<std::shared_mutex> rejected_write_lock(
    command_interface_handles[1]->get_mutex());

  // WHEN the production lifecycle callback attempts to stop every wheel.
  const controller_interface::CallbackReturn result =
    controller.on_deactivate(rclcpp_lifecycle::State{});

  // THEN it reports the rejected safety write to the controller manager.
  EXPECT_EQ(result, controller_interface::CallbackReturn::ERROR);
}

TEST(MecanumDriveControllerUpdate, RejectedCommandWriteIsConsumedWithoutAllocation)
{
  // GIVEN a configured chained controller whose second wheel rejects command writes.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "rejected_command_write_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(
    { rclcpp::Parameter("command_joint_names", std::vector<std::string>{
                                                   "wheel_0", "wheel_1", "wheel_2", "wheel_3" }),
      rclcpp::Parameter("state_joint_names", std::vector<std::string>{
                                                 "wheel_0", "wheel_1", "wheel_2", "wheel_3" }),
      rclcpp::Parameter("interface_name", "velocity"),
      rclcpp::Parameter("kinematics.wheels_radius", 0.1),
      rclcpp::Parameter("kinematics.sum_of_robot_center_projection_on_X_Y_axis", 0.5),
      rclcpp::Parameter("body_frame_control", true), rclcpp::Parameter("enable_odom_tf", false) });
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interface_handles;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interface_handles.reserve(clearpath_mecanum_drive_controller::NR_CMD_ITFS);
  command_interfaces.reserve(clearpath_mecanum_drive_controller::NR_CMD_ITFS);
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_CMD_ITFS; ++index)
  {
    hardware_interface::CommandInterface::SharedPtr command_interface =
      std::make_shared<hardware_interface::CommandInterface>(
        "wheel_" + std::to_string(index), "velocity");
    ASSERT_TRUE(command_interface->set_value(42.0));
    command_interfaces.emplace_back(command_interface, []() {});
    command_interface_handles.push_back(std::move(command_interface));
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interface_handles.reserve(clearpath_mecanum_drive_controller::NR_STATE_ITFS);
  state_interfaces.reserve(clearpath_mecanum_drive_controller::NR_STATE_ITFS);
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_STATE_ITFS; ++index)
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = "velocity";
    interface_info.data_type = "double";
    std::shared_ptr<hardware_interface::StateInterface> state_interface =
      std::make_shared<hardware_interface::StateInterface>(hardware_interface::InterfaceDescription{
        "wheel_" + std::to_string(index), interface_info });
    ASSERT_TRUE(state_interface->set_value(0.0));
    state_interfaces.emplace_back(state_interface, []() {});
    state_interface_handles.push_back(std::move(state_interface));
  }
  controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 3U);
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));
  std::unique_lock<std::shared_mutex> rejected_write_lock(
    command_interface_handles[1]->get_mutex());

  // WHEN the production update path attempts to write the requested wheel motion.
  controller_interface::return_type result{};
  EXPECT_NO_MALLOC(
    result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));
  rejected_write_lock.unlock();

  // THEN the error propagates and the stop path zeroes every writable wheel.
  EXPECT_EQ(result, controller_interface::return_type::ERROR);
  EXPECT_EQ(command_interface_handles[0]->get_optional<double>(), 0.0);
  EXPECT_EQ(command_interface_handles[1]->get_optional<double>(), 42.0);
  EXPECT_EQ(command_interface_handles[2]->get_optional<double>(), 0.0);
  EXPECT_EQ(command_interface_handles[3]->get_optional<double>(), 0.0);

  // WHEN an idle cycle runs after the timeout without receiving a new reference.
  controller_interface::return_type idle_result{};
  EXPECT_NO_MALLOC(
    idle_result = controller.update_and_write_commands(
      rclcpp::Time{ 2, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the rejected reference is not retried or allowed to refresh the command timestamp.
  EXPECT_EQ(idle_result, controller_interface::return_type::OK);
  EXPECT_EQ(command_interface_handles[0]->get_optional<double>(), 0.0);
  EXPECT_EQ(command_interface_handles[1]->get_optional<double>(), 0.0);
  EXPECT_EQ(command_interface_handles[2]->get_optional<double>(), 0.0);
  EXPECT_EQ(command_interface_handles[3]->get_optional<double>(), 0.0);
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, SuccessfulCommandProcessingDoesNotAllocate)
{
  // GIVEN a chained controller with a 1.3 m/s X velocity limit.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "velocity_limit_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("linear.x.has_velocity_limits", true);
  parameters.emplace_back("linear.x.min_velocity", -1.3);
  parameters.emplace_back("linear.x.max_velocity", 1.3);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interface_handles;
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_CMD_ITFS; ++index)
  {
    hardware_interface::CommandInterface::SharedPtr command_interface =
      std::make_shared<hardware_interface::CommandInterface>(
        "wheel_" + std::to_string(index), "velocity");
    command_interfaces.emplace_back(command_interface, []() {});
    command_interface_handles.push_back(std::move(command_interface));
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  for (std::size_t index = 0; index < clearpath_mecanum_drive_controller::NR_STATE_ITFS; ++index)
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = "velocity";
    interface_info.data_type = "double";
    std::shared_ptr<hardware_interface::StateInterface> state_interface =
      std::make_shared<hardware_interface::StateInterface>(hardware_interface::InterfaceDescription{
        "wheel_" + std::to_string(index), interface_info });
    ASSERT_TRUE(state_interface->set_value(0.0));
    state_interfaces.emplace_back(state_interface, []() {});
    state_interface_handles.push_back(std::move(state_interface));
  }
  controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 3U);
  ASSERT_TRUE(reference_interfaces[0]->set_value(10.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));

  // WHEN the update path receives a finite but excessive X reference.
  controller_interface::return_type result{};
  EXPECT_NO_MALLOC(
    result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the 0.1 m wheel radius converts the clamped 1.3 m/s reference to 13 rad/s.
  EXPECT_EQ(result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interface_handles)
  {
    ASSERT_TRUE(command_interface->get_optional<double>().has_value());
    EXPECT_NEAR(*command_interface->get_optional<double>(), 13.0, 1e-12);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, TimeoutStopDoesNotAllocate)
{
  // GIVEN a configured chained controller that has successfully applied motion.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "timeout_allocation_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("command_timeout", 0.1);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 3U);
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));
  ASSERT_EQ(
    controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // WHEN an empty reference reaches the update path after the timeout expires.
  controller_interface::return_type result{};
  EXPECT_NO_MALLOC(
    result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 200000000, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the controller stops every wheel without allocating in the real-time thread.
  EXPECT_EQ(result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, BackwardTimeJumpStopsWithoutRefreshingTimestampOrAllocating)
{
  // GIVEN a configured chained controller that accepted motion at t=1.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "backward_time_jump_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("command_timeout", 0.1);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));
  ASSERT_EQ(
    controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // WHEN simulation time jumps backward while the consumed reference is idle.
  controller_interface::return_type backward_jump_result{};
  EXPECT_NO_MALLOC(
    backward_jump_result = controller.update_and_write_commands(
      rclcpp::Time{ 0, 900000000, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the invalid timing state stops every wheel without becoming a controller error.
  EXPECT_EQ(backward_jump_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
    ASSERT_TRUE(command_interface->set_value(42.0));
  }

  // WHEN time advances to a point still before the original accepted-command timeout.
  controller_interface::return_type before_timeout_result{};
  EXPECT_NO_MALLOC(
    before_timeout_result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 50000000, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the backward jump did not refresh the accepted-command timestamp or replay the reference.
  EXPECT_EQ(before_timeout_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 42.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, FutureStampedReferenceStopsAndIsConsumedWithoutAllocation)
{
  // GIVEN a configured subscriber-mode controller and a velocity command stamped in the future.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "future_stamped_reference_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("reference_timeout", 1.0);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  const std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), clearpath_mecanum_drive_controller::NR_REF_ITFS);

  const rclcpp::Time update_time = controller.get_node()->now();
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg reference;
  reference.header.stamp = update_time + rclcpp::Duration::from_seconds(1.0);
  reference.twist.linear.x = 1.0;
  publish_stamped_reference(controller, reference);

  // WHEN the production update sees the future-dated reference.
  controller_interface::return_type future_reference_result{};
  EXPECT_NO_MALLOC(
    future_reference_result = controller.update_and_write_commands(
      update_time, rclcpp::Duration::from_seconds(0.01)));

  // THEN it fails closed and consumes the invalid command.
  EXPECT_EQ(future_reference_result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
    ASSERT_TRUE(command_interface->set_value(42.0));
  }
  controller_interface::return_type idle_result{};
  EXPECT_NO_MALLOC(
    idle_result = controller.update_and_write_commands(
      update_time + rclcpp::Duration::from_seconds(0.01),
      rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(idle_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 42.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, StaleCallbackReplacesBufferedMotionAndStopsWithoutReplay)
{
  // GIVEN a valid motion command buffered before a stale command arrives.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "stale_callback_replacement_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("reference_timeout", 1.0);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  const std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), clearpath_mecanum_drive_controller::NR_REF_ITFS);

  const rclcpp::Time update_time = controller.get_node()->now();
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg valid_reference;
  valid_reference.header.stamp = update_time;
  valid_reference.twist.linear.x = 1.0;
  publish_stamped_reference(controller, valid_reference);
  auto stale_reference = valid_reference;
  stale_reference.header.stamp = update_time - rclcpp::Duration::from_seconds(2.0);
  publish_stamped_reference(controller, stale_reference);

  // WHEN the update loop consumes the latest buffered callback result.
  controller_interface::return_type stale_result{};
  EXPECT_NO_MALLOC(
    stale_result = controller.update_and_write_commands(
      update_time, rclcpp::Duration::from_seconds(0.01)));

  // THEN the stale command replaces the older motion, stops every wheel, and cannot replay.
  EXPECT_EQ(stale_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
    ASSERT_TRUE(command_interface->set_value(42.0));
  }
  controller_interface::return_type idle_result{};
  EXPECT_NO_MALLOC(
    idle_result = controller.update_and_write_commands(
      update_time + rclcpp::Duration::from_seconds(0.01),
      rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(idle_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 42.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, RetainedStampedReferenceStopsAtCommandTimeoutWithoutAllocation)
{
  // GIVEN a stamped command retained longer than the controller command timeout.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "retained_reference_command_timeout_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("reference_timeout", 1.0);
  parameters.emplace_back("command_timeout", 0.1);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  const std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), clearpath_mecanum_drive_controller::NR_REF_ITFS);
  const rclcpp::Time command_time = controller.get_node()->now();
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg reference;
  reference.header.stamp = command_time;
  reference.twist.linear.x = 1.0;
  publish_stamped_reference(controller, reference);
  ASSERT_EQ(
    controller.update_and_write_commands(
      command_time, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.05),
      rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // WHEN the command timeout expires without another subscriber message.
  controller_interface::return_type timeout_result{};
  EXPECT_NO_MALLOC(
    timeout_result = controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.11),
      rclcpp::Duration::from_seconds(0.01)));

  // THEN the retained target cannot refresh freshness and every wheel stops.
  EXPECT_EQ(timeout_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, BackwardTimeConsumesRetainedStampedReferenceWithoutAllocation)
{
  // GIVEN a subscriber-mode controller that has accepted and retained a stamped command.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "backward_retained_reference_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("reference_timeout", 1.0);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  const std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), clearpath_mecanum_drive_controller::NR_REF_ITFS);
  const rclcpp::Time command_time = controller.get_node()->now();
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg reference;
  reference.header.stamp = command_time;
  reference.twist.linear.x = 1.0;
  publish_stamped_reference(controller, reference);
  ASSERT_EQ(
    controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.1),
      rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // WHEN simulation time moves behind the retained command's timestamp.
  controller_interface::return_type backward_time_result{};
  EXPECT_NO_MALLOC(
    backward_time_result = controller.update_and_write_commands(
      command_time - rclcpp::Duration::from_seconds(0.1),
      rclcpp::Duration::from_seconds(0.01)));

  // THEN the controller stops and consumes the now-future command.
  EXPECT_EQ(backward_time_result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
    ASSERT_TRUE(command_interface->set_value(42.0));
  }
  controller_interface::return_type recovered_time_result{};
  EXPECT_NO_MALLOC(
    recovered_time_result = controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.2),
      rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(recovered_time_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 42.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, RejectedRetainedStampedReferenceCannotRestartMotion)
{
  // GIVEN a retained stamped command and a wheel interface that rejects the first write.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "rejected_retained_reference_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  parameters.emplace_back("reference_timeout", 1.0);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  const std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), clearpath_mecanum_drive_controller::NR_REF_ITFS);
  const rclcpp::Time command_time = controller.get_node()->now();
  clearpath_mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg reference;
  reference.header.stamp = command_time;
  reference.twist.linear.x = 1.0;
  publish_stamped_reference(controller, reference);
  std::unique_lock<std::shared_mutex> rejected_write_lock(command_interfaces[1]->get_mutex());

  // WHEN applying the retained command fails and a later update runs without a new message.
  controller_interface::return_type rejected_result{};
  EXPECT_NO_MALLOC(
    rejected_result = controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.01),
      rclcpp::Duration::from_seconds(0.01)));
  rejected_write_lock.unlock();
  EXPECT_EQ(rejected_result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    ASSERT_TRUE(command_interface->set_value(42.0));
  }
  controller_interface::return_type retry_result{};
  EXPECT_NO_MALLOC(
    retry_result = controller.update_and_write_commands(
      command_time + rclcpp::Duration::from_seconds(0.02),
      rclcpp::Duration::from_seconds(0.01)));

  // THEN the failed message is not reapplied and cannot restart motion.
  EXPECT_EQ(retry_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 42.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerParameters, RuntimeChangesToSnapshottedParametersAreRejected)
{
  // GIVEN a configured controller whose parameters have been copied into runtime state.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "read_only_parameter_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options =
    rclcpp::NodeOptions().parameter_overrides(valid_controller_parameters());
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  const std::array<std::string, 37> snapshotted_parameter_names{
    "use_stamped_vel",
    "reference_timeout",
    "command_timeout",
    "command_joint_names",
    "state_joint_names",
    "reference_joint_names",
    "interface_name",
    "kinematics.base_frame_offset.x",
    "kinematics.base_frame_offset.y",
    "kinematics.base_frame_offset.theta",
    "kinematics.wheels_radius",
    "kinematics.sum_of_robot_center_projection_on_X_Y_axis",
    "linear.x.has_velocity_limits",
    "linear.x.has_acceleration_limits",
    "linear.x.max_velocity",
    "linear.x.min_velocity",
    "linear.x.max_acceleration",
    "linear.x.min_acceleration",
    "linear.y.has_velocity_limits",
    "linear.y.has_acceleration_limits",
    "linear.y.max_velocity",
    "linear.y.min_velocity",
    "linear.y.max_acceleration",
    "linear.y.min_acceleration",
    "angular.z.has_velocity_limits",
    "angular.z.has_acceleration_limits",
    "angular.z.max_velocity",
    "angular.z.min_velocity",
    "angular.z.max_acceleration",
    "angular.z.min_acceleration",
    "base_frame_id",
    "odom_frame_id",
    "enable_odom_tf",
    "twist_covariance_diagonal",
    "pose_covariance_diagonal",
    "body_frame_control",
    "body_frame_yaw_joint",
  };
  for (const std::string & parameter_name : snapshotted_parameter_names)
  {
    EXPECT_TRUE(controller.get_node()->describe_parameter(parameter_name).read_only)
      << parameter_name;
  }

  // WHEN an operator attempts to mutate safety and kinematic parameters at runtime.
  const rcl_interfaces::msg::SetParametersResult result =
    controller.get_node()->set_parameters_atomically(
      { rclcpp::Parameter("kinematics.wheels_radius", 0.2),
        rclcpp::Parameter("linear.x.max_velocity", 0.1),
        rclcpp::Parameter("command_timeout", 2.0) });

  // THEN the node rejects the ineffective runtime configuration change.
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(controller.get_node()->get_parameter("kinematics.wheels_radius").as_double(), 0.1);
  EXPECT_EQ(controller.get_node()->get_parameter("linear.x.max_velocity").as_double(), 0.0);
  EXPECT_EQ(controller.get_node()->get_parameter("command_timeout").as_double(), 1.0);
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, WorldReferenceIsRotatedBeforeApplyingBodyFrameLimits)
{
  // GIVEN a world-frame controller at 45 degrees with unequal body X and Y limits.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "world_frame_limit_test";
  controller_params.controller_manager_update_rate = 100;
  std::vector<rclcpp::Parameter> parameters = valid_controller_parameters();
  apply_parameter_replacements(
    parameters,
    { rclcpp::Parameter("body_frame_control", false),
      rclcpp::Parameter("body_frame_yaw_joint", "base_yaw") });
  parameters.emplace_back("linear.x.has_velocity_limits", true);
  parameters.emplace_back("linear.x.min_velocity", -2.0);
  parameters.emplace_back("linear.x.max_velocity", 2.0);
  parameters.emplace_back("linear.y.has_velocity_limits", true);
  parameters.emplace_back("linear.y.min_velocity", -0.2);
  parameters.emplace_back("linear.y.max_velocity", 0.2);
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(parameters);
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  constexpr double kQuarterTurnRadians = 0.7853981633974483;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles, kQuarterTurnRadians);
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));

  // WHEN the controller processes the world-frame reference.
  const controller_interface::return_type result = controller.update_and_write_commands(
    rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01));

  // THEN it limits the rotated body Y component before mecanum kinematics.
  const double rotated_x = std::sqrt(0.5);
  const double high_wheel_velocity = (rotated_x + 0.2) / 0.1;
  const double low_wheel_velocity = (rotated_x - 0.2) / 0.1;
  ASSERT_EQ(result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    ASSERT_TRUE(command_interface->get_optional<double>().has_value());
  }
  EXPECT_NEAR(*command_interfaces[0]->get_optional<double>(), high_wheel_velocity, 1e-12);
  EXPECT_NEAR(*command_interfaces[1]->get_optional<double>(), low_wheel_velocity, 1e-12);
  EXPECT_NEAR(*command_interfaces[2]->get_optional<double>(), high_wheel_velocity, 1e-12);
  EXPECT_NEAR(*command_interfaces[3]->get_optional<double>(), low_wheel_velocity, 1e-12);
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, InvalidPeriodStopDoesNotAllocate)
{
  // GIVEN a configured chained controller with a finite motion reference.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "invalid_period_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options =
    rclcpp::NodeOptions().parameter_overrides(valid_controller_parameters());
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles);
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));

  // WHEN an invalid zero period reaches the production update path.
  controller_interface::return_type invalid_result{};
  EXPECT_NO_MALLOC(
    invalid_result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.0)));

  // THEN it fails closed before integration and the rejected command is not replayed.
  EXPECT_EQ(invalid_result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  controller_interface::return_type idle_result{};
  EXPECT_NO_MALLOC(
    idle_result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 10000000, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));
  EXPECT_EQ(idle_result, controller_interface::return_type::OK);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, NonFiniteOdometryStopDoesNotAllocate)
{
  // GIVEN finite wheel samples whose forward-kinematics sum overflows.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "odometry_overflow_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options =
    rclcpp::NodeOptions().parameter_overrides(valid_controller_parameters());
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(
      controller, &state_interface_handles, std::nullopt, std::numeric_limits<double>::max());
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));

  // WHEN the production update derives non-finite odometry.
  controller_interface::return_type result{};
  EXPECT_NO_MALLOC(
    result = controller.update_and_write_commands(
      rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(0.01)));

  // THEN the odometry failure propagates and every wheel is stopped.
  EXPECT_EQ(result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  rclcpp::shutdown();
}

TEST(MecanumDriveControllerUpdate, PoseIntegrationOverflowStopsWheelsAndReturnsError)
{
  // GIVEN finite wheel samples and a finite period whose position integration overflows.
  rclcpp::init(0, nullptr);
  clearpath_mecanum_drive_controller::MecanumDriveController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "pose_integration_overflow_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options =
    rclcpp::NodeOptions().parameter_overrides(valid_controller_parameters());
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const double extreme_velocity = std::numeric_limits<double>::max() / 8.0;
  const std::array<double, clearpath_mecanum_drive_controller::NR_STATE_ITFS> wheel_velocities{
    extreme_velocity, extreme_velocity, extreme_velocity, extreme_velocity
  };
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_handles;
  const std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
    assign_controller_interfaces(controller, &state_interface_handles, std::nullopt, 0.0, wheel_velocities);
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces =
    controller.export_reference_interfaces();
  ASSERT_TRUE(reference_interfaces[0]->set_value(1.0));
  ASSERT_TRUE(reference_interfaces[1]->set_value(0.0));
  ASSERT_TRUE(reference_interfaces[2]->set_value(0.0));
  ASSERT_TRUE(controller.set_chained_mode(true));

  // WHEN the production update derives a non-finite integrated position.
  const controller_interface::return_type result = controller.update_and_write_commands(
    rclcpp::Time{ 1, 0, RCL_ROS_TIME }, rclcpp::Duration::from_seconds(1.0e9));

  // THEN the late odometry failure propagates and every wheel is stopped.
  EXPECT_EQ(result, controller_interface::return_type::ERROR);
  for (const auto & command_interface : command_interfaces)
  {
    EXPECT_EQ(command_interface->get_optional<double>(), 0.0);
  }
  rclcpp::shutdown();
}
#endif

TEST(OdometryUpdate, NonFinitePeriodDoesNotMutateState)
{
  // GIVEN odometry with a known integrated pose.
  clearpath_mecanum_drive_controller::Odometry odometry;
  odometry.setWheelsParams(0.5, 0.1);
  ASSERT_TRUE(odometry.update(1.0, 1.0, 1.0, 1.0, 0.1));
  const double x_before_invalid_update = odometry.getX();

  // WHEN a non-finite integration period is supplied.
  const bool updated = odometry.update(
    2.0, 2.0, 2.0, 2.0, std::numeric_limits<double>::quiet_NaN());

  // THEN the update is rejected without changing the integrated state.
  EXPECT_FALSE(updated);
  EXPECT_EQ(odometry.getX(), x_before_invalid_update);
}

TEST(OdometryUpdate, DerivedOverflowDoesNotMutatePoseOrTwist)
{
  // GIVEN odometry with a known finite pose and twist.
  clearpath_mecanum_drive_controller::Odometry odometry;
  odometry.setWheelsParams(0.5, 0.1);
  ASSERT_TRUE(odometry.update(1.0, 1.0, 1.0, 1.0, 0.1));
  const std::array<double, 6> state_before_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };

  // WHEN finite extreme wheel samples overflow the derived forward kinematics.
  const double extreme_velocity = std::numeric_limits<double>::max();
  const bool updated = odometry.update(
    extreme_velocity, extreme_velocity, extreme_velocity, extreme_velocity, 0.1);

  // THEN the update fails without committing any pose or twist field.
  EXPECT_FALSE(updated);
  const std::array<double, 6> state_after_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };
  EXPECT_EQ(state_after_overflow, state_before_overflow);
}

TEST(OdometryUpdate, BaseTwistOverflowDoesNotMutatePoseOrTwist)
{
  // GIVEN finite odometry whose extreme base offset leaves ordinary straight motion valid.
  clearpath_mecanum_drive_controller::Odometry odometry;
  odometry.init(
    rclcpp::Time{ 0, 0, RCL_ROS_TIME },
    { std::numeric_limits<double>::max(), 0.0, 0.0 });
  odometry.setWheelsParams(0.5, 0.1);
  ASSERT_TRUE(odometry.update(1.0, 1.0, 1.0, 1.0, 0.1));
  const std::array<double, 6> state_before_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };

  // WHEN finite pure-yaw wheel samples overflow only the transformed base-frame twist.
  const bool updated = odometry.update(-10.0, -10.0, 10.0, 10.0, 0.1);

  // THEN the update fails without committing any pose or twist field.
  EXPECT_FALSE(updated);
  const std::array<double, 6> state_after_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };
  EXPECT_EQ(state_after_overflow, state_before_overflow);
}

TEST(OdometryUpdate, PoseIntegrationOverflowDoesNotMutatePoseOrTwist)
{
  // GIVEN odometry with a known finite pose and twist.
  clearpath_mecanum_drive_controller::Odometry odometry;
  odometry.setWheelsParams(0.5, 0.1);
  ASSERT_TRUE(odometry.update(1.0, 1.0, 1.0, 1.0, 0.1));
  const std::array<double, 6> state_before_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };

  // WHEN finite straight motion and a finite extreme period overflow only pose integration.
  const bool updated = odometry.update(
    20.0, 20.0, 20.0, 20.0, std::numeric_limits<double>::max());

  // THEN the update fails without committing any pose or twist field.
  EXPECT_FALSE(updated);
  const std::array<double, 6> state_after_overflow{
    odometry.getX(), odometry.getY(), odometry.getRz(),
    odometry.getVx(), odometry.getVy(), odometry.getWz()
  };
  EXPECT_EQ(state_after_overflow, state_before_overflow);
}

TEST(CommandTimeoutValidation, RejectsNonFiniteAndNonPositiveValues)
{
  // GIVEN timeout values that cannot safely bound continuing wheel motion.
  const std::array<double, 5> invalid_values{
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), 0.0, -1.0
  };

  for (const double value : invalid_values)
  {
    // WHEN the generated-parameter validator checks the timeout.
    const tl::expected<void, std::string> result =
      clearpath_mecanum_drive_controller::finite_positive(
        rclcpp::Parameter("command_timeout", value));

    // THEN configuration rejects the unsafe value.
    EXPECT_FALSE(result.has_value());
  }
}

TEST(CommandTimeoutValidation, AcceptsFinitePositiveValue)
{
  // GIVEN a finite, strictly positive command timeout.
  const rclcpp::Parameter parameter("command_timeout", 1.0);

  // WHEN the generated-parameter validator checks the timeout.
  const tl::expected<void, std::string> result =
    clearpath_mecanum_drive_controller::finite_positive(parameter);

  // THEN controller configuration may continue.
  EXPECT_TRUE(result.has_value());
}

TEST(MecanumDriveControllerConfiguration, RejectsWrongCommandJointCount)
{
  // GIVEN fewer than the four command joints required by mecanum kinematics.
  // WHEN controller configuration validates the interface layout.
  // THEN configuration fails before any update can index wheel interfaces.
  expect_configuration_result(
    { rclcpp::Parameter(
        "command_joint_names", std::vector<std::string>{ "wheel_0", "wheel_1", "wheel_2" }) },
    controller_interface::CallbackReturn::FAILURE);
}

TEST(MecanumDriveControllerConfiguration, RejectsWrongStateJointCount)
{
  // GIVEN fewer than the four state joints required by mecanum odometry.
  // WHEN controller configuration validates the interface layout.
  // THEN configuration fails before any update can index wheel interfaces.
  expect_configuration_result(
    { rclcpp::Parameter(
        "state_joint_names", std::vector<std::string>{ "wheel_0", "wheel_1", "wheel_2" }) },
    controller_interface::CallbackReturn::FAILURE);
}

TEST(MecanumDriveControllerConfiguration, RejectsEmptyInterfaceName)
{
  // GIVEN an empty hardware-interface name.
  // WHEN controller configuration validates the interface contract.
  // THEN configuration fails rather than requesting malformed interfaces.
  expect_configuration_result(
    { rclcpp::Parameter("interface_name", "") },
    controller_interface::CallbackReturn::FAILURE);
}

TEST(MecanumDriveControllerConfiguration, RejectsUnsafeGeometry)
{
  // GIVEN nonpositive or non-finite wheel geometry.
  const std::array<rclcpp::Parameter, 4> invalid_geometry{
    rclcpp::Parameter("kinematics.wheels_radius", 0.0),
    rclcpp::Parameter("kinematics.wheels_radius", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter("kinematics.sum_of_robot_center_projection_on_X_Y_axis", -1.0),
    rclcpp::Parameter(
      "kinematics.sum_of_robot_center_projection_on_X_Y_axis",
      std::numeric_limits<double>::quiet_NaN()),
  };

  for (const rclcpp::Parameter & parameter : invalid_geometry)
  {
    // WHEN controller configuration validates the geometry.
    // THEN it fails before division or odometry setup can use the value.
    expect_configuration_result(
      { parameter }, controller_interface::CallbackReturn::FAILURE);
  }
}

TEST(MecanumDriveControllerConfiguration, RejectsInvalidEnabledLimits)
{
  // GIVEN enabled velocity limits with the default equal zero bounds.
  // WHEN controller configuration validates cross-field limit invariants.
  // THEN configuration fails before the command path can claim to enforce them.
  expect_configuration_result(
    { rclcpp::Parameter("linear.x.has_velocity_limits", true) },
    controller_interface::CallbackReturn::FAILURE);
}

TEST(MecanumDriveControllerConfiguration, AcceptsValidBodyFrameYawCombinations)
{
  // GIVEN each supported body-frame/yaw configuration.
  // WHEN controller configuration validates the frame contract.
  // THEN both valid combinations configure successfully.
  expect_configuration_result(
    { rclcpp::Parameter("body_frame_control", true),
      rclcpp::Parameter("body_frame_yaw_joint", "") },
    controller_interface::CallbackReturn::SUCCESS);
  expect_configuration_result(
    { rclcpp::Parameter("body_frame_control", false),
      rclcpp::Parameter("body_frame_yaw_joint", "base_yaw") },
    controller_interface::CallbackReturn::SUCCESS);
}

class GeneratedCommandTimeoutValidation : public ::testing::TestWithParam<double>
{
};

TEST_P(GeneratedCommandTimeoutValidation, RejectsInvalidOverride)
{
  // GIVEN an unsafe node override that bypasses the schema's safe default.
  rclcpp::init(0, nullptr);
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
    { rclcpp::Parameter("command_timeout", GetParam()) });
  const std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("invalid_timeout_test", options);

  // WHEN the generated listener declares and validates its parameters.
  // THEN the schema wiring rejects the unsafe override.
  EXPECT_THROW(
    {
      const clearpath_mecanum_drive_controller::ParamListener listener{ node };
      static_cast<void>(listener);
    },
    rclcpp::exceptions::InvalidParameterValueException);
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(
  UnsafeTimeouts, GeneratedCommandTimeoutValidation,
  ::testing::Values(
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), 0.0, -1.0),
  [](const ::testing::TestParamInfo<double>& info) { return "Case" + std::to_string(info.index); });

TEST(GeneratedParameterValidation, RejectsInvalidSnapshottedNumericValues)
{
  // GIVEN numeric overrides that cannot safely initialize timeout, odometry, or covariance state.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<std::vector<rclcpp::Parameter>> invalid_parameter_sets{
    { rclcpp::Parameter("reference_timeout", -1.0) },
    { rclcpp::Parameter("reference_timeout", nan) },
    { rclcpp::Parameter("reference_timeout", infinity) },
    { rclcpp::Parameter("reference_timeout", -infinity) },
    { rclcpp::Parameter("kinematics.base_frame_offset.x", nan) },
    { rclcpp::Parameter("kinematics.base_frame_offset.y", infinity) },
    { rclcpp::Parameter("kinematics.base_frame_offset.theta", -infinity) },
    { rclcpp::Parameter(
        "pose_covariance_diagonal", std::vector<double>{ 0.1, 0.1, nan, 0.1, 0.1, 0.1 }) },
    { rclcpp::Parameter(
        "pose_covariance_diagonal", std::vector<double>{ 0.1, 0.1, -0.1, 0.1, 0.1, 0.1 }) },
    { rclcpp::Parameter(
        "twist_covariance_diagonal",
        std::vector<double>{ 0.1, 0.1, infinity, 0.1, 0.1, 0.1 }) },
    { rclcpp::Parameter(
        "twist_covariance_diagonal",
        std::vector<double>{ 0.1, 0.1, -infinity, 0.1, 0.1, 0.1 }) },
  };

  rclcpp::init(0, nullptr);
  for (size_t index = 0; index < invalid_parameter_sets.size(); ++index)
  {
    const rclcpp::NodeOptions options =
      rclcpp::NodeOptions().parameter_overrides(invalid_parameter_sets[index]);
    const std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("invalid_numeric_parameter_" + std::to_string(index), options);

    // WHEN the generated listener declares and validates the parameters.
    // THEN it rejects the override before controller configuration can consume it.
    EXPECT_THROW(
      {
        const clearpath_mecanum_drive_controller::ParamListener listener{ node };
        static_cast<void>(listener);
      },
      rclcpp::exceptions::InvalidParameterValueException)
      << "invalid parameter set " << index;
  }
  rclcpp::shutdown();
}

TEST(GeneratedParameterValidation, AcceptsSafeNumericBoundaries)
{
  // GIVEN valid boundary values: disabled reference retention, finite signed offsets, and zero covariance.
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
    { rclcpp::Parameter("reference_timeout", 0.0),
      rclcpp::Parameter("kinematics.base_frame_offset.x", -1.0),
      rclcpp::Parameter("kinematics.base_frame_offset.y", 0.0),
      rclcpp::Parameter("kinematics.base_frame_offset.theta", 1.0),
      rclcpp::Parameter("pose_covariance_diagonal", std::vector<double>(6, 0.0)),
      rclcpp::Parameter("twist_covariance_diagonal", std::vector<double>(6, 0.0)) });
  rclcpp::init(0, nullptr);
  const std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("valid_numeric_parameter_boundaries", options);

  // WHEN the generated listener validates the safe boundaries.
  // THEN configuration input construction succeeds.
  EXPECT_NO_THROW({
    const clearpath_mecanum_drive_controller::ParamListener listener{ node };
    static_cast<void>(listener);
  });
  rclcpp::shutdown();
}
