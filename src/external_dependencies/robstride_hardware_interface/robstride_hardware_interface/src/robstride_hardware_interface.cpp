// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0

#include "robstride_hardware_interface/robstride_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>

namespace robstride_hardware_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

RobstrideHardware::RobstrideHardware()
: rclcpp::Node("robstride_hardware_interface"),
  logger_(rclcpp::get_logger("robstride_hardware_interface"))
{
}

RobstrideHardware::~RobstrideHardware()
{
  if (service_executor_) {
    service_executor_->cancel();
  }
  if (service_executor_thread_.joinable()) {
    service_executor_thread_.join();
  }
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }
}

namespace
{
std::string GetJointParam(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = joint.parameters.find(key);
  return it == joint.parameters.end() ? default_value : it->second;
}

bool HasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & info) {return info.name == name;});
}

// The two timestamps come from different threads, so last_ns can land
// slightly after now_ns. Clamp to 0 instead of wrapping around.
double ElapsedMsSinceUpdate(uint64_t now_ns, uint64_t last_ns)
{
  if (last_ns == 0) {return -1.0;}
  if (last_ns >= now_ns) {return 0.0;}
  return static_cast<double>(now_ns - last_ns) / 1.0e6;
}

// How often write() re-sends the re-enable burst to an untorqued joint.
constexpr uint64_t kRecoveryResendNs = 200'000'000ull;

// CAN_TIMEOUT (0x7028) in its native 1/20000 s units, clamped to range.
// Zero (or negative) disables the motor's watchdog entirely.
uint32_t CanTimeoutRaw(double timeout_ms)
{
  if (timeout_ms <= 0.0) {
    return 0u;
  }
  return std::max(1u, static_cast<uint32_t>(std::min(timeout_ms * 20.0, 100000.0)));
}

constexpr double kTwoPi = 2.0 * M_PI;

// Map x into [center - pi, center + pi).
double WrapToWindow(double x, double center)
{
  return x - kTwoPi * std::floor((x - center + M_PI) / kTwoPi);
}

bool IsEnabled(const robstride_sdk::MotorState & st)
{
  return st.run_state.load(std::memory_order_relaxed) ==
         static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
}
}  // namespace

CallbackReturn RobstrideHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    if (info_.hardware_parameters.count("master_id")) {
      master_id_ = static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("master_id")));
    }
    if (info_.hardware_parameters.count("error_timeout_ms")) {
      error_timeout_ms_ = std::stod(info_.hardware_parameters.at("error_timeout_ms"));
    }
    // Defaults to error_timeout_ms; 0 lets a motor keep holding its last
    // target when the bus goes silent instead of releasing torque.
    motor_can_timeout_ms_ = error_timeout_ms_;
    if (info_.hardware_parameters.count("motor_can_timeout_ms")) {
      motor_can_timeout_ms_ = std::stod(info_.hardware_parameters.at("motor_can_timeout_ms"));
    }
    if (info_.hardware_parameters.count("freeze_on_joint_loss")) {
      const std::string & v = info_.hardware_parameters.at("freeze_on_joint_loss");
      freeze_on_joint_loss_ = v == "true" || v == "True" || v == "1";
    }
    if (info_.hardware_parameters.count("torque_enable")) {
      const std::string & v = info_.hardware_parameters.at("torque_enable");
      torque_enabled_desired_.store(v == "true" || v == "True" || v == "1");
    }
    if (info_.hardware_parameters.count("hold_torque_on_deactivate")) {
      const std::string & v = info_.hardware_parameters.at("hold_torque_on_deactivate");
      hold_torque_on_deactivate_ = v == "true" || v == "True" || v == "1";
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Malformed hardware parameter (master_id/error_timeout_ms/motor_can_timeout_ms): " << e.what());
    return CallbackReturn::ERROR;
  }
  if (motor_can_timeout_ms_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Motor CAN watchdog disabled (motor_can_timeout_ms <= 0): if the bus dies, motors keep "
      "holding their last target until a clean shutdown or a power cut.");
  }
  // The two settings are coupled: deactivation closes the bus, so a motor
  // whose own watchdog is still armed releases torque a short time after the
  // frames stop regardless of what on_deactivate() chose not to send.
  if (hold_torque_on_deactivate_ && motor_can_timeout_ms_ > 0.0) {
    RCLCPP_WARN_STREAM(
      logger_,
      "hold_torque_on_deactivate is set but the motor CAN watchdog is armed at " <<
        motor_can_timeout_ms_ << " ms, so each motor will release torque that long after "
        "deactivation and the arm will drop. Set motor_can_timeout_ms to 0 to hold the pose "
        "after a stop.");
  }

  const size_t n = info_.joints.size();

  // Optional cross-check only: catches a description that lost or gained
  // a joint during editing.
  if (info_.hardware_parameters.count("number_of_joints")) {
    try {
      const size_t declared =
        static_cast<size_t>(std::stoul(info_.hardware_parameters.at("number_of_joints")));
      if (declared != n) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Hardware declares number_of_joints=" << declared << " but the description has " <<
            n << " <joint> block(s)");
        return CallbackReturn::ERROR;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Malformed hardware parameter number_of_joints: " << e.what());
      return CallbackReturn::ERROR;
    }
  }

  joints_.resize(n);
  hw_state_position_.assign(n, 0.0);
  hw_state_velocity_.assign(n, 0.0);
  hw_state_effort_.assign(n, 0.0);
  hw_cmd_position_.assign(n, 0.0);
  hw_cmd_velocity_.assign(n, 0.0);
  freeze_position_.assign(n, 0.0);
  wrap_offset_.assign(n, 0.0);
  seen_enabled_.assign(n, false);
  last_recovery_ns_.assign(n, 0);
  hw_cmd_effort_.assign(n, 0.0);
  hw_cmd_kp_.assign(n, 0.0);
  hw_cmd_kd_.assign(n, 0.0);
  hw_state_temperature_.assign(n, 0.0);
  hw_state_run_state_.assign(n, 0.0);
  hw_state_fault_bits_.assign(n, 0.0);
  pending_kp_ = std::vector<std::atomic<double>>(n);
  pending_kd_ = std::vector<std::atomic<double>>(n);
  has_pending_kp_ = std::vector<std::atomic<bool>>(n);
  has_pending_kd_ = std::vector<std::atomic<bool>>(n);
  for (size_t i = 0; i < n; ++i) {
    has_pending_kp_[i].store(false);
    has_pending_kd_[i].store(false);
  }

  motors_.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    const std::string id_str = GetJointParam(joint, "id");
    if (id_str.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "' is missing its 'id' param");
      return CallbackReturn::ERROR;
    }

    JointHandle & jh = joints_[i];
    jh.joint_name = joint.name;
    try {
      jh.id = static_cast<uint8_t>(std::stoi(id_str));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "': " << e.what());
      return CallbackReturn::ERROR;
    }

    // Device configuration lives in the <gpio> block, matched by id.
    const hardware_interface::ComponentInfo * gpio = nullptr;
    for (const auto & candidate : info_.gpios) {
      const std::string gpio_id = GetJointParam(candidate, "ID");
      if (!gpio_id.empty() && gpio_id == id_str) {
        gpio = &candidate;
        break;
      }
    }
    if (gpio == nullptr) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << joint.name << "' (id=" << id_str <<
          ") has no matching <gpio> block. Every joint needs one declaring "
          "ID / actuator_type / can_interface / control_mode.");
      return CallbackReturn::ERROR;
    }
    jh.gpio_name = gpio->name;

    const std::string type_str = GetJointParam(*gpio, "actuator_type");
    const std::string can_if = GetJointParam(*gpio, "can_interface");
    if (type_str.empty() || can_if.empty()) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "gpio '" << gpio->name << "' is missing a required param "
          "(actuator_type / can_interface)");
      return CallbackReturn::ERROR;
    }

    try {
      jh.actuator_type = robstride_sdk::ActuatorTypeFromString(type_str);
      jh.control_mode = robstride_sdk::RunModeFromString(GetJointParam(*gpio, "control_mode", ""));
      hw_cmd_kp_[i] = std::stod(GetJointParam(*gpio, "kp", "0.0"));
      hw_cmd_kd_[i] = std::stod(GetJointParam(*gpio, "kd", "0.0"));
      jh.direction = std::stod(GetJointParam(*gpio, "direction", "1"));
      if (jh.direction != 1.0 && jh.direction != -1.0) {
        throw std::invalid_argument("direction must be 1 or -1");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "gpio '" << gpio->name << "': " << e.what());
      return CallbackReturn::ERROR;
    }
    jh.can_interface = can_if;

    // A single-turn reading can only be placed unambiguously inside a
    // window narrower than one turn, so center it on the joint's range.
    const auto lim = info_.limits.find(joint.name);
    if (lim != info_.limits.end() && lim->second.has_position_limits) {
      const double lower = lim->second.min_position;
      const double upper = lim->second.max_position;
      jh.wrap_center = 0.5 * (lower + upper);
      if (upper - lower >= kTwoPi) {
        RCLCPP_WARN_STREAM(
          logger_,
          "Joint '" << joint.name << "' spans " << (upper - lower) << " rad, a full turn or "
            "more: its position is reported modulo 2*pi around " << jh.wrap_center <<
            ", so the ends of its range alias onto each other.");
      }
    } else {
      jh.continuous = true;
    }

    // The declared command interface must be the one this mode writes, or
    // a controller would claim it and the joint would never move.
    const char * required_command = nullptr;
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        required_command = hardware_interface::HW_IF_VELOCITY;
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        required_command = hardware_interface::HW_IF_EFFORT;
        break;
      default:
        required_command = hardware_interface::HW_IF_POSITION;
        break;
    }
    if (!HasInterface(joint.command_interfaces, required_command)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << joint.name << "' runs in control_mode '" <<
          GetJointParam(*gpio, "control_mode", "motion") << "', which is driven through the '" <<
          required_command << "' command interface, but the joint does not declare it.");
      return CallbackReturn::ERROR;
    }

    motors_.push_back(
      std::make_unique<robstride_sdk::RobstrideMotor>(jh.id, master_id_, jh.actuator_type));
    jh.motor = motors_.back().get();

    if (buses_.find(can_if) == buses_.end()) {
      buses_[can_if] = std::make_unique<robstride_sdk::CanBus>(can_if);
    }
    buses_[can_if]->RegisterMotor(jh.motor);
  }

  state_pub_ = this->create_publisher<RobstrideStateMsg>(
    "~/robstride_state", rclcpp::SystemDefaultsQoS());
  state_pub_uni_ptr_ = std::make_unique<StatePublisher>(state_pub_);
  state_pub_uni_ptr_->msg_.id.resize(n);
  state_pub_uni_ptr_->msg_.enabled.resize(n);
  state_pub_uni_ptr_->msg_.run_state.resize(n);
  state_pub_uni_ptr_->msg_.fault_bits.resize(n);

  get_data_srv_ = this->create_service<robstride_interfaces::srv::GetDataFromRobstride>(
    "~/get_data_from_robstride",
    std::bind(
      &RobstrideHardware::GetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_data_srv_ = this->create_service<robstride_interfaces::srv::SetDataToRobstride>(
    "~/set_data_to_robstride",
    std::bind(
      &RobstrideHardware::SetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_zero_srv_ = this->create_service<robstride_interfaces::srv::SetZeroRobstride>(
    "~/set_zero_robstride",
    std::bind(
      &RobstrideHardware::SetZeroCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_torque_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_torque",
    std::bind(
      &RobstrideHardware::SetTorqueCallback, this, std::placeholders::_1, std::placeholders::_2));
  reboot_srv_ = this->create_service<robstride_interfaces::srv::RebootRobstride>(
    "~/reboot_robstride",
    std::bind(
      &RobstrideHardware::RebootCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(this->get_node_base_interface());
  service_executor_thread_ = std::thread([this]() {service_executor_->spin();});

  RCLCPP_INFO_STREAM(
    logger_,
    "Initialized " << n << " RobStride joint(s) across " << buses_.size() << " CAN bus(es)");
  return CallbackReturn::SUCCESS;
}

// Both exports follow the URDF declaration: an undeclared interface must
// not be claimable.
std::vector<hardware_interface::StateInterface> RobstrideHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & declared = info_.joints[i].state_interfaces;
    const std::string & name = joints_[i].joint_name;
    if (HasInterface(declared, hardware_interface::HW_IF_POSITION)) {
      state_interfaces.emplace_back(
        name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_VELOCITY)) {
      state_interfaces.emplace_back(
        name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_EFFORT)) {
      state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_state_effort_[i]);
    }

    const auto * gpio = FindGpio(joints_[i].gpio_name);
    if (gpio == nullptr) {
      continue;
    }
    if (HasInterface(gpio->state_interfaces, "temperature")) {
      state_interfaces.emplace_back(gpio->name, "temperature", &hw_state_temperature_[i]);
    }
    if (HasInterface(gpio->state_interfaces, "run_state")) {
      state_interfaces.emplace_back(gpio->name, "run_state", &hw_state_run_state_[i]);
    }
    if (HasInterface(gpio->state_interfaces, "fault_bits")) {
      state_interfaces.emplace_back(gpio->name, "fault_bits", &hw_state_fault_bits_[i]);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & declared = info_.joints[i].command_interfaces;
    const std::string & name = joints_[i].joint_name;
    if (HasInterface(declared, hardware_interface::HW_IF_POSITION)) {
      command_interfaces.emplace_back(
        name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_VELOCITY)) {
      command_interfaces.emplace_back(
        name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_EFFORT)) {
      command_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_[i]);
    }

    const auto * gpio = FindGpio(joints_[i].gpio_name);
    if (gpio == nullptr) {
      continue;
    }
    if (HasInterface(gpio->command_interfaces, "kp")) {
      command_interfaces.emplace_back(gpio->name, "kp", &hw_cmd_kp_[i]);
    }
    if (HasInterface(gpio->command_interfaces, "kd")) {
      command_interfaces.emplace_back(gpio->name, "kd", &hw_cmd_kd_[i]);
    }
  }
  return command_interfaces;
}

CallbackReturn RobstrideHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reactivation is the only way out of a freeze; see read().
  frozen_ = false;
  std::fill(seen_enabled_.begin(), seen_enabled_.end(), false);

  for (auto & kv : buses_) {
    if (!kv.second->Open()) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open CAN interface '" << kv.first << "'");
      return CallbackReturn::ERROR;
    }
    kv.second->Start();
  }

  // Arm each motor's own CAN watchdog, a second line of defence beside the
  // host-side staleness check. RAM-resident, so lost on a power cycle.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(
      jh.motor->EncodeParamWriteU32(
        robstride_sdk::ParamIndex::CAN_TIMEOUT, CanTimeoutRaw(motor_can_timeout_ms_)));
  }

  // Select each joint's run_mode (0x7005) before enabling.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(
      jh.motor->EncodeParamWriteU8(
        robstride_sdk::ParamIndex::RUN_MODE, static_cast<uint8_t>(jh.control_mode)));
  }

  // Activation must never produce motion. The closed-loop modes keep their
  // target inside the motor, so enabling with a stale loc_ref would servo
  // back to wherever it was last aimed. Preload every mode's target from
  // the measured position first. Stop frames are answered with the same
  // Type-2 feedback, so send one as a position probe: harmless on an
  // already-disabled motor, and it also clears a crashed run's leftovers.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(jh.motor->EncodeDisable(false));
  }

  const uint64_t probe_start_ns = robstride_sdk::NowNs();
  const auto probe_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(error_timeout_ms_));
  auto probed = [probe_start_ns](const JointHandle & jh) {
      return jh.motor->state().last_update_ns.load(std::memory_order_relaxed) > probe_start_ns;
    };
  while (std::chrono::steady_clock::now() < probe_deadline &&
    !std::all_of(joints_.begin(), joints_.end(), probed))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    JointHandle & jh = joints_[i];
    if (!probed(jh)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << jh.joint_name << "' (id=" << static_cast<int>(jh.id) <<
          ") reported no position within " << error_timeout_ms_ <<
          " ms; refusing to enable torque against an unknown target");
      for (auto & other : joints_) {
        buses_[other.can_interface]->SendFrame(other.motor->EncodeDisable(false));
      }
      for (auto & kv : buses_) {
        kv.second->Stop();
        kv.second->Close();
      }
      return CallbackReturn::ERROR;
    }

    // Same source write() uses one cycle later, not MECH_POS: a target in a
    // different frame would just move the jump one cycle later.
    const float measured = jh.motor->state().position.load(std::memory_order_relaxed);
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_POSITION_PP:
      case robstride_sdk::RunMode::RUN_MODE_POSITION_CSP:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::LOC_REF, measured));
        break;
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::SPD_REF, 0.0f));
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::IQ_REF, 0.0f));
        break;
      default:
        break;
    }

    // Seed the command interfaces from the same reading.
    hw_state_position_[i] = WrapMotorPosition(i, measured);
    hw_cmd_position_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }

  // Activating without torque is a supported end state: the motors keep
  // answering the Stop frames write() sends, so state still reads while
  // every joint can be moved by hand. Nothing to confirm here.
  if (!torque_enabled_desired_.load(std::memory_order_relaxed)) {
    is_active_ = true;
    RCLCPP_INFO(
      logger_,
      "RobStride hardware activated with torque disabled (torque_enable=false). Joints report "
      "state but are free to move; call ~/set_torque to engage.");
    return CallbackReturn::SUCCESS;
  }

  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(jh.motor->EncodeEnable());
  }

  // Wait for each motor to confirm it is enabled. Motors are reused across
  // reactivations, so require a timestamp newer than this activation's own
  // start rather than merely non-zero.
  const uint64_t activate_start_ns = robstride_sdk::NowNs();
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(error_timeout_ms_));
  std::vector<bool> confirmed(joints_.size(), false);
  size_t num_confirmed = 0;
  auto next_resend = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (num_confirmed < joints_.size() && std::chrono::steady_clock::now() < deadline) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (confirmed[i]) {continue;}
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      if (st.last_update_ns.load(std::memory_order_relaxed) > activate_start_ns &&
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR))
      {
        confirmed[i] = true;
        ++num_confirmed;
      }
    }
    if (num_confirmed < joints_.size()) {
      // Enable frames are fire-and-forget; resend in case one was dropped.
      if (std::chrono::steady_clock::now() >= next_resend) {
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (!confirmed[i]) {
            buses_[joints_[i].can_interface]->SendFrame(joints_[i].motor->EncodeEnable());
          }
        }
        next_resend += std::chrono::milliseconds(200);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (num_confirmed < joints_.size()) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!confirmed[i]) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Joint '" << joints_[i].joint_name << "' (id=" <<
            static_cast<int>(joints_[i].id) << ") did not confirm enable within " <<
            error_timeout_ms_ << " ms");
      }
    }
    // A silent motor and a bus that never transmitted look identical here.
    for (auto & kv : buses_) {
      const int err = kv.second->last_send_errno();
      if (err != 0) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "CAN interface '" << kv.first << "' failed to transmit (" << std::strerror(err) <<
            "), so the enable frames above may never have reached the bus. This usually "
            "means the interface is bus-off; on an adapter without bus-off restart support "
            "it must be re-plugged and re-configured with scripts/setup_can.sh.");
      }
    }
    for (auto & jh : joints_) {
      buses_[jh.can_interface]->SendFrame(jh.motor->EncodeDisable(false));
    }
    for (auto & kv : buses_) {
      kv.second->Stop();
      kv.second->Close();
    }
    return CallbackReturn::ERROR;
  }

  // Seed the command interfaces from measured state so write()'s first
  // cycle holds position instead of snapping to 0.
  for (size_t i = 0; i < joints_.size(); ++i) {
    hw_state_position_[i] =
      WrapMotorPosition(i, joints_[i].motor->state().position.load(std::memory_order_relaxed));
    hw_cmd_position_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }

  is_active_ = true;
  RCLCPP_INFO(logger_, "RobStride hardware activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobstrideHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  // Disabling torque is the right default for a rig that can rest unpowered,
  // but it drops a gravity-loaded arm wherever it happens to be standing. With
  // hold_torque_on_deactivate the motors keep their last target, so a stop
  // holds the pose; nothing new is commanded either way.
  if (hold_torque_on_deactivate_) {
    RCLCPP_WARN(
      logger_,
      "Deactivating with torque engaged (hold_torque_on_deactivate=true): the joints stay "
      "rigid at their current pose. Cut motor power to make the arm limp.");
  } else {
    for (auto & jh : joints_) {
      const auto it = buses_.find(jh.can_interface);
      if (it != buses_.end()) {
        it->second->SendFrame(jh.motor->EncodeDisable(false));
      }
    }
  }
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }

  RCLCPP_INFO(logger_, "RobStride hardware deactivated");
  return CallbackReturn::SUCCESS;
}

return_type RobstrideHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  bool any_fault = false;
  bool any_lost = false;
  std::string lost_joints;
  const uint64_t now_ns = robstride_sdk::NowNs();
  const bool torque_desired = torque_enabled_desired_.load(std::memory_order_relaxed);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const robstride_sdk::MotorState & st = joints_[i].motor->state();
    const double dir = joints_[i].direction;
    hw_state_position_[i] = WrapMotorPosition(i, st.position.load(std::memory_order_relaxed));
    hw_state_velocity_[i] = dir * st.velocity.load(std::memory_order_relaxed);
    hw_state_effort_[i] = dir * st.torque.load(std::memory_order_relaxed);
    hw_state_temperature_[i] = st.temperature.load(std::memory_order_relaxed);
    hw_state_run_state_[i] =
      static_cast<double>(st.run_state.load(std::memory_order_relaxed));
    hw_state_fault_bits_[i] =
      static_cast<double>(st.fault_bits.load(std::memory_order_relaxed));
    any_fault |= st.fault_present.load(std::memory_order_relaxed);

    // While untorqued, track the command to the measured position so the
    // motor has a fresh target the instant it re-engages.
    const bool enabled = IsEnabled(st);
    if (!enabled) {
      hw_cmd_position_[i] = hw_state_position_[i];
      hw_cmd_velocity_[i] = 0.0;
      hw_cmd_effort_[i] = 0.0;
      // Same for the frozen hold target: a joint that dropped out may have
      // moved while limp, so it must resume holding where it actually is.
      freeze_position_[i] = hw_state_position_[i];
    }

    if (!is_active_) {
      continue;
    }

    // A joint that was enabled and no longer is has rebooted (a power blip
    // clears its RAM) or faulted. Its position may now be a whole turn off
    // and a controller may still hold a setpoint from before, so this is a
    // loss even when it answers again well inside error_timeout_ms.
    bool dropped_out = false;
    if (!torque_desired) {
      seen_enabled_[i] = false;
    } else if (enabled) {
      seen_enabled_[i] = true;
    } else {
      dropped_out = seen_enabled_[i];
    }

    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    const bool stale = elapsed_ms < 0.0 || elapsed_ms > error_timeout_ms_;
    if (stale || dropped_out) {
      any_lost = true;
      // Collected, not logged here: _THROTTLE suppresses per call site, so
      // logging in the loop would hide all but one joint.
      if (!lost_joints.empty()) {
        lost_joints += ", ";
      }
      lost_joints += joints_[i].joint_name + (stale ? " (stale)" : " (left enabled state)");
    }
  }

  if (state_pub_uni_ptr_ && state_pub_uni_ptr_->trylock()) {
    auto & msg = state_pub_uni_ptr_->msg_;
    msg.header.stamp = time;
    msg.comm_state = any_fault ? -1 : 0;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      msg.id[i] = joints_[i].id;
      msg.enabled[i] =
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
      msg.run_state[i] = st.run_state.load(std::memory_order_relaxed);
      msg.fault_bits[i] = st.fault_bits.load(std::memory_order_relaxed);
    }
    state_pub_uni_ptr_->unlockAndPublish();
  }

  // Services are spun on their own thread; no spin_some() on this cycle.

  if (!any_lost && !frozen_) {
    return return_type::OK;
  }

  if (!any_lost) {
    // Frozen but reporting again. Not resumed automatically: a link that
    // dropped once can drop again, and the controllers' setpoints moved on.
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 5000,
      "All joints reporting again but still frozen, holding position and ignoring "
        "controller commands. Reactivate the hardware component to resume control.");
    return return_type::OK;
  }

  if (!freeze_on_joint_loss_) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 1000,
      "Lost [" << lost_joints << "]; deactivating hardware");
    return return_type::ERROR;
  }

  // ERROR here would deactivate the whole component and cut torque on every
  // joint. Freeze instead: hold the measured positions and ignore commands.
  if (!frozen_) {
    frozen_ = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      freeze_position_[i] = hw_state_position_[i];
    }
    RCLCPP_ERROR_STREAM(
      logger_,
      "Lost [" << lost_joints << "]; holding all reachable joints at their current "
        "position and ignoring controller commands. A lost joint is not re-enabled. "
        "Reactivate the hardware component to resume control once the cause has been "
        "dealt with.");
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 5000,
      "Still frozen; lost [" << lost_joints << "]");
  }
  return return_type::OK;
}

return_type RobstrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const uint64_t now_ns = robstride_sdk::NowNs();
  const bool torque_desired = torque_enabled_desired_.load(std::memory_order_relaxed);

  std::unordered_map<std::string, std::vector<can_frame>> frames_by_bus;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointHandle & jh = joints_[i];
    const robstride_sdk::MotorState & st = jh.motor->state();

    // Apply any gain change parked by SetDataCallback, keeping the service
    // thread away from the buffers this cycle reads.
    if (has_pending_kp_[i].exchange(false, std::memory_order_relaxed)) {
      hw_cmd_kp_[i] = pending_kp_[i].load(std::memory_order_relaxed);
    }
    if (has_pending_kd_[i].exchange(false, std::memory_order_relaxed)) {
      hw_cmd_kd_[i] = pending_kd_[i].load(std::memory_order_relaxed);
    }

    if (!torque_desired) {
      // Sent from this RT cycle, not the service thread, to avoid racing
      // with the frames this loop already sends.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }

    const bool currently_enabled = IsEnabled(st);
    if (!currently_enabled && frozen_) {
      // A joint that dropped out stays limp until the hardware is
      // reactivated; re-enabling it here is what let a rebooted motor chase
      // a stale setpoint. The Stop frame keeps its feedback coming.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }
    if (!currently_enabled) {
      // Torque wanted but not confirmed: resend Enable, and run_mode with
      // it, since a brief power loss clears the motor's RAM. Rate-limited,
      // because this burst every cycle for every joint would exceed what
      // 1 Mbps CAN can carry when a whole rig recovers at once.
      if (now_ns - last_recovery_ns_[i] >= kRecoveryResendNs) {
        last_recovery_ns_[i] = now_ns;
        frames_by_bus[jh.can_interface].push_back(
          jh.motor->EncodeParamWriteU8(
            robstride_sdk::ParamIndex::RUN_MODE, static_cast<uint8_t>(jh.control_mode)));
        // Also RAM-resident: without re-arming, the motor rejoins with no
        // watchdog and would hold its last command on the next dropout.
        frames_by_bus[jh.can_interface].push_back(
          jh.motor->EncodeParamWriteU32(
            robstride_sdk::ParamIndex::CAN_TIMEOUT, CanTimeoutRaw(motor_can_timeout_ms_)));
        frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeEnable());
      }
      continue;
    }

    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    const bool stale = elapsed_ms < 0.0 || elapsed_ms > error_timeout_ms_;
    // fault_present is refreshed by every feedback frame and self-heals;
    // fault_bits stays latched with nothing to reset it, so it is not used.
    const bool faulted = st.fault_present.load(std::memory_order_relaxed);
    if (stale || faulted) {
      // Don't drive a motor we can't hear toward a possibly stale target.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }
    // Motion mode uses the combined Type-1 frame; the other modes are
    // driven by writing their own target parameter. While frozen the
    // controllers are not in charge, so hold the captured position.
    // Controllers speak the joint frame; MotorTargetPosition() maps back.
    const double target_position =
      MotorTargetPosition(i, frozen_ ? freeze_position_[i] : hw_cmd_position_[i]);
    const double target_velocity = jh.direction * (frozen_ ? 0.0 : hw_cmd_velocity_[i]);
    const double target_effort = jh.direction * (frozen_ ? 0.0 : hw_cmd_effort_[i]);

    can_frame frame{};
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::SPD_REF, static_cast<float>(target_velocity));
        break;
      case robstride_sdk::RunMode::RUN_MODE_POSITION_PP:
      case robstride_sdk::RunMode::RUN_MODE_POSITION_CSP:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::LOC_REF, static_cast<float>(target_position));
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::IQ_REF, static_cast<float>(target_effort));
        break;
      case robstride_sdk::RunMode::RUN_MODE_MOTION:
      case robstride_sdk::RunMode::RUN_MODE_SET_ZERO:
      default:
        frame = jh.motor->EncodeMotionCommand(
          static_cast<float>(target_position),
          static_cast<float>(target_velocity),
          static_cast<float>(hw_cmd_kp_[i]),
          static_cast<float>(hw_cmd_kd_[i]),
          static_cast<float>(target_effort));
        break;
    }
    frames_by_bus[jh.can_interface].push_back(frame);
  }

  for (auto & kv : frames_by_bus) {
    const int sent = buses_[kv.first]->SendFrames(kv.second);
    if (sent == static_cast<int>(kv.second.size())) {
      continue;
    }
    // Otherwise the joints just go stale and the logs blame the motors.
    const int err = buses_[kv.first]->last_send_errno();
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 1000,
      "CAN transmit incomplete on '" << kv.first << "': queued " << sent << " of " <<
        kv.second.size() << " frames (" << (err ? std::strerror(err) : "queue full") << "). " <<
        "The interface is not draining its transmit queue -- typically bus-off. Adapters "
        "that report 'Device doesn't support restart from Bus Off' cannot recover on their "
        "own: re-plug the adapter and re-run scripts/setup_can.sh.");
  }
  return return_type::OK;
}

double RobstrideHardware::WrapMotorPosition(size_t i, double motor_position)
{
  const JointHandle & jh = joints_[i];
  const double unwrapped = jh.direction * motor_position;
  const double wrapped = WrapToWindow(unwrapped, jh.wrap_center);
  wrap_offset_[i] = unwrapped - wrapped;
  return wrapped;
}

double RobstrideHardware::MotorTargetPosition(size_t i, double joint_position) const
{
  const JointHandle & jh = joints_[i];
  double delta = joint_position - hw_state_position_[i];
  if (jh.continuous) {
    // The same pose sits every 2*pi, so take the nearest one.
    delta = WrapToWindow(delta, 0.0);
  }
  return jh.direction * (hw_state_position_[i] + wrap_offset_[i] + delta);
}

JointHandle * RobstrideHardware::FindJointById(uint8_t id)
{
  for (auto & jh : joints_) {
    if (jh.id == id) {return &jh;}
  }
  return nullptr;
}

const hardware_interface::ComponentInfo * RobstrideHardware::FindGpio(
  const std::string & name) const
{
  for (const auto & gpio : info_.gpios) {
    if (gpio.name == name) {return &gpio;}
  }
  return nullptr;
}

void RobstrideHardware::GetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const robstride_sdk::MotorState & st = jh->motor->state();
  if (request->item_name == "position") {
    response->item_data = st.position.load();
  } else if (request->item_name == "velocity") {
    response->item_data = st.velocity.load();
  } else if (request->item_name == "torque" || request->item_name == "effort") {
    response->item_data = st.torque.load();
  } else if (request->item_name == "temperature") {
    response->item_data = st.temperature.load();
  } else if (request->item_name == "run_state") {
    response->item_data = static_cast<float>(st.run_state.load());
  } else if (request->item_name == "fault_bits") {
    response->item_data = static_cast<float>(st.fault_bits.load());
  } else {
    response->result = false;
    return;
  }
  response->result = true;
}

void RobstrideHardware::SetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const size_t idx = static_cast<size_t>(jh - joints_.data());

  // kp/kd are also gpio command interfaces; both paths land in the same
  // slot, which write() owns.
  if (request->item_name == "kp") {
    pending_kp_[idx].store(request->item_data, std::memory_order_relaxed);
    has_pending_kp_[idx].store(true, std::memory_order_relaxed);
    response->result = true;
    return;
  }
  if (request->item_name == "kd") {
    pending_kd_[idx].store(request->item_data, std::memory_order_relaxed);
    has_pending_kd_[idx].store(true, std::memory_order_relaxed);
    response->result = true;
    return;
  }

  // Otherwise treat item_name as a RobStride parameter index and write it.
  // Raw passthrough in the motor frame: gpio direction is not applied.
  try {
    const uint16_t index = static_cast<uint16_t>(std::stoul(request->item_name, nullptr, 0));
    const auto it = buses_.find(jh->can_interface);
    if (it == buses_.end()) {
      response->result = false;
      return;
    }
    response->result =
      it->second->SendFrame(jh->motor->EncodeParamWriteFloat(index, request->item_data));
  } catch (const std::exception &) {
    response->result = false;
  }
}

void RobstrideHardware::SetZeroCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const auto it = buses_.find(jh->can_interface);
  response->result = it != buses_.end() && it->second->SendFrame(jh->motor->EncodeSetZero());
  // The next feedback frame reports a relabeled raw position, so discard
  // the wraparound offset.
  jh->motor->ResetPositionTracking();
}

void RobstrideHardware::SetTorqueCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // Record the desired state only; write() sends the frames, so this does
  // not race with the RT loop. Deactivate any active controller before
  // disabling torque, or it keeps holding its setpoint (see README).
  torque_enabled_desired_.store(request->data, std::memory_order_relaxed);
  response->success = true;
}

void RobstrideHardware::RebootCallback(
  const std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }

  // Unconditional by design: whether a given fault is safe to clear is a
  // policy call for the caller, so this never gates on fault type.
  const auto it = buses_.find(jh->can_interface);
  // Not a literal MCU reboot: this is a Stop frame with the clear-fault
  // flag set. write() re-enables the motor on its own afterwards.
  response->result = it != buses_.end() && it->second->SendFrame(jh->motor->EncodeDisable(true));
}

}  // namespace robstride_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robstride_hardware_interface::RobstrideHardware, hardware_interface::SystemInterface)
