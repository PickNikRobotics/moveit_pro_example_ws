// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef FAKE_ROBSTRIDE_MOTOR_HPP_
#define FAKE_ROBSTRIDE_MOTOR_HPP_

#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "robstride_sdk/robstride_protocol.hpp"

/// A minimal stand-in for one RobStride motor on a SocketCAN interface.
///
/// It answers every addressed command with the Type-2 feedback frame the real
/// motor sends, which is all the hardware component needs to get through
/// activation: a position to preload targets from, and a run_state that
/// reports MOTOR once Enable has been seen. It records the communication type
/// of every frame it receives so a test can assert what was (or was not) sent.
class FakeRobstrideMotor
{
public:
  FakeRobstrideMotor(const std::string & interface, uint8_t motor_id, uint8_t master_id)
  : motor_id_(motor_id), master_id_(master_id)
  {
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
      return;
    }
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(socket_);
      socket_ = -1;
      return;
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(socket_);
      socket_ = -1;
      return;
    }
    timeval timeout{};
    timeout.tv_usec = 20000;
    ::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    running_ = true;
    thread_ = std::thread(&FakeRobstrideMotor::Run, this);
  }

  ~FakeRobstrideMotor()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  bool ok() const {return socket_ >= 0;}

  /// Communication types received so far, in order.
  std::vector<uint8_t> received() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }

  /// Position the fake reports, in rad.
  void set_position(double position) {position_ = position;}

private:
  void Run()
  {
    while (running_) {
      can_frame frame{};
      const ssize_t bytes = ::read(socket_, &frame, sizeof(frame));
      if (bytes != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }
      if (!(frame.can_id & CAN_EFF_FLAG)) {
        continue;
      }
      const auto parsed = robstride_sdk::ParseExtendedId(frame.can_id & CAN_EFF_MASK);
      if (parsed.dest_id != motor_id_) {
        continue;  // Not addressed to this motor, and never our own replies.
      }
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        received_.push_back(parsed.comm_type);
      }
      if (parsed.comm_type == robstride_sdk::COMM_MOTOR_ENABLE) {
        enabled_ = true;
      } else if (parsed.comm_type == robstride_sdk::COMM_MOTOR_STOP) {
        enabled_ = false;
      }
      SendFeedback();
    }
  }

  void SendFeedback()
  {
    // RS00 ranges; the test's joint uses the same model.
    const auto & limits =
      robstride_sdk::GetActuatorLimitsTable().at(robstride_sdk::ActuatorType::RS00);
    const uint8_t mode = enabled_ ?
      static_cast<uint8_t>(robstride_sdk::RunState::MOTOR) :
      static_cast<uint8_t>(robstride_sdk::RunState::RESET);
    // data_area_2 layout for a feedback frame: motor id in bits 0-7, the six
    // fault flags in bits 8-13, run_state in bits 14-15.
    const uint16_t data_area_2 =
      static_cast<uint16_t>(motor_id_) | static_cast<uint16_t>(mode) << 14;

    can_frame frame{};
    frame.can_id = robstride_sdk::BuildExtendedId(
      robstride_sdk::COMM_MOTOR_FEEDBACK, data_area_2, master_id_) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    const uint16_t position = robstride_sdk::FloatToUint16(
      static_cast<float>(position_.load()),
      static_cast<float>(-limits.position_max), static_cast<float>(limits.position_max));
    const uint16_t zero_velocity = robstride_sdk::FloatToUint16(
      0.0f, static_cast<float>(-limits.velocity_max),
      static_cast<float>(limits.velocity_max));
    const uint16_t zero_torque = robstride_sdk::FloatToUint16(
      0.0f, static_cast<float>(-limits.torque_max), static_cast<float>(limits.torque_max));
    frame.data[0] = static_cast<uint8_t>(position >> 8);
    frame.data[1] = static_cast<uint8_t>(position & 0xFF);
    frame.data[2] = static_cast<uint8_t>(zero_velocity >> 8);
    frame.data[3] = static_cast<uint8_t>(zero_velocity & 0xFF);
    frame.data[4] = static_cast<uint8_t>(zero_torque >> 8);
    frame.data[5] = static_cast<uint8_t>(zero_torque & 0xFF);
    frame.data[6] = 0;
    frame.data[7] = 250;  // 25.0 C
    [[maybe_unused]] const ssize_t written = ::write(socket_, &frame, sizeof(frame));
  }

  const uint8_t motor_id_;
  const uint8_t master_id_;
  int socket_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<double> position_{0.0};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<uint8_t> received_;
};

#endif  // FAKE_ROBSTRIDE_MOTOR_HPP_
