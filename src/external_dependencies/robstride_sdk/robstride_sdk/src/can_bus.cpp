// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "robstride_sdk/can_bus.hpp"

#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace robstride_sdk
{

namespace
{
constexpr int kMaxBatch = 64;
constexpr int kSocketBufferBytes = 1 << 20;  // 1 MiB send/recv buffers
}  // namespace

CanBus::CanBus(std::string ifname)
: ifname_(std::move(ifname))
{
}

CanBus::~CanBus()
{
  Stop();
  Close();
}

bool CanBus::Open()
{
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    Close();
    return false;
  }

  const int enable_canfd = 0;  // classic CAN 2.0, 8-byte frames per protocol spec
  setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

  const int sndbuf = kSocketBufferBytes;
  const int rcvbuf = kSocketBufferBytes;
  setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
  setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    Close();
    return false;
  }

  const int flags = fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    Close();
    return false;
  }

  epoll_fd_ = epoll_create1(0);
  if (epoll_fd_ < 0) {
    Close();
    return false;
  }
  struct epoll_event ev {};
  ev.events = EPOLLIN;
  ev.data.fd = fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd_, &ev) < 0) {
    Close();
    return false;
  }

  return true;
}

void CanBus::Close()
{
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void CanBus::RegisterMotor(RobstrideMotor * motor)
{
  motors_[motor->motor_id()] = motor;
}

void CanBus::Start()
{
  if (running_.exchange(true)) {
    return;
  }
  read_thread_ = std::thread(&CanBus::ReadThreadMain, this);
}

void CanBus::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

int CanBus::SendFrames(const std::vector<can_frame> & frames)
{
  if (frames.empty()) {
    return 0;
  }

  std::vector<struct iovec> iovecs(frames.size());
  std::vector<struct mmsghdr> msgs(frames.size());
  for (size_t i = 0; i < frames.size(); ++i) {
    iovecs[i].iov_base = const_cast<can_frame *>(&frames[i]);
    iovecs[i].iov_len = sizeof(can_frame);
    std::memset(&msgs[i], 0, sizeof(msgs[i]));
    msgs[i].msg_hdr.msg_iov = &iovecs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
  }

  int sent_total = 0;
  int send_errno = 0;
  unsigned int offset = 0;
  while (offset < msgs.size()) {
    const int n = sendmmsg(fd_, msgs.data() + offset, msgs.size() - offset, 0);
    if (n < 0) {
      // Keep the reason so callers can tell a wedged bus from silent motors.
      send_errno = errno;
      break;
    }
    sent_total += n;
    offset += static_cast<unsigned int>(n);
    if (n == 0) {
      break;
    }
  }
  last_send_errno_.store(send_errno, std::memory_order_relaxed);
  return sent_total;
}

bool CanBus::SendFrame(const can_frame & frame)
{
  return SendFrames({frame}) == 1;
}

void CanBus::DispatchFrame(const can_frame & frame)
{
  if (!(frame.can_id & CAN_EFF_FLAG)) {
    return;
  }
  const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  const ParsedId parsed = ParseExtendedId(can_id);

  uint8_t source_motor_id;
  if (parsed.comm_type == COMM_MOTOR_FEEDBACK) {
    source_motor_id = static_cast<uint8_t>(parsed.data_area_2 & 0xFF);
  } else if (parsed.comm_type == COMM_FAULT_FEEDBACK) {
    source_motor_id = static_cast<uint8_t>((parsed.data_area_2 >> 8) & 0xFF);
  } else {
    return;
  }

  const auto it = motors_.find(source_motor_id);
  if (it == motors_.end()) {
    return;
  }
  it->second->HandleFrame(parsed.comm_type, parsed.data_area_2, frame.data, frame.can_dlc);
}

void CanBus::ReadThreadMain()
{
  std::vector<can_frame> frame_buf(kMaxBatch);
  std::vector<struct iovec> iovecs(kMaxBatch);
  std::vector<struct mmsghdr> msgs(kMaxBatch);
  for (int i = 0; i < kMaxBatch; ++i) {
    iovecs[i].iov_base = &frame_buf[i];
    iovecs[i].iov_len = sizeof(can_frame);
    std::memset(&msgs[i], 0, sizeof(msgs[i]));
    msgs[i].msg_hdr.msg_iov = &iovecs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
  }

  struct epoll_event events[4];
  while (running_.load(std::memory_order_relaxed)) {
    const int n_events = epoll_wait(epoll_fd_, events, 4, 200 /* ms, allows Stop() to be noticed */);
    if (n_events <= 0) {
      continue;
    }
    while (true) {
      const int n = recvmmsg(fd_, msgs.data(), kMaxBatch, MSG_DONTWAIT, nullptr);
      if (n <= 0) {
        break;
      }
      for (int i = 0; i < n; ++i) {
        DispatchFrame(frame_buf[i]);
      }
      if (n < kMaxBatch) {
        break;
      }
    }
  }
}

}  // namespace robstride_sdk
