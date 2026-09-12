#pragma once

#include <fmt/core.h>
#include <libserial/SerialPort.h>

#include <chrono>
#include <feetech_driver/common.hpp>
#include <range/v3/all.hpp>
#include <string>

namespace feetech_driver {

class SerialPort {
 public:
  explicit SerialPort(const std::string& /*dev*/);
  ~SerialPort();
  Result configure(LibSerial::BaudRate baud_rate = LibSerial::BaudRate::BAUD_1000000);
  Result open();
  Result close();
  Result flashInputBuffer() noexcept;
  Result flashOutputBuffer() noexcept;

  // PickNik addition: an EEPROM write's acknowledgement (the servo commits to
  // flash before replying) can arrive after the 10 ms control-loop timeout
  // below; a caller writing an EEPROM register raises this around that one
  // call and restores it after, rather than raising it globally and risking a
  // lost SRAM (position) reply stalling the control loop.
  void set_timeout(const std::chrono::milliseconds timeout) noexcept { timeout_ = timeout; }
  [[nodiscard]] std::chrono::milliseconds timeout() const noexcept { return timeout_; }

  Result read_byte(uint8_t* byte) {
    try {
      port_.ReadByte(*byte, static_cast<std::size_t>(timeout_.count()));
    } catch (const LibSerial::ReadTimeout& e) {
      return tl::make_unexpected(fmt::format("SerialPort::read_byte [{}]", e.what()));
    }

    return {};
  }

  template <std::size_t N>
  Result read(std::array<uint8_t, N>* buffer) {
    return check_port().and_then([&]() -> Result {
      for (auto& byte : *buffer) {
        if (const auto result = read_byte(&byte); !result) {
          return tl::make_unexpected(fmt::format("SerialPort::read -> {}", result.error()));
        }
      }
      return {};
    });
  }

  template <std::size_t N>
  Result write(const std::array<uint8_t, N>& buffer) {
    return check_port().and_then([&]() -> Result {
      try {
        port_.Write(std::string(buffer.begin(), buffer.end()));
      } catch (const std::runtime_error& e) {
        return tl::make_unexpected(fmt::format("SerialPort::write [{}]", e.what()));
      }
      return {};
    });
  }

 private:
  [[nodiscard]] Result check_port() const noexcept;
  std::string dev_;
  std::chrono::milliseconds timeout_ = std::chrono::milliseconds(10);
  LibSerial::SerialPort port_;
};
}  // namespace feetech_driver
