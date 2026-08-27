#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ydlidar {

/// @brief Raw POSIX serial port wrapper with DTR motor control and custom baudrate support
class X4ProSerial {
public:
  X4ProSerial() = default;
  ~X4ProSerial();

  X4ProSerial(const X4ProSerial &) = delete;
  X4ProSerial &operator=(const X4ProSerial &) = delete;

  /// @brief Open serial port and set baudrate (default: 128000)
  bool open(const std::string &port, int baudrate = 128000);

  /// @brief Close serial port and deactivate DTR
  void close();

  /// @brief Check if port is open
  [[nodiscard]] bool isOpen() const noexcept {
    return fd_ >= 0;
  }

  /// @brief Read up to len bytes from port with timeout.
  /// @return >0: bytes read, 0: timeout (no data), -1: hard I/O error
  int read(uint8_t *buf, int len, int timeout_ms = 100);

  /// @brief Write bytes to serial port.
  /// @return Bytes written or -1 on error.
  int write(const uint8_t *buf, int len);

  /// @brief Control DTR line (used for LiDAR motor on/off)
  void setDTR(bool on);

  /// @brief Flush serial RX/TX buffers
  void flush();

private:
  int fd_{-1};

  bool applyBaudrate(int baudrate);
};

} // namespace ydlidar
