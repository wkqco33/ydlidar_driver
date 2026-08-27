#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "x4pro_protocol.hpp"
#include "x4pro_serial.hpp"

namespace ydlidar {

/// @brief High-level controller for YDLidar X4 Pro device
class X4ProLidar {
public:
  using ScanCallback = std::function<void(const LidarScan &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  X4ProLidar() = default;
  ~X4ProLidar();

  X4ProLidar(const X4ProLidar &) = delete;
  X4ProLidar &operator=(const X4ProLidar &) = delete;

  /// @brief Connect to serial port and initialize LiDAR
  bool connect(const std::string &port, int baudrate = 128000);

  /// @brief Stop scan and disconnect serial port
  void disconnect();

  /// @brief Start scanning thread
  bool startScan();

  /// @brief Stop scanning
  void stopScan();

  [[nodiscard]] bool isConnected() const noexcept {
    return serial_.isOpen();
  }
  [[nodiscard]] bool isScanning() const noexcept {
    return scanning_.load();
  }
  [[nodiscard]] bool isDeviceInfoOk() const noexcept {
    return dev_info_ok_;
  }

  /// @brief Register scan completion callback
  void setScanCallback(ScanCallback cb) {
    scan_cb_ = std::move(cb);
  }

  /// @brief Register hardware error callback
  void setErrorCallback(ErrorCallback cb) {
    error_cb_ = std::move(cb);
  }

  [[nodiscard]] const DeviceInfo &deviceInfo() const noexcept {
    return dev_info_;
  }

private:
  X4ProSerial serial_;
  DeviceInfo dev_info_{};
  bool dev_info_ok_{false};
  ScanCallback scan_cb_;
  ErrorCallback error_cb_;
  std::atomic<bool> scanning_{false};
  std::thread scan_thread_;

  bool sendCmd(uint8_t cmd);
  bool waitResponseHeader(ResponseHeader &hdr, int timeout_ms = 500);
  bool getDeviceInfo();
  bool getHealthInfo();
  void scanLoop();
  bool processScanData(std::string &error_reason);
};

} // namespace ydlidar
