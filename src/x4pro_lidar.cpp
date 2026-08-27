#include "x4pro_lidar.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

namespace ydlidar {

namespace {
constexpr int RESPONSE_HEADER_SIZE = 7;
constexpr size_t RX_BUFFER_SIZE = 4096;
} // namespace

X4ProLidar::~X4ProLidar() {
  disconnect();
}

bool X4ProLidar::connect(const std::string &port, int baudrate) {
  if (!serial_.open(port, baudrate)) {
    return false;
  }

  sendCmd(CMD_STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial_.flush();

  serial_.setDTR(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  dev_info_ok_ = getDeviceInfo();
  return true;
}

void X4ProLidar::disconnect() {
  stopScan();
  if (serial_.isOpen()) {
    sendCmd(CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    serial_.setDTR(false);
    serial_.close();
  }
}

bool X4ProLidar::startScan() {
  if (!serial_.isOpen() || scanning_.load()) {
    return false;
  }

  serial_.flush();
  if (!sendCmd(CMD_SCAN)) {
    return false;
  }

  ResponseHeader hdr{};
  if (waitResponseHeader(hdr, 1000) && hdr.type != ANS_TYPE_MEASUREMENT) {
    return false;
  }

  scanning_.store(true);
  scan_thread_ = std::thread([this]() { scanLoop(); });
  return true;
}

void X4ProLidar::stopScan() {
  if (!scanning_.exchange(false)) {
    return;
  }

  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }

  sendCmd(CMD_STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  serial_.flush();
}

bool X4ProLidar::sendCmd(uint8_t cmd) {
  uint8_t buf[2];
  int len = X4ProProtocol::buildCmd(cmd, buf);
  return serial_.write(buf, len) == len;
}

bool X4ProLidar::waitResponseHeader(ResponseHeader &hdr, int timeout_ms) {
  uint8_t buf[RESPONSE_HEADER_SIZE] = {};
  int received = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    if (received == 0) {
      uint8_t b;
      if (serial_.read(&b, 1, 50) == 1 && b == ANS_SYNC_BYTE1) {
        buf[received++] = b;
      }
    } else if (received == 1) {
      uint8_t b;
      if (serial_.read(&b, 1, 50) == 1) {
        if (b == ANS_SYNC_BYTE2) {
          buf[received++] = b;
        } else {
          received = 0;
        }
      }
    } else {
      int need = RESPONSE_HEADER_SIZE - received;
      int got = serial_.read(buf + received, need, 100);
      if (got > 0) {
        received += got;
      }
      if (received == RESPONSE_HEADER_SIZE) {
        return X4ProProtocol::parseResponseHeader(buf, hdr);
      }
    }
  }
  return false;
}

bool X4ProLidar::getDeviceInfo() {
  serial_.flush();
  if (!sendCmd(CMD_GET_EAI)) {
    return false;
  }

  ResponseHeader hdr{};
  if (!waitResponseHeader(hdr, 500) || hdr.type != ANS_TYPE_DEVINFO) {
    return false;
  }

  if (X4ProProtocol::getPayloadSize(hdr) < sizeof(DeviceInfo)) {
    return false;
  }

  uint8_t payload[sizeof(DeviceInfo)] = {};
  int received = 0;
  while (received < static_cast<int>(sizeof(DeviceInfo))) {
    int got = serial_.read(payload + received, sizeof(DeviceInfo) - received, 200);
    if (got <= 0) {
      return false;
    }
    received += got;
  }
  std::memcpy(&dev_info_, payload, sizeof(DeviceInfo));
  return true;
}

bool X4ProLidar::getHealthInfo() {
  serial_.flush();
  if (!sendCmd(CMD_GET_HEALTH)) {
    return false;
  }

  ResponseHeader hdr{};
  if (!waitResponseHeader(hdr, 500) || hdr.type != ANS_TYPE_HEALTH) {
    return false;
  }

  uint8_t payload[sizeof(HealthInfo)] = {};
  int received = 0;
  while (received < static_cast<int>(sizeof(HealthInfo))) {
    int got = serial_.read(payload + received, sizeof(HealthInfo) - received, 200);
    if (got <= 0) {
      return false;
    }
    received += got;
  }
  HealthInfo health{};
  std::memcpy(&health, payload, sizeof(HealthInfo));
  return health.status == 0;
}

void X4ProLidar::scanLoop() {
  std::string error_reason;
  bool hard_error = processScanData(error_reason);
  scanning_.store(false);
  if (hard_error) {
    if (serial_.isOpen()) {
      serial_.setDTR(false);
      serial_.close();
    }
    if (error_cb_) {
      error_cb_(error_reason);
    }
  }
}

bool X4ProLidar::processScanData(std::string &error_reason) {
  std::vector<uint8_t> rx_buf;
  rx_buf.reserve(RX_BUFFER_SIZE);

  LidarScan current_scan;
  current_scan.points.reserve(720);

  uint8_t chunk[512];

  while (scanning_.load()) {
    int n = serial_.read(chunk, sizeof(chunk), 50);
    if (n < 0) {
      error_reason = "serial read failure";
      return true;
    }
    if (n > 0) {
      rx_buf.insert(rx_buf.end(), chunk, chunk + n);
    }

    // Process all complete packets in buffer
    size_t offset = 0;
    while (rx_buf.size() >= offset + SCAN_PKT_HEADER_SIZE) {
      // Find sync words 0xAA 0x55
      if (rx_buf[offset] != SCAN_SYNC_A || rx_buf[offset + 1] != SCAN_SYNC_B) {
        ++offset;
        continue;
      }

      const uint8_t lsn = rx_buf[offset + 3];
      if (lsn == 0 || lsn > SCAN_PKT_MAX_SAMPLES) {
        offset += 2;
        continue;
      }

      const size_t total_pkt_len = SCAN_PKT_HEADER_SIZE + (static_cast<size_t>(lsn) * 2);
      if (rx_buf.size() < offset + total_pkt_len) {
        // Wait for remainder of packet
        break;
      }

      ScanPacketHeader hdr{};
      std::memcpy(&hdr, &rx_buf[offset], sizeof(ScanPacketHeader));

      const auto *sample_ptr =
          reinterpret_cast<const uint16_t *>(&rx_buf[offset + SCAN_PKT_HEADER_SIZE]);

      if (X4ProProtocol::validateChecksum(hdr, sample_ptr)) {
        const bool is_new_scan = (hdr.ct & 0x01) != 0;
        if (is_new_scan && !current_scan.points.empty()) {
          using namespace std::chrono;
          current_scan.stamp_ns = static_cast<uint64_t>(
              duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count());
          if (scan_cb_) {
            scan_cb_(current_scan);
          }
          current_scan.points.clear();
          current_scan.points.reserve(720);
        }

        for (int i = 0; i < static_cast<int>(lsn); ++i) {
          const uint16_t si = sample_ptr[i];
          if (si == 0) {
            continue;
          }

          float dist_m = X4ProProtocol::sampleToDistM(si);
          float angle = X4ProProtocol::interpolateAngle(hdr.fsa, hdr.lsa, i, lsn);

          // Reversion: coordinate transformation (360 - angle)
          angle = 360.0f - angle;
          if (angle >= 360.0f) {
            angle -= 360.0f;
          } else if (angle < 0.0f) {
            angle += 360.0f;
          }

          current_scan.points.push_back({angle, dist_m});
        }
      }

      offset += total_pkt_len;
    }

    if (offset > 0) {
      rx_buf.erase(rx_buf.begin(), rx_buf.begin() + offset);
    }
  }

  return false;
}

} // namespace ydlidar
