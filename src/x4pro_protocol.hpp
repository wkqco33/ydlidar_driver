#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

/// @brief YDLIDAR protocol constants
namespace ydlidar {

constexpr uint8_t CMD_SYNC_BYTE = 0xA5;
constexpr uint8_t ANS_SYNC_BYTE1 = 0xA5;
constexpr uint8_t ANS_SYNC_BYTE2 = 0x5A;

constexpr uint8_t CMD_STOP = 0x65;
constexpr uint8_t CMD_SCAN = 0x60;
constexpr uint8_t CMD_FORCE_SCAN = 0x61;
constexpr uint8_t CMD_RESET = 0x80;
constexpr uint8_t CMD_GET_EAI = 0x55;
constexpr uint8_t CMD_GET_HEALTH = 0x92;

constexpr uint8_t ANS_TYPE_DEVINFO = 0x04;
constexpr uint8_t ANS_TYPE_HEALTH = 0x06;
constexpr uint8_t ANS_TYPE_MEASUREMENT = 0x81;

constexpr uint8_t ANS_MODE_SINGLE = 0x00;
constexpr uint8_t ANS_MODE_CONTINUOUS = 0x01;

constexpr uint8_t SCAN_SYNC_A = 0xAA;
constexpr uint8_t SCAN_SYNC_B = 0x55;

constexpr int SCAN_PKT_HEADER_SIZE = 10;
constexpr int SCAN_PKT_MAX_SAMPLES = 40;

#pragma pack(push, 1)

/// @brief Command packet structure (Header + Command)
struct CmdPacket {
  uint8_t sync; // 0xA5
  uint8_t cmd;
};

/// @brief 7-byte response header
struct ResponseHeader {
  uint8_t sync1;          // 0xA5
  uint8_t sync2;          // 0x5A
  uint32_t size_and_mode; // [29:0]=size, [31:30]=mode
  uint8_t type;
};

/// @brief Device info response payload
struct DeviceInfo {
  uint8_t model;
  uint16_t firmware_version;
  uint8_t hardware_version;
  uint8_t serialnum[16];
};

/// @brief Device health response payload
struct HealthInfo {
  uint8_t status; // 0=OK, 1=Warning, 2=Error
  uint16_t error_code;
};

/// @brief 10-byte scan packet header
struct ScanPacketHeader {
  uint8_t sync_a; // 0xAA
  uint8_t sync_b; // 0x55
  uint8_t ct;     // Packet type (bit0=1: new scan cycle)
  uint8_t lsn;    // Sample count (1..40)
  uint16_t fsa;   // First sample angle Q6 (bit0=1)
  uint16_t lsa;   // Last sample angle Q6
  uint16_t cs;    // Checksum
};

#pragma pack(pop)

/// @brief 2D Polar LiDAR measurement point
struct LidarPoint {
  float angle_deg; // Degrees in [-180.0, 180.0]
  float dist_m;    // Distance in meters
};

/// @brief A complete 360-degree LiDAR scan
struct LidarScan {
  std::vector<LidarPoint> points;
  uint64_t stamp_ns{0};
};

/// @brief Protocol serializer and parser helper functions
class X4ProProtocol {
public:
  /// @brief Serialize a 2-byte command packet.
  /// @return Number of serialized bytes (2).
  static int buildCmd(uint8_t cmd, uint8_t *buf) noexcept;

  /// @brief Parse response header from 7-byte buffer.
  static bool parseResponseHeader(const uint8_t *buf, ResponseHeader &hdr) noexcept;

  /// @brief Extract payload size from response header.
  static uint32_t getPayloadSize(const ResponseHeader &hdr) noexcept;

  /// @brief Extract continuous/single mode from response header.
  static uint8_t getResponseMode(const ResponseHeader &hdr) noexcept;

  /// @brief Calculate interpolated angle in degrees for sample at index `idx`.
  static float interpolateAngle(uint16_t fsa_raw, uint16_t lsa_raw, int idx, int lsn) noexcept;

  /// @brief Convert raw 16-bit sample value (0.25mm per unit) to meters.
  static inline float sampleToDistM(uint16_t si) noexcept {
    return static_cast<float>(si >> 2) * 0.001f;
  }

  /// @brief Validate XOR checksum of a scan packet.
  static bool validateChecksum(const ScanPacketHeader &hdr, const uint16_t *samples) noexcept;

  /// @brief Normalize angle to [-180.0, 180.0] range.
  static float normalizeAngleDeg(float deg) noexcept;
};

} // namespace ydlidar
