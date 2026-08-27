#include "x4pro_protocol.hpp"

#include <cmath>
#include <cstring>

namespace ydlidar {

int X4ProProtocol::buildCmd(uint8_t cmd, uint8_t *buf) noexcept {
  buf[0] = CMD_SYNC_BYTE;
  buf[1] = cmd;
  return 2;
}

bool X4ProProtocol::parseResponseHeader(const uint8_t *buf, ResponseHeader &hdr) noexcept {
  if (buf[0] != ANS_SYNC_BYTE1 || buf[1] != ANS_SYNC_BYTE2) {
    return false;
  }
  hdr.sync1 = buf[0];
  hdr.sync2 = buf[1];
  std::memcpy(&hdr.size_and_mode, buf + 2, sizeof(hdr.size_and_mode));
  hdr.type = buf[6];
  return true;
}

uint32_t X4ProProtocol::getPayloadSize(const ResponseHeader &hdr) noexcept {
  return hdr.size_and_mode & 0x3FFFFFFFU;
}

uint8_t X4ProProtocol::getResponseMode(const ResponseHeader &hdr) noexcept {
  return static_cast<uint8_t>((hdr.size_and_mode >> 30) & 0x03U);
}

float X4ProProtocol::interpolateAngle(uint16_t fsa_raw, uint16_t lsa_raw, int idx,
                                      int lsn) noexcept {
  const float fsa_deg = static_cast<float>(fsa_raw >> 1) * (1.0f / 64.0f);
  const float lsa_deg = static_cast<float>(lsa_raw >> 1) * (1.0f / 64.0f);

  float diff = lsa_deg - fsa_deg;
  if (diff < 0.0f) {
    diff += 360.0f;
  }

  const float step = (lsn > 1) ? (diff / static_cast<float>(lsn - 1)) : 0.0f;
  return fsa_deg + step * static_cast<float>(idx);
}

bool X4ProProtocol::validateChecksum(const ScanPacketHeader &hdr,
                                     const uint16_t *samples) noexcept {
  uint16_t checksum = static_cast<uint16_t>(hdr.sync_a) | (static_cast<uint16_t>(hdr.sync_b) << 8);
  checksum ^= (static_cast<uint16_t>(hdr.ct) | (static_cast<uint16_t>(hdr.lsn) << 8));
  checksum ^= hdr.fsa;
  checksum ^= hdr.lsa;

  for (int i = 0; i < hdr.lsn; ++i) {
    checksum ^= samples[i];
  }

  return checksum == hdr.cs;
}

float X4ProProtocol::normalizeAngleDeg(float deg) noexcept {
  deg = std::fmod(deg + 180.0f, 360.0f);
  if (deg < 0.0f) {
    deg += 360.0f;
  }
  return deg - 180.0f;
}

} // namespace ydlidar
