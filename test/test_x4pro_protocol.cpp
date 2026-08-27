#include <gtest/gtest.h>

#include <cmath>

#include "x4pro_protocol.hpp"

namespace ydlidar {

TEST(X4ProProtocolTest, BuildCommand) {
  uint8_t buf[2] = {0, 0};
  int len = X4ProProtocol::buildCmd(CMD_SCAN, buf);
  EXPECT_EQ(len, 2);
  EXPECT_EQ(buf[0], CMD_SYNC_BYTE);
  EXPECT_EQ(buf[1], CMD_SCAN);

  len = X4ProProtocol::buildCmd(CMD_STOP, buf);
  EXPECT_EQ(len, 2);
  EXPECT_EQ(buf[0], CMD_SYNC_BYTE);
  EXPECT_EQ(buf[1], CMD_STOP);
}

TEST(X4ProProtocolTest, ParseResponseHeaderValid) {
  uint8_t valid_buf[7] = {
      0xA5, 0x5A, 0x14, 0x00, 0x00, 0x40, // size=20, mode=1 (0x40000014)
      0x04                                // type=DEVINFO
  };

  ResponseHeader hdr{};
  EXPECT_TRUE(X4ProProtocol::parseResponseHeader(valid_buf, hdr));
  EXPECT_EQ(hdr.sync1, 0xA5);
  EXPECT_EQ(hdr.sync2, 0x5A);
  EXPECT_EQ(hdr.type, ANS_TYPE_DEVINFO);
  EXPECT_EQ(X4ProProtocol::getPayloadSize(hdr), 20u);
  EXPECT_EQ(X4ProProtocol::getResponseMode(hdr), 1u);
}

TEST(X4ProProtocolTest, ParseResponseHeaderInvalidSync) {
  uint8_t invalid_buf[7] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x04};
  ResponseHeader hdr{};
  EXPECT_FALSE(X4ProProtocol::parseResponseHeader(invalid_buf, hdr));
}

TEST(X4ProProtocolTest, SampleToDistConversion) {
  // Raw sample: bit[1:0] unused, bit[15:2] distance in 0.25mm units
  // If si = (1000 << 2) = 4000, distance = 1000 * 0.25mm = 250mm = 0.25m
  uint16_t sample_1000 = static_cast<uint16_t>(1000 << 2);
  EXPECT_NEAR(X4ProProtocol::sampleToDistM(sample_1000), 1.0f, 1e-4f);

  uint16_t sample_4000 = static_cast<uint16_t>(4000 << 2);
  EXPECT_NEAR(X4ProProtocol::sampleToDistM(sample_4000), 4.0f, 1e-4f);

  EXPECT_FLOAT_EQ(X4ProProtocol::sampleToDistM(0), 0.0f);
}

TEST(X4ProProtocolTest, InterpolateAngle) {
  // FSA: 0 degrees -> raw = (0 * 64) << 1 | 1 = 1
  // LSA: 30 degrees -> raw = (30 * 64) << 1 = 3840
  // LSN = 4 samples -> step = 30 / 3 = 10 degrees per index
  uint16_t fsa = (0 * 64) << 1 | 1;
  uint16_t lsa = (30 * 64) << 1;

  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 0, 4), 0.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 1, 4), 10.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 2, 4), 20.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 3, 4), 30.0f, 1e-3f);
}

TEST(X4ProProtocolTest, InterpolateAngleWrapAround) {
  // FSA: 350 degrees, LSA: 10 degrees -> cross 360 boundary
  uint16_t fsa = (350 * 64) << 1 | 1;
  uint16_t lsa = (10 * 64) << 1;

  // Total diff = 20 degrees, 3 samples -> indices: 0 (350), 1 (360), 2 (370)
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 0, 3), 350.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 1, 3), 360.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(fsa, lsa, 2, 3), 370.0f, 1e-3f);
}

TEST(X4ProProtocolTest, ChecksumValidation) {
  ScanPacketHeader hdr{};
  hdr.sync_a = SCAN_SYNC_A;
  hdr.sync_b = SCAN_SYNC_B;
  hdr.ct = 0x01;
  hdr.lsn = 2;
  hdr.fsa = 0x1234;
  hdr.lsa = 0x5678;

  uint16_t samples[2] = {0x0100, 0x0200};

  // Compute expected checksum
  uint16_t expected_cs =
      static_cast<uint16_t>(hdr.sync_a) | (static_cast<uint16_t>(hdr.sync_b) << 8);
  expected_cs ^= (static_cast<uint16_t>(hdr.ct) | (static_cast<uint16_t>(hdr.lsn) << 8));
  expected_cs ^= hdr.fsa;
  expected_cs ^= hdr.lsa;
  expected_cs ^= samples[0];
  expected_cs ^= samples[1];

  hdr.cs = expected_cs;
  EXPECT_TRUE(X4ProProtocol::validateChecksum(hdr, samples));

  // Corrupted checksum
  hdr.cs = expected_cs ^ 0x00FF;
  EXPECT_FALSE(X4ProProtocol::validateChecksum(hdr, samples));
}

TEST(X4ProProtocolTest, NormalizeAngleDeg) {
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(0.0f), 0.0f, 1e-4f);
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(180.0f), -180.0f, 1e-4f);
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(-180.0f), -180.0f, 1e-4f);
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(270.0f), -90.0f, 1e-4f);
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(-270.0f), 90.0f, 1e-4f);
  EXPECT_NEAR(X4ProProtocol::normalizeAngleDeg(720.0f), 0.0f, 1e-4f);
}

} // namespace ydlidar
