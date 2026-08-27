#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "x4pro_protocol.hpp"

namespace ydlidar {

TEST(X4ProParserTest, ParseSinglePacket) {
  ScanPacketHeader hdr{};
  hdr.sync_a = SCAN_SYNC_A;
  hdr.sync_b = SCAN_SYNC_B;
  hdr.ct = 0x01; // New scan
  hdr.lsn = 3;
  // FSA = 0 deg, LSA = 30 deg
  hdr.fsa = static_cast<uint16_t>((0 * 64) << 1 | 1);
  hdr.lsa = static_cast<uint16_t>((30 * 64) << 1);

  // 3 sample points: 1.0m, 2.0m, 3.0m
  // In Q2: dist_m * 1000 / 0.25 = dist_m * 4000
  uint16_t samples[3] = {static_cast<uint16_t>(1000 << 2), static_cast<uint16_t>(2000 << 2),
                         static_cast<uint16_t>(3000 << 2)};

  uint16_t cs = static_cast<uint16_t>(hdr.sync_a) | (static_cast<uint16_t>(hdr.sync_b) << 8);
  cs ^= (static_cast<uint16_t>(hdr.ct) | (static_cast<uint16_t>(hdr.lsn) << 8));
  cs ^= hdr.fsa;
  cs ^= hdr.lsa;
  for (int i = 0; i < 3; ++i) {
    cs ^= samples[i];
  }
  hdr.cs = cs;

  EXPECT_TRUE(X4ProProtocol::validateChecksum(hdr, samples));

  // Verify converted distances
  EXPECT_NEAR(X4ProProtocol::sampleToDistM(samples[0]), 1.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::sampleToDistM(samples[1]), 2.0f, 1e-3f);
  EXPECT_NEAR(X4ProProtocol::sampleToDistM(samples[2]), 3.0f, 1e-3f);

  // Verify interpolated angles
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(hdr.fsa, hdr.lsa, 0, hdr.lsn), 0.0f, 1e-2f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(hdr.fsa, hdr.lsa, 1, hdr.lsn), 15.0f, 1e-2f);
  EXPECT_NEAR(X4ProProtocol::interpolateAngle(hdr.fsa, hdr.lsa, 2, hdr.lsn), 30.0f, 1e-2f);
}

} // namespace ydlidar
