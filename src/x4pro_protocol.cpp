#include "x4pro_protocol.hpp"

#include <cmath>
#include <cstring>

int X4ProProtocol::buildCmd(uint8_t cmd, uint8_t *buf)
{
    buf[0] = LIDAR_CMD_SYNC_BYTE;
    buf[1] = cmd;
    return 2;
}

bool X4ProProtocol::parseResponseHeader(const uint8_t *buf, ResponseHeader &hdr)
{
    if (buf[0] != LIDAR_ANS_SYNC_BYTE1 || buf[1] != LIDAR_ANS_SYNC_BYTE2)
    {
        return false;
    }
    hdr.sync1 = buf[0];
    hdr.sync2 = buf[1];
    memcpy(&hdr.size_and_mode, buf + 2, 4);
    hdr.type = buf[6];
    return true;
}

uint32_t X4ProProtocol::getPayloadSize(const ResponseHeader &hdr)
{
    // [29:0] = size
    return hdr.size_and_mode & 0x3FFFFFFF;
}

uint8_t X4ProProtocol::getResponseMode(const ResponseHeader &hdr)
{
    // [31:30] = mode
    return (hdr.size_and_mode >> 30) & 0x03;
}

float X4ProProtocol::interpolateAngle(
    uint16_t fsa_raw, uint16_t lsa_raw, int idx, int lsn)
{
    // bit[0]은 플래그 — 제거 후 Q6 → 도 변환
    const float fsa_deg = static_cast<float>(fsa_raw >> 1) / 64.0f;
    const float lsa_deg = static_cast<float>(lsa_raw >> 1) / 64.0f;

    float diff = lsa_deg - fsa_deg;
    if (diff < 0.0f) diff += 360.0f;

    const float step = (lsn > 1) ? diff / static_cast<float>(lsn - 1) : 0.0f;
    return fsa_deg + step * static_cast<float>(idx);
}
