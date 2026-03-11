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

bool X4ProProtocol::decodeScanNode(const ScanNode &node, LidarPoint &pt)
{
    // 거리가 0이면 무효 포인트
    if (node.dist_q2 == 0)
    {
        return false;
    }

    // 각도 디코딩: angle_q6 >> 1 (bit[0]은 start 플래그), 단위: 1/64 도
    float angle_deg = static_cast<float>(node.angle_q6 >> 1) / 64.0f;

    // X4 Pro reversion=true → 360 - angle
    // inverted=true → 그대로 사용 (이미 reversion에서 처리)
    angle_deg = 360.0f - angle_deg;
    if (angle_deg >= 360.0f)
        angle_deg -= 360.0f;

    // 거리 디코딩: dist_q2 / 4.0, 단위: mm → m
    float dist_m = static_cast<float>(node.dist_q2) / 4.0f / 1000.0f;

    pt.angle_deg = angle_deg;
    pt.dist_m = dist_m;
    return true;
}
