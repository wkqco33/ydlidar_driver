#pragma once

#include <cstdint>
#include <vector>

/// ========================================================
/// YDLIDAR X4/X4Pro 통신 프로토콜 상수 및 구조체 정의
/// ========================================================

// ---- 커맨드 헤더 ----
static constexpr uint8_t LIDAR_CMD_SYNC_BYTE = 0xA5;
static constexpr uint8_t LIDAR_ANS_SYNC_BYTE1 = 0xA5;
static constexpr uint8_t LIDAR_ANS_SYNC_BYTE2 = 0x5A;

// ---- 커맨드 코드 ----
static constexpr uint8_t LIDAR_CMD_STOP = 0x65;
static constexpr uint8_t LIDAR_CMD_SCAN = 0x60;
static constexpr uint8_t LIDAR_CMD_FORCE_SCAN = 0x61;
static constexpr uint8_t LIDAR_CMD_RESET = 0x80;
static constexpr uint8_t LIDAR_CMD_GET_EAI = 0x55;    // 장치 정보
static constexpr uint8_t LIDAR_CMD_GET_HEALTH = 0x92; // 상태 정보

// ---- 응답 타입 코드 ----
static constexpr uint8_t LIDAR_ANS_TYPE_DEVINFO = 0x04;
static constexpr uint8_t LIDAR_ANS_TYPE_HEALTH = 0x06;
static constexpr uint8_t LIDAR_ANS_TYPE_MEASUREMENT = 0x81;

// ---- 응답 모드 ----
static constexpr uint8_t LIDAR_ANS_MODE_SINGLE = 0x00;
static constexpr uint8_t LIDAR_ANS_MODE_CONTINUOUS = 0x01;

// ---- 스캔 패킷 플래그 ----
static constexpr uint8_t LIDAR_RESP_SCAN_SYNC_FLAG = 0x01; // 새 스캔 시작
static constexpr uint8_t LIDAR_RESP_SCAN_START_FLAG = 0x01;

// 스캔 패킷 헤더 크기 (sync_quality 1 + angle_q6 2 + dist_q2 2 = 최소 5바이트)
static constexpr int SCAN_PACKET_HEADER_SIZE = 5;
// 하나의 패킷에 담기는 최대 포인트 수 (X4: 단일 채널, 패킷당 1 노드)
// SingleChannel 모드에서는 패킷 1개 = 포인트 1개
static constexpr int SCAN_NODE_SIZE = 5; // 1 + 2 + 2 bytes

#pragma pack(push, 1)

/// 커맨드 패킷 (헤더 + 커맨드 코드)
struct CmdPacket
{
    uint8_t sync; ///< 0xA5
    uint8_t cmd;
};

/// 응답 헤더 (7 bytes)
struct ResponseHeader
{
    uint8_t sync1;          ///< 0xA5
    uint8_t sync2;          ///< 0x5A
    uint32_t size_and_mode; ///< [29:0]=size, [31:30]=mode
    uint8_t type;
};

/// 장치 정보 응답 페이로드
struct DeviceInfo
{
    uint8_t model;
    uint16_t firmware_version;
    uint8_t hardware_version;
    uint8_t serialnum[16];
};

/// 상태 정보 응답 페이로드
struct HealthInfo
{
    uint8_t status; ///< 0=OK, 1=Warning, 2=Error
    uint16_t error_code;
};

/// SingleChannel 스캔 노드 (X4 Pro)
/// sync_quality: bit[0] = sync 플래그, bit[1]=0(질 무시)
/// angle_q6:     각도 [degree] × 64 (고정소수점), bit[0]=start 플래그
/// dist_q2:      거리 [mm] × 4
struct ScanNode
{
    uint8_t sync_quality;
    uint16_t angle_q6;
    uint16_t dist_q2;
};

#pragma pack(pop)

/// 파서가 돌려주는 라이다 포인트 (극좌표)
struct LidarPoint
{
    float angle_deg; ///< 도 단위 [-180, 180]
    float dist_m;    ///< 미터 단위
};

/// 스캔 1회전 결과
struct LidarScan
{
    std::vector<LidarPoint> points;
    uint64_t stamp_ns{0}; ///< 수신 완료 시각 (nanoseconds)
};

/// ========================================================
/// 프로토콜 레벨 파서
/// ========================================================
class X4ProProtocol
{
public:
    /// 커맨드 패킷을 buf에 직렬화. 반환값: 직렬화 바이트 수.
    static int buildCmd(uint8_t cmd, uint8_t *buf);

    /// 응답 헤더 파싱. buf는 최소 7 bytes.
    static bool parseResponseHeader(const uint8_t *buf, ResponseHeader &hdr);

    /// 응답 payload 크기 추출
    static uint32_t getPayloadSize(const ResponseHeader &hdr);

    /// 응답 연속/단일 모드 추출
    static uint8_t getResponseMode(const ResponseHeader &hdr);

    /// 스캔 노드 디코딩 (ScanNode → LidarPoint)
    static bool decodeScanNode(const ScanNode &node, LidarPoint &pt);
};
