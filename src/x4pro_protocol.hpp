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

// ---- X4 Pro 스캔 패킷 동기화 바이트 ----
// X4/X4Pro 스캔 데이터 패킷: [0xAA][0x55][CT][LSN][FSA×2][LSA×2][CS×2][SI×LSN×2]
static constexpr uint8_t SCAN_SYNC_A = 0xAA;
static constexpr uint8_t SCAN_SYNC_B = 0x55;

// 패킷 헤더 크기: sync(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2) = 10 bytes
static constexpr int SCAN_PKT_HEADER_SIZE = 10;
// 패킷당 최대 샘플 수
static constexpr int SCAN_PKT_MAX_SAMPLES = 40;

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

/// X4 Pro 스캔 패킷 헤더 (10 bytes)
/// 동기화: [0xAA][0x55]
/// CT  : bit0=1 → 새 회전의 첫 번째 패킷
/// LSN : 이 패킷에 포함된 샘플 수 (1~40)
/// FSA : 첫 샘플 각도 (Q6, bit0=1 항상)
/// LSA : 마지막 샘플 각도 (Q6)
/// CS  : 체크섬
struct ScanPacketHeader
{
    uint8_t  sync_a;  ///< 0xAA
    uint8_t  sync_b;  ///< 0x55
    uint8_t  ct;      ///< 패킷 타입 (bit0=1: 새 회전 시작)
    uint8_t  lsn;     ///< 샘플 수
    uint16_t fsa;     ///< 첫 샘플 각도 Q6 (bits15:1=angle×64, bit0=1)
    uint16_t lsa;     ///< 마지막 샘플 각도 Q6
    uint16_t cs;      ///< 체크섬
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

    /// 스캔 패킷에서 각도 보간 (각도[deg] 반환)
    /// fsa_raw: FSA 필드 원시값, lsa_raw: LSA 필드 원시값
    /// idx: 샘플 인덱스 (0~lsn-1), lsn: 총 샘플 수
    static float interpolateAngle(uint16_t fsa_raw, uint16_t lsa_raw, int idx, int lsn);

    /// 샘플 원시값 → 거리(m) 변환
    /// si: 16비트 샘플 data (dist_q2 형식, 0.25mm 단위)
    static float sampleToDistM(uint16_t si) { return static_cast<float>(si >> 2) / 1000.0f; }
};
