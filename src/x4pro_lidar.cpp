#include "x4pro_lidar.hpp"

#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>

// 응답 헤더 크기 = 7 bytes
static constexpr int RESPONSE_HEADER_SIZE = 7;

X4ProLidar::~X4ProLidar()
{
    stopScan();
    disconnect();
}

bool X4ProLidar::connect(const std::string &port, int baudrate)
{
    if (!serial_.open(port, baudrate))
    {
        return false;
    }

    // 혹시 이전에 스캔 중이었으면 정지
    sendCmd(LIDAR_CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_.flush();

    // DTR 올려서 모터 활성화 (X4 Pro: support_motor_dtr=true)
    serial_.setDTR(true);
    // FTDI/CH340 USB-Serial 어댑터가 안정화될 때까지 충분히 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 장치 정보 조회 — X4 Pro SingleChannel 모드에서 응답이 없거나
    // 느릴 수 있으므로 실패해도 연결을 끊지 않고 경고만 기록
    if (!getDeviceInfo()) {
        // 경고 수준: 스캔 자체는 정상 동작 가능
        dev_info_ok_ = false;
    } else {
        dev_info_ok_ = true;
    }

    return true;
}

void X4ProLidar::disconnect()
{
    stopScan();
    if (serial_.isOpen())
    {
        sendCmd(LIDAR_CMD_STOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        serial_.setDTR(false);
        serial_.close();
    }
}

bool X4ProLidar::startScan()
{
    if (!serial_.isOpen())
        return false;
    if (scanning_)
        return true;

    serial_.flush();
    if (!sendCmd(LIDAR_CMD_SCAN))
        return false;

    // X4 Pro SingleChannel: 스캔 응답 헤더를 기다리되,
    // 타임아웃이 나도 데이터가 흘러오면 스캔을 시작할 수 있음
    ResponseHeader hdr{};
    bool hdr_ok = waitResponseHeader(hdr, 1000);
    if (hdr_ok) {
        // 헤더가 왔는데 타입이 다르면 진짜 오류
        if (hdr.type != LIDAR_ANS_TYPE_MEASUREMENT) {
            return false;
        }
    }
    // 헤더 타임아웃이어도 스캔 루프 진입 시도
    // (SingleChannel 디바이스는 헤더 없이 바로 스캔 데이터를 보내기도 함)

    scanning_ = true;
    std::thread([this]()
                { scanLoop(); })
        .detach();
    return true;
}

void X4ProLidar::stopScan()
{
    if (!scanning_)
        return;
    scanning_ = false;
    sendCmd(LIDAR_CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    serial_.flush();
}

// ---- private ----

bool X4ProLidar::sendCmd(uint8_t cmd)
{
    uint8_t buf[2];
    int len = X4ProProtocol::buildCmd(cmd, buf);
    return serial_.write(buf, len) == len;
}

bool X4ProLidar::waitResponseHeader(ResponseHeader &hdr, int timeout_ms)
{
    uint8_t buf[RESPONSE_HEADER_SIZE] = {};
    int received = 0;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    // 0xA5 0x5A 동기화
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (received == 0)
        {
            uint8_t b;
            if (serial_.read(&b, 1, 50) == 1 && b == LIDAR_ANS_SYNC_BYTE1)
            {
                buf[received++] = b;
            }
        }
        else if (received == 1)
        {
            uint8_t b;
            if (serial_.read(&b, 1, 50) == 1)
            {
                if (b == LIDAR_ANS_SYNC_BYTE2)
                {
                    buf[received++] = b;
                }
                else
                {
                    received = 0;
                }
            }
        }
        else
        {
            // 나머지 5바이트 읽기
            int need = RESPONSE_HEADER_SIZE - received;
            int got = serial_.read(buf + received, need, 100);
            if (got > 0)
                received += got;
            if (received == RESPONSE_HEADER_SIZE)
            {
                return X4ProProtocol::parseResponseHeader(buf, hdr);
            }
        }
    }
    return false;
}

bool X4ProLidar::getDeviceInfo()
{
    serial_.flush();
    if (!sendCmd(LIDAR_CMD_GET_EAI))
        return false;

    ResponseHeader hdr{};
    if (!waitResponseHeader(hdr, 500))
        return false;
    if (hdr.type != LIDAR_ANS_TYPE_DEVINFO)
        return false;

    uint32_t size = X4ProProtocol::getPayloadSize(hdr);
    if (size < sizeof(DeviceInfo))
        return false;

    uint8_t payload[sizeof(DeviceInfo)] = {};
    int received = 0;
    while (received < static_cast<int>(sizeof(DeviceInfo)))
    {
        int got = serial_.read(payload + received,
                               sizeof(DeviceInfo) - received, 200);
        if (got <= 0)
            return false;
        received += got;
    }
    memcpy(&dev_info_, payload, sizeof(DeviceInfo));
    return true;
}

bool X4ProLidar::getHealthInfo()
{
    serial_.flush();
    if (!sendCmd(LIDAR_CMD_GET_HEALTH))
        return false;

    ResponseHeader hdr{};
    if (!waitResponseHeader(hdr, 500))
        return false;
    if (hdr.type != LIDAR_ANS_TYPE_HEALTH)
        return false;

    uint8_t payload[sizeof(HealthInfo)] = {};
    int received = 0;
    while (received < static_cast<int>(sizeof(HealthInfo)))
    {
        int got = serial_.read(payload + received,
                               sizeof(HealthInfo) - received, 200);
        if (got <= 0)
            return false;
        received += got;
    }
    HealthInfo health{};
    memcpy(&health, payload, sizeof(HealthInfo));
    return health.status == 0;
}

void X4ProLidar::scanLoop()
{
    std::string error_reason;
    bool hard_error = processScanData(error_reason);
    scanning_ = false;
    if (hard_error)
    {
        // disconnect() before invoking the callback so the callback's own
        // isConnected()/isScanning() checks (e.g. deciding whether to
        // reconnect) see accurate state.
        disconnect();
        if (error_cb_) error_cb_(error_reason);
    }
}

/// X4 Pro 스캔 패킷 수신 루프
///
/// 실제 패킷 포맷 (X4/X4Pro, SingleChannel=true):
///   [0xAA][0x55][CT:1][LSN:1][FSA:2][LSA:2][CS:2][SI×LSN×2]
///   총 헤더 10 bytes + LSN×2 bytes payload
///
/// CT  bit0=1 → 이 패킷이 새 회전의 시작
/// FSA/LSA bit[15:1]=angle_q6, bit[0]=플래그
/// SI[i] = dist_q2 (0.25mm 단위, bit[15:2]=거리, bit[1:0]=미사용)
bool X4ProLidar::processScanData(std::string &error_reason)
{
    LidarScan current_scan;
    uint8_t byte_in;

    while (scanning_)
    {
        // ── Step 1: 0xAA 0x55 동기화 ─────────────────────────────────
        int n = serial_.read(&byte_in, 1, 50);
        if (n < 0) { error_reason = "serial read error (sync byte 1)"; return true; }
        if (n != 1) continue;
        if (byte_in != SCAN_SYNC_A) continue;
        n = serial_.read(&byte_in, 1, 50);
        if (n < 0) { error_reason = "serial read error (sync byte 2)"; return true; }
        if (n != 1) continue;
        if (byte_in != SCAN_SYNC_B) continue;

        // ── Step 2: 나머지 헤더 8 bytes (CT, LSN, FSA×2, LSA×2, CS×2) ──
        uint8_t hdr[8];
        int rcv = 0;
        while (rcv < 8 && scanning_) {
            int got = serial_.read(hdr + rcv, 8 - rcv, 50);
            if (got < 0) { error_reason = "serial read error (header)"; return true; }
            if (got > 0) rcv += got;
        }
        if (!scanning_) break;

        const uint8_t  ct  = hdr[0];
        const uint8_t  lsn = hdr[1];
        const uint16_t fsa = static_cast<uint16_t>(hdr[2]) |
                             (static_cast<uint16_t>(hdr[3]) << 8);
        const uint16_t lsa = static_cast<uint16_t>(hdr[4]) |
                             (static_cast<uint16_t>(hdr[5]) << 8);

        // LSN 범위 검사 (sanity check)
        if (lsn == 0 || lsn > SCAN_PKT_MAX_SAMPLES) continue;

        // ── Step 3: LSN×2 bytes 샘플 데이터 수신 ────────────────────────
        uint16_t samples[SCAN_PKT_MAX_SAMPLES];
        auto * raw = reinterpret_cast<uint8_t *>(samples);
        const int data_bytes = lsn * 2;
        rcv = 0;
        while (rcv < data_bytes && scanning_) {
            int got = serial_.read(raw + rcv, data_bytes - rcv, 50);
            if (got < 0) { error_reason = "serial read error (sample data)"; return true; }
            if (got > 0) rcv += got;
        }
        if (!scanning_) break;

        // ── Step 4: 새 회전 감지 → 이전 스캔 콜백 ───────────────────────
        const bool is_new_scan = (ct & 0x01) != 0;
        if (is_new_scan && !current_scan.points.empty()) {
            using namespace std::chrono;
            current_scan.stamp_ns = static_cast<uint64_t>(
                duration_cast<nanoseconds>(
                    system_clock::now().time_since_epoch()).count());
            if (scan_cb_) scan_cb_(current_scan);
            current_scan.points.clear();
        }

        // ── Step 5: 포인트 디코딩 (각도 보간 + 거리 변환) ──────────────
        for (int i = 0; i < static_cast<int>(lsn); i++) {
            const uint16_t si = samples[i];
            if (si == 0) continue;  // 무효 포인트

            float dist_m = X4ProProtocol::sampleToDistM(si);

            // 각도 보간: FSA~LSA 사이를 lsn 등분
            float angle = X4ProProtocol::interpolateAngle(fsa, lsa, i, lsn);

            // X4 Pro reversion=true: 회전 방향 반전
            angle = 360.0f - angle;
            if (angle >= 360.0f) angle -= 360.0f;
            if (angle <    0.0f) angle += 360.0f;

            current_scan.points.push_back({angle, dist_m});
        }
    }
    return false;
}
