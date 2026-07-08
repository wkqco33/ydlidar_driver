#pragma once

#include "x4pro_serial.hpp"
#include "x4pro_protocol.hpp"

#include <string>
#include <atomic>
#include <functional>

/// X4 Pro 라이다 제어 클래스
/// 시리얼 포트를 직접 열고, SDK 없이 프로토콜을 직접 구현합니다.
class X4ProLidar
{
public:
    using ScanCallback = std::function<void(const LidarScan &)>;
    // Called from the scan thread when a hard serial error is detected
    // (port unplugged/died) - by the time this fires, disconnect() has
    // already been called internally so isConnected()/isScanning() are
    // both false; the argument is a human-readable reason.
    using ErrorCallback = std::function<void(const std::string &)>;

    X4ProLidar() = default;
    ~X4ProLidar();

    /// 포트 연결 및 장치 정보 조회
    bool connect(const std::string &port, int baudrate = 128000);

    /// 연결 해제
    void disconnect();

    /// 스캔 시작 (비동기: 내부 루프 스레드 시작)
    bool startScan();

    /// 스캔 중지
    void stopScan();

    bool isConnected() const { return serial_.isOpen(); }
    bool isScanning()  const { return scanning_; }
    bool isDeviceInfoOk() const { return dev_info_ok_; }

    /// 스캔 완료 시 호출될 콜백 등록
    void setScanCallback(ScanCallback cb) { scan_cb_ = std::move(cb); }

    /// 스캔 중 하드 에러(포트 소실 등) 발생 시 호출될 콜백 등록
    void setErrorCallback(ErrorCallback cb) { error_cb_ = std::move(cb); }

    const DeviceInfo &deviceInfo() const { return dev_info_; }

private:
    X4ProSerial serial_;
    DeviceInfo dev_info_{};
    bool dev_info_ok_{false};
    ScanCallback scan_cb_;
    ErrorCallback error_cb_;
    std::atomic<bool> scanning_{false};

    // ---- 내부 헬퍼 ----
    bool sendCmd(uint8_t cmd);
    bool waitResponseHeader(ResponseHeader &hdr, int timeout_ms = 500);
    bool getDeviceInfo();
    bool getHealthInfo();

    /// 스캔 수신 루프 (startScan 내에서 호출)
    void scanLoop();

    /// 수신 버퍼에서 스캔 노드를 읽고 1회전이 완료되면 콜백 호출.
    /// 하드 에러(포트 소실)로 중단된 경우 true를 반환하고 error_reason에 사유를 채운다.
    bool processScanData(std::string &error_reason);
};
