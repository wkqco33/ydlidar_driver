#pragma once

#include <string>
#include <cstdint>
#include <cstddef>

/// UART 시리얼 포트 래퍼
/// - termios 기반 raw 모드
/// - DTR 핀으로 모터 ON/OFF (X4 Pro는 support_motor_dtr = true)
class X4ProSerial
{
public:
    X4ProSerial() = default;
    ~X4ProSerial();

    /// 포트 오픈 및 baudrate 설정 (기본 128000)
    bool open(const std::string &port, int baudrate = 128000);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    /// 최대 len 바이트 읽기.
    /// 반환값 > 0: 수신된 바이트 수
    /// 반환값 == 0: 타임아웃 (정상 - 아직 데이터 없음)
    /// 반환값 == -1: 하드 에러 (포트가 사라짐/닫힘 등 - 재연결 필요)
    int read(uint8_t *buf, int len, int timeout_ms = 100);

    /// len 바이트 쓰기. 송신 바이트 수 반환.
    int write(const uint8_t *buf, int len);

    /// DTR 라인 설정으로 모터 제어
    void setDTR(bool on);

    /// 수신 버퍼 비우기
    void flush();

private:
    int fd_{-1};

    bool applyBaudrate(int baudrate);
};
