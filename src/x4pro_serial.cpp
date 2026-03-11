#include "x4pro_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>

// baudrate 숫자 → termios B* 상수 변환
static speed_t toBaudConst(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    // 128000 등 비표준 baudrate는 B0 반환 → BOTHER 방식으로 처리
    default:
        return B0;
    }
}

X4ProSerial::~X4ProSerial()
{
    close();
}

bool X4ProSerial::open(const std::string &port, int baudrate)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        return false;
    }

    // blocking 모드로 전환
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!applyBaudrate(baudrate))
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    flush();
    return true;
}

bool X4ProSerial::applyBaudrate(int baudrate)
{
    struct termios tio;
    if (tcgetattr(fd_, &tio) < 0)
        return false;

    cfmakeraw(&tio);
    tio.c_cflag |= (CREAD | CLOCAL);
    tio.c_cflag &= ~CRTSCTS; // 하드웨어 흐름 제어 비활성
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t spd = toBaudConst(baudrate);
    if (spd != B0)
    {
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);
    }
    else
    {
        // 128000처럼 표준 B* 상수가 없는 baudrate는 Linux termios2 + BOTHER 방식 사용
        //
        // glibc <termios.h>와 asm/termbits.h를 동시에 포함하면 struct 충돌 발생.
        // 따라서 ioctl 번호를 커널 ABI 값으로 직접 하드코딩한다.
        //
        //   sizeof(struct termios2) = 44 (0x2C) on arm64 & x86_64
        //   TCGETS2 = _IOR('T', 0x2A, termios2) = (2<<30)|(44<<16)|('T'<<8)|0x2A = 0x802C542A
        //   TCSETS2 = _IOW('T', 0x2B, termios2) = (1<<30)|(44<<16)|('T'<<8)|0x2B = 0x402C542B
        static constexpr unsigned long kTCGETS2 = 0x802C542AUL;
        static constexpr unsigned long kTCSETS2 = 0x402C542BUL;

#ifndef BOTHER
#define BOTHER 0010000  // 0x1000, 커스텀 baudrate 선택자
#endif

        // 커널 ABI와 동일한 레이아웃 (arm64/x86_64 모두 44 bytes)
        struct my_termios2 {
            tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
            cc_t     c_line;
            cc_t     c_cc[19];  // NCCS=19 (glibc termios의 32와 다름)
            speed_t  c_ispeed, c_ospeed;
        };
        static_assert(sizeof(my_termios2) == 44, "termios2 size mismatch");

        struct my_termios2 tio2;
        if (ioctl(fd_, kTCGETS2, &tio2) == 0)
        {
            tio2.c_cflag &= ~static_cast<tcflag_t>(CBAUD);
            tio2.c_cflag |= BOTHER;
            tio2.c_cflag |= (CREAD | CLOCAL);
            tio2.c_cflag &= ~CRTSCTS;
            tio2.c_iflag = 0;
            tio2.c_oflag = 0;
            tio2.c_lflag = 0;
            tio2.c_cc[VMIN]  = 0;
            tio2.c_cc[VTIME] = 0;
            tio2.c_ispeed = static_cast<speed_t>(baudrate);
            tio2.c_ospeed = static_cast<speed_t>(baudrate);
            if (ioctl(fd_, kTCSETS2, &tio2) == 0)
                return true;
        }
        // 폴백: 115200
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
    }

    return tcsetattr(fd_, TCSANOW, &tio) == 0;
}

void X4ProSerial::close()
{
    if (fd_ >= 0)
    {
        setDTR(false);
        ::close(fd_);
        fd_ = -1;
    }
}

int X4ProSerial::read(uint8_t *buf, int len, int timeout_ms)
{
    if (fd_ < 0)
        return -1;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0)
        return 0;

    return ::read(fd_, buf, len);
}

int X4ProSerial::write(const uint8_t *buf, int len)
{
    if (fd_ < 0)
        return -1;
    return ::write(fd_, buf, len);
}

void X4ProSerial::setDTR(bool on)
{
    if (fd_ < 0)
        return;
    int status;
    ioctl(fd_, TIOCMGET, &status);
    if (on)
        status |= TIOCM_DTR;
    else
        status &= ~TIOCM_DTR;
    ioctl(fd_, TIOCMSET, &status);
}

void X4ProSerial::flush()
{
    if (fd_ >= 0)
        tcflush(fd_, TCIOFLUSH);
}
