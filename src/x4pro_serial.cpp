#include "x4pro_serial.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace ydlidar {

namespace {

speed_t toBaudConst(int baud) noexcept {
  switch (baud) {
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
    default:
      return B0;
  }
}

} // namespace

X4ProSerial::~X4ProSerial() {
  close();
}

bool X4ProSerial::open(const std::string &port, int baudrate) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  if (!applyBaudrate(baudrate)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  flush();
  return true;
}

bool X4ProSerial::applyBaudrate(int baudrate) {
  struct termios tio;
  if (tcgetattr(fd_, &tio) < 0) {
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CREAD | CLOCAL);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  speed_t spd = toBaudConst(baudrate);
  if (spd != B0) {
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
  } else {
    // Non-standard baudrate via Linux termios2 + BOTHER
    static constexpr unsigned long kTCGETS2 = 0x802C542AUL;
    static constexpr unsigned long kTCSETS2 = 0x402C542BUL;

#ifndef BOTHER
#define BOTHER 0010000
#endif

    struct my_termios2 {
      tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
      cc_t c_line;
      cc_t c_cc[19];
      speed_t c_ispeed, c_ospeed;
    };
    static_assert(sizeof(my_termios2) == 44, "termios2 size mismatch");

    struct my_termios2 tio2;
    if (ioctl(fd_, kTCGETS2, &tio2) == 0) {
      tio2.c_cflag &= ~static_cast<tcflag_t>(CBAUD);
      tio2.c_cflag |= BOTHER;
      tio2.c_cflag |= (CREAD | CLOCAL);
      tio2.c_cflag &= ~CRTSCTS;
      tio2.c_iflag = 0;
      tio2.c_oflag = 0;
      tio2.c_lflag = 0;
      tio2.c_cc[VMIN] = 0;
      tio2.c_cc[VTIME] = 0;
      tio2.c_ispeed = static_cast<speed_t>(baudrate);
      tio2.c_ospeed = static_cast<speed_t>(baudrate);
      if (ioctl(fd_, kTCSETS2, &tio2) == 0) {
        return true;
      }
    }
    // Fallback: 115200
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
  }

  return tcsetattr(fd_, TCSANOW, &tio) == 0;
}

void X4ProSerial::close() {
  if (fd_ >= 0) {
    setDTR(false);
    ::close(fd_);
    fd_ = -1;
  }
}

int X4ProSerial::read(uint8_t *buf, int len, int timeout_ms) {
  if (fd_ < 0) {
    return -1;
  }

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (ret == 0) {
    return 0; // timeout
  }
  if (ret < 0) {
    if (errno == EINTR) {
      return 0;
    }
    return -1;
  }

  int n = ::read(fd_, buf, len);
  if (n > 0) {
    return n;
  }
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    return 0;
  }
  return -1;
}

int X4ProSerial::write(const uint8_t *buf, int len) {
  if (fd_ < 0) {
    return -1;
  }
  return ::write(fd_, buf, len);
}

void X4ProSerial::setDTR(bool on) {
  if (fd_ < 0) {
    return;
  }
  int status = 0;
  if (ioctl(fd_, TIOCMGET, &status) == 0) {
    if (on) {
      status |= TIOCM_DTR;
    } else {
      status &= ~TIOCM_DTR;
    }
    ioctl(fd_, TIOCMSET, &status);
  }
}

void X4ProSerial::flush() {
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

} // namespace ydlidar
