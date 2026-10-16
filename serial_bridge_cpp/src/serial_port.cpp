// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

#include "serial_bridge_cpp/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace serial_bridge_cpp
{

namespace
{

bool to_speed(int baudrate, speed_t & speed)
{
  switch (baudrate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    case 460800: speed = B460800; return true;
    case 500000: speed = B500000; return true;
    case 921600: speed = B921600; return true;
    case 1000000: speed = B1000000; return true;
    default: return false;
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baudrate, std::string & error)
{
  close();

  speed_t speed;
  if (!to_speed(baudrate, speed)) {
    error = "unsupported baudrate " + std::to_string(baudrate);
    return false;
  }

  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0) {
    error = std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    error = std::string("tcgetattr: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&tty);                 // 8 data bits, no parity, no echo, no line editing
  tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem lines, enable receiver
  tty.c_cflag &= ~CSTOPB;          // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;         // no HW flow control
  tty.c_cc[VMIN] = 0;              // read() never blocks: waiting is done by poll()
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    error = std::string("tcsetattr: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);

  std::lock_guard<std::mutex> lock(mutex_);
  fd_ = fd;
  return true;
}

void SerialPort::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

ssize_t SerialPort::read(char * buffer, std::size_t size, int timeout_ms)
{
  // fd_ is only changed by open()/close(), which are called from the
  // same reader thread, so it is safe to use it here without the lock.
  const int fd = fd_;
  if (fd < 0) {
    return -1;
  }

  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;
  // The thread sleeps in the kernel here until data arrives or timeout.
  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret == 0) {
    return 0;
  }
  if (ret < 0) {
    return errno == EINTR ? 0 : -1;
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;  // e.g. USB cable unplugged
  }

  const ssize_t n = ::read(fd, buffer, size);
  if (n < 0) {
    return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
  }
  if (n == 0) {
    return -1;  // POLLIN but nothing to read: device is gone
  }
  return n;
}

bool SerialPort::write(const std::string & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    return false;
  }
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace serial_bridge_cpp
