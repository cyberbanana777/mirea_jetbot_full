// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Minimal POSIX serial port (termios + poll), no external dependencies.

#ifndef SERIAL_BRIDGE_CPP__SERIAL_PORT_HPP_
#define SERIAL_BRIDGE_CPP__SERIAL_PORT_HPP_

#include <cstddef>
#include <mutex>
#include <string>
#include <sys/types.h>

namespace serial_bridge_cpp
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open and configure the port as raw 8N1. On error returns false and
  // fills `error` with a description.
  bool open(const std::string & device, int baudrate, std::string & error);
  void close();
  bool is_open() const;

  // Wait up to timeout_ms for data and read what is available.
  // Returns >0 - bytes read, 0 - timeout, -1 - error / device disconnected.
  // Must be called from a single (reader) thread.
  ssize_t read(char * buffer, std::size_t size, int timeout_ms);

  // Write the whole string. Thread-safe. Returns false on error.
  bool write(const std::string & data);

private:
  int fd_ = -1;
  mutable std::mutex mutex_;  // protects fd_ against close() during write()
};

}  // namespace serial_bridge_cpp

#endif  // SERIAL_BRIDGE_CPP__SERIAL_PORT_HPP_
