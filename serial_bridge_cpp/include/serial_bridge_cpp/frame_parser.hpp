// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Parsing of the ESP32 text protocol: frames "$type;v1;v2;...;#".
// Pure C++ without ROS, so it can be unit-tested separately.

#ifndef SERIAL_BRIDGE_CPP__FRAME_PARSER_HPP_
#define SERIAL_BRIDGE_CPP__FRAME_PARSER_HPP_

#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace serial_bridge_cpp
{

// Byte-stream state machine that cuts "$...#" frames out of the UART stream.
// Same logic as SerialReceiver in the ESP32 firmware:
//  - '$' always starts a new frame (an unfinished previous one is dropped);
//  - a frame longer than max_len is dropped as garbage.
// Bytes outside of frames are collected into text lines (Serial.println() of
// the firmware: errors, Wi-Fi / OTA messages). A line ends with '\n';
// the "\r\n" right after a frame belongs to the frame (so a text line
// interrupted by a frame stays one line); '\r' is ignored, other
// non-printable bytes become '?', empty lines are skipped and a line
// longer than max_len is emitted in parts.
class FrameParser
{
public:
  explicit FrameParser(std::size_t max_len = 256)
  : max_len_(max_len)
  {
    frame_.reserve(max_len_);
    text_.reserve(max_len_);
  }

  // Feed raw bytes. on_frame(const std::string &) is called for every
  // complete frame (including '$' and '#'), on_text(const std::string &) -
  // for every complete text line outside of frames (without "\r\n").
  template<typename FrameCallback, typename TextCallback>
  void feed(
    const char * data, std::size_t size, FrameCallback && on_frame,
    TextCallback && on_text)
  {
    for (std::size_t i = 0; i < size; ++i) {
      const char c = data[i];
      if (c == '$') {
        frame_.assign(1, '$');
        receiving_ = true;
      } else if (receiving_) {
        if (frame_.size() >= max_len_) {
          receiving_ = false;
          frame_.clear();
          continue;
        }
        frame_.push_back(c);
        if (c == '#') {
          receiving_ = false;
          after_frame_ = true;
          on_frame(frame_);
          frame_.clear();
        }
      } else if (after_frame_ && (c == '\r' || c == '\n')) {
        after_frame_ = (c == '\r');   // "\r\n" после кадра - часть кадра
      } else if (c == '\n') {
        flush_text(on_text);
      } else if (c != '\r') {
        after_frame_ = false;
        const auto u = static_cast<unsigned char>(c);
        text_.push_back((u >= 0x20 && u < 0x7f) ? c : (c == '\t' ? ' ' : '?'));
        if (text_.size() >= max_len_) {
          flush_text(on_text);
        }
      }
    }
  }

  // Frames only (text outside of frames is dropped)
  template<typename FrameCallback>
  void feed(const char * data, std::size_t size, FrameCallback && on_frame)
  {
    feed(data, size, std::forward<FrameCallback>(on_frame), [](const std::string &) {});
  }

  void reset()
  {
    receiving_ = false;
    frame_.clear();
    text_.clear();
    after_frame_ = false;
  }

private:
  template<typename TextCallback>
  void flush_text(TextCallback && on_text)
  {
    // trim spaces; skip empty lines (e.g. "\r\n" after every frame)
    const auto first = text_.find_first_not_of(' ');
    if (first != std::string::npos) {
      const auto last = text_.find_last_not_of(' ');
      on_text(text_.substr(first, last - first + 1));
    }
    text_.clear();
  }

  std::size_t max_len_;
  std::string frame_;
  std::string text_;
  bool receiving_ = false;
  bool after_frame_ = false;
};

// Split a complete frame "$1;0.1;-2.5;#" into numbers {1, 0.1, -2.5}.
// Empty fields (e.g. after the trailing ';') are skipped.
// Returns false if the frame is malformed or a field is not a number.
inline bool parse_fields(const std::string & frame, std::vector<double> & out)
{
  out.clear();
  if (frame.size() < 2 || frame.front() != '$' || frame.back() != '#') {
    return false;
  }
  const char * p = frame.c_str() + 1;              // after '$'
  const char * end = frame.c_str() + frame.size() - 1;  // at '#'
  while (p < end) {
    const char * sep = p;
    while (sep < end && *sep != ';') {
      ++sep;
    }
    if (sep > p) {
      char * num_end = nullptr;
      errno = 0;
      const double value = std::strtod(p, &num_end);
      // the whole field must be a number: strtod has to stop exactly at ';'/'#'
      if (num_end != sep || errno == ERANGE) {
        return false;
      }
      out.push_back(value);
    }
    p = sep + 1;
  }
  return !out.empty();
}

}  // namespace serial_bridge_cpp

#endif  // SERIAL_BRIDGE_CPP__FRAME_PARSER_HPP_
