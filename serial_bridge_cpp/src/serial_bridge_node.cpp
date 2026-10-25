// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// АННОТАЦИЯ
// Единый узел связи с ESP32 вместо трёх Python-узлов (twist_to_command,
// serial_bridge_node, feedback_processor). Читает UART в отдельном потоке,
// который спит в poll() до прихода данных, разбирает кадры $...# и сразу
// публикует /odom, TF odom->base_link, скорости колёс и PID-коэффициенты.
// Команды /cmd_vel и сырые строки из serial/cmd пишет в порт. При потере
// порта (переподключение USB) пытается открыть его заново.
//
// ANNOTATION
// Single ESP32 bridge node replacing three Python nodes. A reader thread
// sleeps in poll() until data arrives, parses $...# frames and publishes
// odometry, TF, wheel speeds and PID coefficients directly. /cmd_vel and raw
// strings from serial/cmd are written to the port. Reopens the port if the
// device disappears.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
// Galactic/Humble have only the .h header, newer distros deprecate it
#if __has_include("tf2_ros/transform_broadcaster.hpp")
#include "tf2_ros/transform_broadcaster.hpp"
#else
#include "tf2_ros/transform_broadcaster.h"
#endif

#include "serial_bridge_cpp/covariance.hpp"
#include "serial_bridge_cpp/frame_parser.hpp"
#include "serial_bridge_cpp/serial_port.hpp"

namespace serial_bridge_cpp
{

// Message types of the ESP32 protocol (first field of the frame)
constexpr int MSG_ODOMETRY = 1;   // $1;x;y;theta;v;w;v_left;v_right;#
constexpr int MSG_PID = 2;        // $2;KpL;KiL;KdL;KpR;KiR;KdR;#
constexpr std::size_t ODOMETRY_FIELDS = 8;  // type + 7 values
constexpr std::size_t PID_FIELDS = 7;       // type + 6 values

class SerialBridgeNode : public rclcpp::Node
{
public:
  explicit SerialBridgeNode(const rclcpp::NodeOptions & options)
  : Node("serial_bridge_node", options)
  {
    port_name_ = declare_parameter<std::string>("serial_port", "/dev/esp32");
    baudrate_ = static_cast<int>(declare_parameter<int64_t>("baudrate", 115200));
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // Raw frames are only needed by simple_PID_configurator (log window)
    publish_raw_ = declare_parameter<bool>("publish_raw_feedback", true);
    reconnect_period_ = std::chrono::duration<double>(
      declare_parameter<double>("reconnect_period", 1.0));
    // Diagonals of the odometry covariances (placeholders, see covariance.hpp)
    pose_covariance_ = diagonal_to_covariance(
      declare_parameter<std::vector<double>>(
        "pose_covariance_diagonal", default_pose_covariance_diagonal()),
      "pose_covariance_diagonal");
    twist_covariance_ = diagonal_to_covariance(
      declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal", default_twist_covariance_diagonal()),
      "twist_covariance_diagonal");

    // Topic names are the same as in the Python version
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pid_pub_ = create_publisher<std_msgs::msg::String>("/PIDcoeffs", 10);
    left_speed_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/sensors/wheel/left/speed", 10);
    right_speed_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/sensors/wheel/right/speed", 10);
    if (publish_raw_) {
      raw_pub_ = create_publisher<std_msgs::msg::String>("serial/feedback", 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_cmd_vel(*msg);});
    raw_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "serial/cmd", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {send(msg->data);});

    reader_ = std::thread([this]() {reader_loop();});

    RCLCPP_INFO(
      get_logger(), "Serial bridge started: %s @ %d, %s -> %s",
      port_name_.c_str(), baudrate_, frame_id_.c_str(), child_frame_id_.c_str());
  }

  ~SerialBridgeNode() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();  // poll() timeout is short, so join is fast
    }
    port_.close();
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    char buf[64];
    const int len = std::snprintf(
      buf, sizeof(buf), "$1;%.3f;%.3f;#", msg.linear.x, msg.angular.z);
    if (len > 0 && len < static_cast<int>(sizeof(buf))) {
      send(std::string(buf, static_cast<std::size_t>(len)));
    }
  }

  void send(const std::string & data)
  {
    if (!port_.write(data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot write to %s (port is not open)", port_name_.c_str());
    }
  }

  // ---------------------------------------------------------------- reader
  void reader_loop()
  {
    std::array<char, 512> buffer{};
    while (running_) {
      if (!port_.is_open() && !try_open()) {
        sleep_interruptible(reconnect_period_);
        continue;
      }

      const ssize_t n = port_.read(buffer.data(), buffer.size(), 100);
      if (n > 0) {
        parser_.feed(
          buffer.data(), static_cast<std::size_t>(n),
          [this](const std::string & frame) {handle_frame(frame);});
      } else if (n < 0) {
        RCLCPP_ERROR(
          get_logger(), "Serial port %s lost, reconnecting...", port_name_.c_str());
        port_.close();
        parser_.reset();
      }
      // n == 0: timeout, just check running_ and wait again
    }
  }

  bool try_open()
  {
    std::string error;
    if (port_.open(port_name_, baudrate_, error)) {
      RCLCPP_INFO(get_logger(), "Serial port %s opened", port_name_.c_str());
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Cannot open %s: %s",
      port_name_.c_str(), error.c_str());
    return false;
  }

  void sleep_interruptible(std::chrono::duration<double> duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (running_ && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // ---------------------------------------------------------------- frames
  void handle_frame(const std::string & frame)
  {
    if (raw_pub_) {
      std_msgs::msg::String raw;
      raw.data = frame;
      raw_pub_->publish(raw);
    }

    if (!parse_fields(frame, fields_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Bad frame: %s", frame.c_str());
      return;
    }

    const int type = static_cast<int>(fields_[0]);
    if (type == MSG_ODOMETRY && fields_.size() >= ODOMETRY_FIELDS) {
      publish_odometry(
        fields_[1], fields_[2], fields_[3], fields_[4], fields_[5]);
      publish_wheel_speeds(fields_[6], fields_[7]);
    } else if (type == MSG_PID && fields_.size() >= PID_FIELDS) {
      publish_pid(fields_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unexpected frame (type %d, %zu fields): %s",
        type, fields_.size(), frame.c_str());
    }
  }

  void publish_odometry(double x, double y, double theta, double vx, double wz)
  {
    const auto stamp = now();
    // Rotation around Z only: q = (0, 0, sin(theta/2), cos(theta/2))
    const double qz = std::sin(theta / 2.0);
    const double qw = std::cos(theta / 2.0);

    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = stamp;
    odom->header.frame_id = frame_id_;
    odom->child_frame_id = child_frame_id_;
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.orientation.z = qz;
    odom->pose.pose.orientation.w = qw;
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.angular.z = wz;
    odom->pose.covariance = pose_covariance_;
    odom->twist.covariance = twist_covariance_;
    odom_pub_->publish(std::move(odom));

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = frame_id_;
      tf.child_frame_id = child_frame_id_;
      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.rotation.z = qz;
      tf.transform.rotation.w = qw;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publish_wheel_speeds(double left, double right)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(left);
    left_speed_pub_->publish(msg);
    msg.data = static_cast<float>(right);
    right_speed_pub_->publish(msg);
  }

  void publish_pid(const std::vector<double> & f)
  {
    // Same format as the Python version: "KpL;KiL;KdL;KpR;KiR;KdR"
    char buf[160];
    std::snprintf(
      buf, sizeof(buf), "%g;%g;%g;%g;%g;%g", f[1], f[2], f[3], f[4], f[5], f[6]);
    std_msgs::msg::String msg;
    msg.data = buf;
    pid_pub_->publish(msg);
  }

  // Parameters
  std::string port_name_;
  int baudrate_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_;
  bool publish_raw_;
  std::chrono::duration<double> reconnect_period_;
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_cmd_sub_;

  // Serial
  SerialPort port_;
  FrameParser parser_;
  std::vector<double> fields_;  // reused between frames, no allocations
  std::atomic<bool> running_{true};
  std::thread reader_;  // declared last: started after everything above exists
};

}  // namespace serial_bridge_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(serial_bridge_cpp::SerialBridgeNode)
