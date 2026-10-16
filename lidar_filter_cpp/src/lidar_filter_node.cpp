// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// АННОТАЦИЯ
// Узел ROS 2 для фильтрации LaserScan по угловому сектору (C++-версия
// lidar_filter с теми же параметрами). Маска сектора считается один раз и
// пересчитывается только при изменении геометрии скана, поэтому обработка
// скана - это копирование куска массива. Время скана (header.stamp)
// сохраняется исходным. Если сектор разрывный (инверсия или переход через
// ±π), выкинутые лучи внутри диапазона заменяются на NaN, чтобы углы
// остальных лучей не сдвигались.
//
// ANNOTATION
// ROS 2 node filtering LaserScan by angular sector (C++ version of
// lidar_filter, same parameters). The sector mask is cached per scan
// geometry; the original header.stamp is kept; for non-contiguous sectors
// removed beams inside the kept span become NaN.

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "lidar_filter_cpp/sector.hpp"

namespace lidar_filter_cpp
{

class LaserSectorFilter : public rclcpp::Node
{
public:
  explicit LaserSectorFilter(const rclcpp::NodeOptions & options)
  : Node("laser_sector_filter", options)
  {
    const double start_deg = declare_parameter<double>("start_angle_deg", -135.0);
    const double end_deg = declare_parameter<double>("end_angle_deg", 135.0);
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan_filtered");
    invert_ = declare_parameter<bool>("invert_sector", false);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

    start_rad_ = start_deg * M_PI / 180.0;
    end_rad_ = end_deg * M_PI / 180.0;

    RCLCPP_INFO(
      get_logger(), "Lidar sector filter: %.1f deg .. %.1f deg%s, %s -> %s",
      start_deg, end_deg, invert_ ? " (inverted)" : "",
      input_topic.c_str(), output_topic.c_str());

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, 10);
    // BEST_EFFORT, as in the Python version (lidar driver compatibility)
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::QoS(10).best_effort(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(*msg);});
  }

private:
  const std::optional<Sector> & sector_for(const sensor_msgs::msg::LaserScan & msg)
  {
    const std::size_t n = msg.ranges.size();
    if (!cache_valid_ || n != cache_n_ || msg.angle_min != cache_angle_min_ ||
      msg.angle_increment != cache_increment_)
    {
      cache_valid_ = true;
      cache_n_ = n;
      cache_angle_min_ = msg.angle_min;
      cache_increment_ = msg.angle_increment;
      sector_ = compute_sector(n, msg.angle_min, msg.angle_increment, start_rad_, end_rad_, invert_);
    }
    return sector_;
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    const auto & sector = sector_for(msg);
    if (!sector) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Sector is empty! Check angle parameters.");
      return;
    }
    const std::size_t first = sector->first;
    const std::size_t last = sector->last;

    auto out = std::make_unique<sensor_msgs::msg::LaserScan>();
    out->header.frame_id = output_frame_id_.empty() ? msg.header.frame_id : output_frame_id_;
    // Keep the measurement time: TF lookups must use the time of the scan
    out->header.stamp = msg.header.stamp;
    out->angle_min = msg.angle_min + static_cast<float>(first) * msg.angle_increment;
    out->angle_max = msg.angle_min + static_cast<float>(last) * msg.angle_increment;
    out->angle_increment = msg.angle_increment;
    out->time_increment = msg.time_increment;
    out->scan_time = msg.scan_time;
    out->range_min = msg.range_min;
    out->range_max = msg.range_max;

    out->ranges.assign(msg.ranges.begin() + first, msg.ranges.begin() + last + 1);
    for (const std::size_t i : sector->gaps) {
      out->ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }

    if (msg.intensities.size() > last) {
      out->intensities.assign(
        msg.intensities.begin() + first, msg.intensities.begin() + last + 1);
      for (const std::size_t i : sector->gaps) {
        out->intensities[i] = 0.0F;
      }
    }

    pub_->publish(std::move(out));
  }

  double start_rad_;
  double end_rad_;
  bool invert_;
  std::string output_frame_id_;

  // Cache of the sector for the last seen scan geometry
  bool cache_valid_ = false;
  std::size_t cache_n_ = 0;
  float cache_angle_min_ = 0.0F;
  float cache_increment_ = 0.0F;
  std::optional<Sector> sector_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  // namespace lidar_filter_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_filter_cpp::LaserSectorFilter)
