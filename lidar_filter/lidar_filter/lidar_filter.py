#!/usr/bin/env python3

# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Узел ROS 2 для фильтрации данных лазерного скана (LaserScan) по угловому
сектору. Конфигурируется параметрами: задает входной/выходной топики,
границы сектора в градусах и режим инверсии. Использует QoS с политикой
BEST_EFFORT для совместимости с лидарами.

Маска сектора считается один раз и пересчитывается только при изменении
геометрии скана (angle_min, angle_increment, число лучей). Если оставшиеся
лучи идут непрерывно, скан обрезается; если сектор разрывный (инверсия или
переход через ±π), выкинутые лучи внутри диапазона заменяются на NaN, чтобы
не сдвигать углы остальных лучей.

ANNOTATION
ROS 2 node for filtering laser scan (LaserScan) data by angular sector.
Configurable via parameters: sets input/output topics, sector boundaries
in degrees, and inversion mode. Uses QoS with BEST_EFFORT policy for
lidar compatibility.

The sector mask is computed once and recomputed only when the scan geometry
changes. A contiguous result is cropped; for a non-contiguous result the
removed beams inside the kept span are set to NaN, so angles of the other
beams stay correct.
'''

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    """Normalize angle to range [-pi, pi]."""
    return math.remainder(angle, 2.0 * math.pi)


def angle_in_sector(angle, start, end):
    """Check if (normalized) angle is within sector, considering cyclicity."""
    if start <= end:
        return start <= angle <= end
    # Sector crosses -pi/pi boundary
    return angle >= start or angle <= end


def compute_sector(num_readings, angle_min, angle_increment,
                   start, end, invert):
    """
    Compute what to keep from a scan with the given geometry.

    Returns None if nothing is kept, otherwise (first, last, gaps):
    first/last - indices of the first and last kept beam,
    gaps - indices (relative to first) of removed beams inside [first, last].
    """
    start = normalize_angle(start)
    end = normalize_angle(end)

    keep = []
    for i in range(num_readings):
        angle = normalize_angle(angle_min + i * angle_increment)
        keep.append(angle_in_sector(angle, start, end) != invert)

    if not any(keep):
        return None

    first = keep.index(True)
    last = num_readings - 1 - keep[::-1].index(True)
    gaps = [i - first for i in range(first, last + 1) if not keep[i]]
    return first, last, gaps


class LaserSectorFilter(Node):
    def __init__(self):
        super().__init__('laser_sector_filter')

        # Declare parameters: sector start and end in degrees
        self.declare_parameter('start_angle_deg', -135.0)  # sector start in degrees
        self.declare_parameter('end_angle_deg', 135.0)    # sector end in degrees
        self.declare_parameter('input_topic', '/scan')   # input topic
        self.declare_parameter('output_topic', '/scan_filtered')  # output topic
        self.declare_parameter('invert_sector', False)   # invert sector (keep everything except specified)
        self.declare_parameter('output_frame_id', '')   # frame_id for output topic

        # Get parameters
        self.start_angle_deg = self.get_parameter('start_angle_deg').value
        self.end_angle_deg = self.get_parameter('end_angle_deg').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.invert_sector = self.get_parameter('invert_sector').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        # Convert degrees to radians
        self.start_angle_rad = math.radians(self.start_angle_deg)
        self.end_angle_rad = math.radians(self.end_angle_deg)

        self.get_logger().info(
            f'Lidar sector filter initialized with parameters:'
        )
        self.get_logger().info(
            f'  Sector: {self.start_angle_deg}° to {self.end_angle_deg}°'
        )
        self.get_logger().info(f'  Input topic: {self.input_topic}')
        self.get_logger().info(f'  Output topic: {self.output_topic}')
        self.get_logger().info(f'  Invert sector: {self.invert_sector}')

        # Cache: scan geometry -> result of compute_sector()
        self._cache_key = None
        self._cache_value = None

        # Configure QoS for reliable data transmission with lidar
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            10
        )

    def _get_sector(self, msg):
        key = (len(msg.ranges), msg.angle_min, msg.angle_increment)
        if key != self._cache_key:
            self._cache_key = key
            self._cache_value = compute_sector(
                len(msg.ranges), msg.angle_min, msg.angle_increment,
                self.start_angle_rad, self.end_angle_rad, self.invert_sector
            )
        return self._cache_value

    def scan_callback(self, msg):
        sector = self._get_sector(msg)
        if sector is None:
            self.get_logger().warn(
                'Sector is empty! Check angle parameters.',
                throttle_duration_sec=5.0
            )
            return
        first, last, gaps = sector

        filtered_scan = LaserScan()

        if self.output_frame_id == '':
            filtered_scan.header.frame_id = msg.header.frame_id
        else:
            filtered_scan.header.frame_id = self.output_frame_id
        # Keep the original measurement time: TF lookups (AMCL, costmaps,
        # SLAM) must use the time when the scan was taken.
        filtered_scan.header.stamp = msg.header.stamp

        filtered_scan.angle_min = msg.angle_min + first * msg.angle_increment
        filtered_scan.angle_max = msg.angle_min + last * msg.angle_increment
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        ranges = msg.ranges[first:last + 1]
        for i in gaps:
            ranges[i] = math.nan
        filtered_scan.ranges = ranges

        if len(msg.intensities) > last:
            intensities = msg.intensities[first:last + 1]
            for i in gaps:
                intensities[i] = 0.0
            filtered_scan.intensities = intensities

        self.publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)

    node = LaserSectorFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log shutdown message BEFORE destroying node
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    except Exception as e:
        # Log error BEFORE destroying node
        node.get_logger().error(f'Unexpected error: {e}')

    # Clean shutdown
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
