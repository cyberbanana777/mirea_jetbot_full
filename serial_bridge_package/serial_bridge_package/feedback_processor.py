#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS2-узел для обработки сырых данных с ESP32 в формате $data1;...;data13# с
публикацией в темы одометрии, IMU, температуры, напряжения, нагрузки и позиции
колес. Преобразует углы в кватернионы и публикует преобразования систем
координат через TF. Ожидает строго форматированные сообщения с 13 числовыми
полями без обработки ошибок связи.

ANNOTATION
ROS2 node for processing raw ESP32 data in $data1;...;data13# format with
publishing to odometry, IMU, temperature, voltage, load and wheel position
topics. Converts angles to quaternions and broadcasts coordinate frame
transformations via TF. Requires strictly formatted 13-field numeric messages
without communication error handling.
'''

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

NUMS_OF_FIELDS = 8

class FeedbackProcessor(Node):
    """
    ROS2 node for processing feedback data from ESP32 and publishing to 
    various topics.
    """

    def __init__(self):
        super().__init__('feedback_processor')
        
        # Subscription to raw data from ESP32
        self.subscription = self.create_subscription(
            String,
            '/esp32_feedback',  # Topic where ESP32 data is published
            self.feedback_callback,
            10
        )
        
        # Publishers for various data types
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.PID_coeff_pub = self.create_publisher(String, '/PIDcoeffs', 10)
        self.left_speed_pub = self.create_publisher(Float32, '/sensors/wheel/left/speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, '/sensors/wheel/right/speed', 10)

        # TF broadcaster for coordinate system transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters for coordinate frames
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        self.get_logger().info(
            'Feedback processor node started. Waiting for ESP32 data...'
            )

    def feedback_callback(self, msg):
        """
        Process raw message from ESP32 and publish to appropriate topics.
        """
        try:
            raw_data = msg.data.strip()
            
            # Validate message format
            if not raw_data.startswith('$') or not raw_data.endswith('#'):
                self.get_logger().warning(
                    f'Invalid message format: {raw_data}'
                    )
                return
            
            # Remove starting $ and ending #
            content = raw_data[1:-1]
            
            # Split data by semicolons
            parts = content.split(';')
            msg_type = float(parts[0])
            
            if len(parts) >= NUMS_OF_FIELDS:  # Check if all fields are present
                if msg_type == 1:    
                    # Parse data
                    x_position = float(parts[1])
                    y_position = float(parts[2])
                    omega_angle = float(parts[3])
                    x_real_linear_velocity = float(parts[4])
                    z_real_angular_velocity = float(parts[5])
                    left_wheel_speed = float(parts[6])
                    right_wheel_speed = float(parts[7])

                    # Publish data to appropriate topics
                    self.publish_odometry(
                        x_position, y_position, omega_angle,
                        x_real_linear_velocity, z_real_angular_velocity
                    )

                    self.publish_wheels_speed(
                        left_wheel_speed,
                        right_wheel_speed
                    )
                    
                    self.get_logger().debug(
                        f'Processed ESP32 message: position ({x_position:.2f}, {y_position:.2f})'
                    )
                
                elif msg_type == 2:
                    # Parse data
                    Kp_L = float(parts[1])
                    Ki_L = float(parts[2])
                    Kd_L = float(parts[3])
                    Kp_R = float(parts[4])
                    Ki_R = float(parts[5])
                    Kd_R = float(parts[6])
                    
                    # Publish PID coefficients
                    self.publish_PID_coefficients(Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R)
                    
                else:
                    self.get_logger().debug(
                        f'Invalid msg_type = {msg_type})'
                    )
                   
                
            else:
                self.get_logger().warning(
                    f'Insufficient data in message. Expected {NUMS_OF_FIELDS} fields, got {len(parts)}'
                )
                
        except (ValueError, IndexError) as e:
            self.get_logger().warning(
                f'Error parsing message: {msg.data}. Error: {e}'
                )

    def publish_PID_coefficients(self, Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R):
        """Publish PID coefficients."""
        pid_msg = String()
        pid_msg.data = f'{Kp_L};{Ki_L};{Kd_L};{Kp_R};{Ki_R};{Kd_R}'
        self.PID_coeff_pub.publish(pid_msg)

    def publish_wheels_speed(self, left_wheel_speed, right_wheel_speed):
        """Publish wheel speed data."""
        
        left_speed_msg = Float32()
        left_speed_msg.data = left_wheel_speed
        self.left_speed_pub.publish(left_speed_msg)
        
        right_speed_msg = Float32()
        right_speed_msg.data = right_wheel_speed
        self.right_speed_pub.publish(right_speed_msg)

    def publish_odometry(self, x, y, theta, vx, vth):
        """Publish odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        
        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert angle to quaternion)
        q = self.angle_to_quaternion(theta)
        odom_msg.pose.pose.orientation = q
        
        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transformation
        self.publish_tf_transform(x, y, theta)

    def publish_tf_transform(self, x, y, theta):
        """Publish transformation between odom and base_link frames."""
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        q = self.angle_to_quaternion(theta)
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)


    @staticmethod
    def angle_to_quaternion(angle):
        """Convert angle (in radians) to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(angle / 2.0)
        q.w = math.cos(angle / 2.0)
        return q


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = FeedbackProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
