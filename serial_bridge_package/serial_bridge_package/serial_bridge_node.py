#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS2-мост для двунаправленной связи между UART-устройствами и ROS-топиками с
буферизацией и парсингом сообщений в формате $message#. Использует
многопоточность для асинхронного чтения порта, поддерживает настраиваемые
параметры подключения. Требует прав доступа к последовательному порту и
стабильного соединения с устройством.

ANNOTATION
ROS2 bridge for bidirectional communication between UART devices and ROS
topics with message buffering and parsing in $message# format. Uses
multithreading for asynchronous port reading, supports configurable
connection parameters. Requires serial port access privileges and stable
device connection.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

# Protection against garbage on the line: an unfinished frame longer than
# this is dropped (real ESP32 frames are < 100 chars).
MAX_FRAME_LEN = 256


def extract_frames(buffer):
    """
    Extract complete '$...#' frames from the text buffer.

    Returns (frames, rest), where rest is the unfinished tail to keep.
    Like the ESP32 receiver, a new '$' restarts the frame, so a broken
    frame without '#' does not swallow the next valid one.
    """
    frames = []
    while True:
        start = buffer.find('$')
        if start == -1:
            return frames, ''
        end = buffer.find('#', start + 1)
        if end == -1:
            rest = buffer[buffer.rfind('$'):]
            if len(rest) > MAX_FRAME_LEN:
                rest = ''
            return frames, rest
        # Restart from the last '$' before '#'
        start = buffer.rfind('$', start, end)
        frames.append(buffer[start:end + 1])
        buffer = buffer[end + 1:]


class SerialBridgeNode(Node):
    """
    ROS2 node for bridging between ROS2 topics and serial port communication.
    """

    def __init__(self):
        super().__init__('serial_bridge_node')

        # Declare parameters for port configuration
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ros_out_topic', 'serial/from')
        self.declare_parameter('ros_in_topic', 'serial/to')

        # Get parameter values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        ros_out_topic = self.get_parameter('ros_out_topic').get_parameter_value().string_value
        ros_in_topic = self.get_parameter('ros_in_topic').get_parameter_value().string_value

        # Initialize serial port
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.5  # Read timeout in seconds
            )
            self.get_logger().info(
                f"Serial port {serial_port} opened at {baudrate} baud"
                )
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening port {serial_port}: {e}")
            raise e

        # Create Publisher for sending data FROM serial port TO ROS
        self.pub_to_ros = self.create_publisher(String, ros_out_topic, 10)

        # Create Subscriber for receiving data FROM ROS and sending TO serial port
        self.sub_from_ros = self.create_subscription(
            String,
            ros_in_topic,
            self.ros_callback,
            10
        )

        # Flag for controlling worker threads
        self.running = True

        # Start thread for reading from serial port
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True  # Thread will exit when main thread exits
        self.read_thread.start()

        self.get_logger().info("Serial bridge node started. Waiting for data...")

    def ros_callback(self, msg):
        """
        Callback for messages received from ROS topic. Sends data to
        serial port.
        """
        try:
            data_to_send = msg.data
            self.ser.write(data_to_send.encode('utf-8'))
            self.get_logger().debug(f"Sent to UART: {data_to_send.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def read_from_serial(self):
        """
        Worker thread: reads data from serial port and publishes to ROS topic.

        The read is BLOCKING: ser.read() sleeps inside the OS until at least
        one byte arrives or the port timeout expires. The previous version
        polled ser.in_waiting in a loop without any wait, which kept one CPU
        core at 100% and held the GIL, slowing down rclpy callbacks.
        """
        buffer = ''

        while self.running and rclpy.ok():
            try:
                # Wait for at least 1 byte (or timeout), then take everything
                # that is already in the driver buffer.
                data_bytes = self.ser.read(max(1, self.ser.in_waiting))
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")
                break

            if not data_bytes:
                continue  # timeout, no data - just wait again

            buffer += data_bytes.decode('utf-8', errors='ignore')
            frames, buffer = extract_frames(buffer)

            for frame in frames:
                ros_msg = String()
                ros_msg.data = frame
                self.pub_to_ros.publish(ros_msg)

    def destroy_node(self):
        """Cleanup when node is stopped."""
        self.get_logger().info("Shutting down serial bridge node...")
        self.running = False
        
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        
        super().destroy_node()


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    serial_bridge_node = SerialBridgeNode()

    try:
        rclpy.spin(serial_bridge_node)
    except KeyboardInterrupt:
        serial_bridge_node.get_logger().info(
            "Serial bridge-node stopped by user (Ctrl+C)"
            )
    finally:
        serial_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()