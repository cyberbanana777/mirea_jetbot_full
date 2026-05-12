#!/usr/bin/env python3

"""
Serial Bridge GUI - PySide6 edition with dark hacker theme (orange accents)
Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
SPDX-License-Identifier: MIT

Usage:
  python3 fast_gui_pyside.py [-p /dev/ttyUSB0] [-b 115200]
"""

import sys
import time
import argparse
from collections import deque
import threading
import queue

import serial
import config as cfg

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QTextEdit, QDockWidget,
    QStatusBar, QMenuBar, QMenu, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtGui import QAction, QFont, QColor

import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
plt.style.use('dark_background')


class SerialBridgeGUI(QMainWindow):
    """
    PySide6 GUI for Serial Bridge with ESP32
    - PID tuning
    - Real-time plots with switchable presets (wheels / platform)
    - Velocity/Wheel command sending using the same preset
    - Collapsible console
    - Dark hacker theme (orange accents)
    """

    # Custom signals for thread-safe GUI updates
    log_received_signal = Signal(str)
    log_sent_signal = Signal(str)
    status_signal = Signal(str)
    pid_values_signal = Signal(list)  # current PID values from device

    def __init__(self, serial_port='/dev/esp32', baudrate=115200):
        super().__init__()
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.running = True

        # Data storage
        self.max_data_points = 200
        self.time_data = deque(maxlen=self.max_data_points)
        self.v_linear_x_data = deque(maxlen=self.max_data_points)
        self.v_angular_z_data = deque(maxlen=self.max_data_points)
        self.v_left_data = deque(maxlen=self.max_data_points)
        self.v_right_data = deque(maxlen=self.max_data_points)
        self.start_time = time.time()

        # PID coefficients storage
        self.last_coefficients = [0.0] * 6
        self.current_pid_values = [0.0] * 6

        # Control/plot preset: 0 = wheel velocities, 1 = platform velocities
        self.control_preset = 0

        # Serial connection
        self.init_serial()

        # Queue for incoming messages
        self.received_queue = queue.Queue()

        # Build the UI
        self.setWindowTitle("ESP32 Serial Bridge - PID & Plots")
        self.setGeometry(100, 100, 1600, 950)

        # Apply dark hacker theme (orange)
        self._apply_dark_theme()

        self._create_actions()
        self._create_menu_bar()
        self._create_central_widget()
        self._create_console_dock()
        self._create_status_bar()

        # Connect signals to slots
        self.log_received_signal.connect(self._append_received_log)
        self.log_sent_signal.connect(self._append_sent_log)
        self.status_signal.connect(self.statusBar().showMessage)
        self.pid_values_signal.connect(self._update_current_pid_display)

        # Start serial reading thread
        self.read_thread = threading.Thread(target=self._read_from_serial_optimized, daemon=True)
        self.read_thread.start()

        # Timer for periodic GUI updates (process incoming queue)
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._update_gui)
        self.gui_timer.start(150)

        # Plot update timer at higher rate
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots_if_needed)
        self.plot_timer.start(50)

        self.last_plot_update = 0

    def _apply_dark_theme(self):
        """Set a global dark hacker stylesheet with orange accents"""
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #ff8c00;
                font-family: "Courier New", monospace;
                font-size: 11pt;
            }
            QGroupBox {
                border: 1px solid #ff8c00;
                border-radius: 4px;
                margin-top: 1ex;
                padding-top: 10px;
                font-weight: bold;
                color: #ff8c00;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QLineEdit, QTextEdit {
                background-color: #000000;
                color: #ff8c00;
                border: 1px solid #ff8c00;
                padding: 6px;
                selection-background-color: #ff8c00;
                selection-color: #000000;
                font-size: 12pt;
            }
            QPushButton {
                background-color: #2b2b2b;
                color: #ff8c00;
                border: 1px solid #ff8c00;
                padding: 10px 20px;
                font-weight: bold;
                min-width: 120px;
                min-height: 40px;
                font-size: 11pt;
            }
            QPushButton:hover {
                background-color: #3c3c3c;
            }
            QPushButton:pressed {
                background-color: #cc7000;
                color: #000000;
            }
            QLabel {
                color: #ff8c00;
                font-size: 11pt;
            }
            QMenuBar {
                background-color: #1e1e1e;
                color: #ff8c00;
            }
            QMenuBar::item:selected {
                background: #ff8c00;
                color: #000000;
            }
            QMenu {
                background-color: #1e1e1e;
                color: #ff8c00;
                border: 1px solid #ff8c00;
            }
            QMenu::item:selected {
                background-color: #ff8c00;
                color: #000000;
            }
            QDockWidget {
                color: #ff8c00;
                titlebar-close-icon: url(none);
            }
            QStatusBar {
                background-color: #1e1e1e;
                color: #ff8c00;
            }
        """)

    def init_serial(self):
        """Initialize serial connection"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.status_signal.emit(f"Serial port {self.serial_port} opened at {self.baudrate} baud")
        except serial.SerialException as e:
            self.ser = None
            self.status_signal.emit(f"Error opening port: {e}")

    # ----------------------------------------------------------------------
    # UI creation
    # ----------------------------------------------------------------------
    def _create_actions(self):
        """Create actions used in menus/toolbars"""
        self.toggle_console_action = QAction("Toggle Console", self)
        self.toggle_console_action.setCheckable(True)
        self.toggle_console_action.setChecked(False)
        self.toggle_console_action.triggered.connect(self._toggle_console)

        self.clear_sent_action = QAction("Clear Sent Log", self)
        self.clear_sent_action.triggered.connect(self._clear_sent_log)
        self.clear_received_action = QAction("Clear Received Log", self)
        self.clear_received_action.triggered.connect(self._clear_received_log)

    def _create_menu_bar(self):
        menu_bar = self.menuBar()
        view_menu = menu_bar.addMenu("View")
        view_menu.addAction(self.toggle_console_action)

        log_menu = menu_bar.addMenu("Log")
        log_menu.addAction(self.clear_sent_action)
        log_menu.addAction(self.clear_received_action)

    def _create_central_widget(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # --- Top: PID Setup & Current Values (side by side) ---
        top_layout = QHBoxLayout()
        self._build_pid_setup_group(top_layout)
        self._build_current_pid_group(top_layout)
        main_layout.addLayout(top_layout)

        # --- Row of PID action buttons ---
        main_layout.addLayout(self._build_pid_buttons_row())

        # --- Command group (velocities, send, stop, switch) ---
        self._build_command_group(main_layout)

        # --- Plots ---
        self._build_plots(main_layout)

    def _build_pid_setup_group(self, parent_layout):
        group = QGroupBox("PID Coefficients Setup")
        layout = QVBoxLayout(group)

        pid_layout = QHBoxLayout()

        left_frame = QGroupBox("Left Motor")
        left_form = QVBoxLayout(left_frame)
        self.kp_l_edit = QLineEdit("0.0")
        self.ki_l_edit = QLineEdit("0.0")
        self.kd_l_edit = QLineEdit("0.0")
        for label_text, widget in [
            ("Kp_L:", self.kp_l_edit), ("Ki_L:", self.ki_l_edit), ("Kd_L:", self.kd_l_edit)
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label_text))
            row.addWidget(widget)
            left_form.addLayout(row)
        pid_layout.addWidget(left_frame)

        right_frame = QGroupBox("Right Motor")
        right_form = QVBoxLayout(right_frame)
        self.kp_r_edit = QLineEdit("0.0")
        self.ki_r_edit = QLineEdit("0.0")
        self.kd_r_edit = QLineEdit("0.0")
        for label_text, widget in [
            ("Kp_R:", self.kp_r_edit), ("Ki_R:", self.ki_r_edit), ("Kd_R:", self.kd_r_edit)
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label_text))
            row.addWidget(widget)
            right_form.addLayout(row)
        pid_layout.addWidget(right_frame)

        layout.addLayout(pid_layout)
        parent_layout.addWidget(group)

    def _build_current_pid_group(self, parent_layout):
        group = QGroupBox("Current PID Values")
        layout = QVBoxLayout(group)

        pid_layout = QHBoxLayout()

        left_frame = QGroupBox("Left Motor")
        left_form = QVBoxLayout(left_frame)
        self.curr_kp_l = QLabel("0.0")
        self.curr_ki_l = QLabel("0.0")
        self.curr_kd_l = QLabel("0.0")
        for label_text, widget in [
            ("Kp_L:", self.curr_kp_l), ("Ki_L:", self.curr_ki_l), ("Kd_L:", self.curr_kd_l)
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label_text))
            row.addWidget(widget)
            left_form.addLayout(row)
        pid_layout.addWidget(left_frame)

        right_frame = QGroupBox("Right Motor")
        right_form = QVBoxLayout(right_frame)
        self.curr_kp_r = QLabel("0.0")
        self.curr_ki_r = QLabel("0.0")
        self.curr_kd_r = QLabel("0.0")
        for label_text, widget in [
            ("Kp_R:", self.curr_kp_r), ("Ki_R:", self.curr_ki_r), ("Kd_R:", self.curr_kd_r)
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label_text))
            row.addWidget(widget)
            right_form.addLayout(row)
        pid_layout.addWidget(right_frame)

        layout.addLayout(pid_layout)
        parent_layout.addWidget(group)

    def _build_pid_buttons_row(self):
        """Create a horizontal row: Write Flash | Send PID | Copy Current->Setup | Get PID"""
        row = QHBoxLayout()
        row.setSpacing(10)

        self.write_flash_btn = QPushButton("Write to Flash")
        self.write_flash_btn.setStyleSheet("background-color: #8b3a62; color: #ff8c00;")
        self.write_flash_btn.clicked.connect(self._send_write_flash)
        row.addWidget(self.write_flash_btn)

        self.send_pid_btn = QPushButton("Send PID Coefficients")
        self.send_pid_btn.setStyleSheet("background-color: #3d1a00; color: #ff8c00;")
        self.send_pid_btn.clicked.connect(self._send_pid_coefficients)
        row.addWidget(self.send_pid_btn)

        self.copy_pid_btn = QPushButton("Copy Current → Setup")
        self.copy_pid_btn.setStyleSheet("background-color: #1a1a1a; color: #ff8c00;")
        self.copy_pid_btn.clicked.connect(self._copy_current_to_setup)
        row.addWidget(self.copy_pid_btn)

        self.get_pid_btn = QPushButton("Get Current PID")
        self.get_pid_btn.setStyleSheet("background-color: #001a33; color: #ff8c00;")
        self.get_pid_btn.clicked.connect(self._get_current_pid)
        row.addWidget(self.get_pid_btn)

        return row

    def _build_command_group(self, parent_layout):
        """Group for velocity/wheel commands: labels + edits + Send + STOP in row1,
           switch preset button below."""
        group = QGroupBox("Command")
        layout = QVBoxLayout(group)

        # Row 1: labels, inputs, Send, STOP
        row1 = QHBoxLayout()
        self.cmd_label1 = QLabel("Left Wheel:")
        self.cmd_edit1 = QLineEdit("0.0")
        self.cmd_edit1.setMinimumWidth(120)
        self.cmd_label2 = QLabel("Right Wheel:")
        self.cmd_edit2 = QLineEdit("0.0")
        self.cmd_edit2.setMinimumWidth(120)

        self.send_cmd_btn = QPushButton("Send")
        self.send_cmd_btn.setStyleSheet("background-color: #3d1a00; color: #ff8c00;")
        self.send_cmd_btn.clicked.connect(self._send_command)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet("background-color: #7f0000; color: #ffffff; font-size: 18pt; font-weight: bold;")
        self.stop_btn.setFixedHeight(60)
        self.stop_btn.setMinimumWidth(120)
        self.stop_btn.clicked.connect(self._send_stop)

        row1.addWidget(self.cmd_label1)
        row1.addWidget(self.cmd_edit1)
        row1.addWidget(self.cmd_label2)
        row1.addWidget(self.cmd_edit2)
        row1.addWidget(self.send_cmd_btn)
        row1.addWidget(self.stop_btn)
        layout.addLayout(row1)

        # Row 2: switch preset button, centered
        row2 = QHBoxLayout()
        row2.addStretch()
        self.preset_btn = QPushButton("Switch to Linear/Angular Control")
        self.preset_btn.clicked.connect(self._toggle_control_preset)
        row2.addWidget(self.preset_btn)
        row2.addStretch()
        layout.addLayout(row2)

        parent_layout.addWidget(group)

    def _update_command_fields(self):
        """Change labels and default values based on control_preset"""
        if self.control_preset == 0:
            self.cmd_label1.setText("Left Wheel:")
            self.cmd_label2.setText("Right Wheel:")
        else:
            self.cmd_label1.setText("Linear Velocity:")
            self.cmd_label2.setText("Angular Velocity:")
        self.cmd_edit1.setText("0.0")
        self.cmd_edit2.setText("0.0")

    def _build_plots(self, parent_layout):
        """Create Matplotlib canvas with two subplots"""
        self.figure = Figure(figsize=(14, 5))
        self.figure.patch.set_facecolor('#1e1e1e')
        self.canvas = FigureCanvas(self.figure)
        parent_layout.addWidget(self.canvas)

        self.ax1 = self.figure.add_subplot(121)
        self.ax2 = self.figure.add_subplot(122)
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor('#1e1e1e')
            ax.tick_params(colors='#ff8c00')
            ax.xaxis.label.set_color('#ff8c00')
            ax.yaxis.label.set_color('#ff8c00')
            ax.title.set_color('#ff8c00')
            ax.grid(True, color='#ff8c00', alpha=0.3)

        self.line1, = self.ax1.plot([], [], '#ff8c00', linewidth=1.5)
        self.line2, = self.ax2.plot([], [], '#ffb347', linewidth=1.5)

        self._apply_preset_labels()
        self.figure.tight_layout()

    def _apply_preset_labels(self):
        """Update plot titles, line colors and command fields according to current preset"""
        if self.control_preset == 0:
            self.ax1.set_title('Left Wheel Velocity (v_left)')
            self.ax2.set_title('Right Wheel Velocity (v_right)')
            self.line1.set_color('#ff8c00')
            self.line2.set_color('#ffb347')
        else:
            self.ax1.set_title('Linear Velocity (v_linear_x)')
            self.ax2.set_title('Angular Velocity (v_angular_z)')
            self.line1.set_color('#ff8c00')
            self.line2.set_color('#ffb347')
        self.canvas.draw_idle()
        self._update_command_fields()

    def _create_console_dock(self):
        """Collapsible console for sent/received messages"""
        self.console_dock = QDockWidget("Communication Console", self)
        self.console_dock.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)
        console_widget = QWidget()
        console_layout = QHBoxLayout(console_widget)

        sent_group = QGroupBox("Sent Messages")
        sent_layout = QVBoxLayout(sent_group)
        self.sent_text = QTextEdit()
        self.sent_text.setReadOnly(True)
        self.sent_text.setFont(QFont("Courier", 9))
        sent_layout.addWidget(self.sent_text)
        console_layout.addWidget(sent_group)

        recv_group = QGroupBox("Received Messages")
        recv_layout = QVBoxLayout(recv_group)
        self.received_text = QTextEdit()
        self.received_text.setReadOnly(True)
        self.received_text.setFont(QFont("Courier", 9))
        recv_layout.addWidget(self.received_text)
        console_layout.addWidget(recv_group)

        self.console_dock.setWidget(console_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.console_dock)
        self.console_dock.hide()

    def _create_status_bar(self):
        self.statusBar().showMessage("Ready")

    # ----------------------------------------------------------------------
    # Slots for button actions
    # ----------------------------------------------------------------------
    @Slot()
    def _copy_current_to_setup(self):
        """Copy current PID values to the setup entry fields"""
        if len(self.current_pid_values) == 6:
            self.kp_l_edit.setText(f"{self.current_pid_values[0]:.6f}")
            self.ki_l_edit.setText(f"{self.current_pid_values[1]:.6f}")
            self.kd_l_edit.setText(f"{self.current_pid_values[2]:.6f}")
            self.kp_r_edit.setText(f"{self.current_pid_values[3]:.6f}")
            self.ki_r_edit.setText(f"{self.current_pid_values[4]:.6f}")
            self.kd_r_edit.setText(f"{self.current_pid_values[5]:.6f}")
            self.status_signal.emit("Current PID copied to setup fields")

    @Slot()
    def _toggle_control_preset(self):
        """Switch between wheel control (preset 0) and platform velocity (preset 1)"""
        self.control_preset = 1 - self.control_preset
        if self.control_preset == 0:
            self.preset_btn.setText("Switch to Linear/Angular Control")
        else:
            self.preset_btn.setText("Switch to Wheel Control")
        self._apply_preset_labels()

    @Slot()
    def _send_command(self):
        """Send velocity/wheel command based on current preset"""
        if not self.ser or not self.ser.is_open:
            self.status_signal.emit("Error: serial port not open")
            return
        try:
            val1 = float(self.cmd_edit1.text())
            val2 = float(self.cmd_edit2.text())
            if self.control_preset == 0:
                msg = f"$2;{val1:.3f};{val2:.3f};#"
                self.log_sent_signal.emit(f"Wheel: left={val1:.2f}, right={val2:.2f}")
            else:
                msg = f"$1;{val1:.3f};{val2:.3f};#"
                self.log_sent_signal.emit(f"Velocity: linear={val1:.2f}, angular={val2:.2f}")
            self.ser.write(msg.encode('utf-8'))
            self.status_signal.emit("Command sent")
        except ValueError:
            self.status_signal.emit("Error: invalid number in command fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_pid_coefficients(self):
        """Send PID coefficients using command $3"""
        if not self.ser or not self.ser.is_open:
            self.status_signal.emit("Error: serial port not open")
            return
        try:
            coeffs = [
                float(self.kp_l_edit.text()),
                float(self.ki_l_edit.text()),
                float(self.kd_l_edit.text()),
                float(self.kp_r_edit.text()),
                float(self.ki_r_edit.text()),
                float(self.kd_r_edit.text())
            ]
            msg = f"$3;{';'.join(f'{c:.6f}' for c in coeffs)};#"
            self.ser.write(msg.encode('utf-8'))
            self.log_sent_signal.emit(f"PID: {coeffs}")
            self.status_signal.emit("PID coefficients sent")
        except ValueError:
            self.status_signal.emit("Error: invalid number in PID fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_write_flash(self):
        """Write PID coefficients to flash ($4)"""
        if not self.ser or not self.ser.is_open:
            self.status_signal.emit("Error: serial port not open")
            return
        try:
            coeffs = [
                float(self.kp_l_edit.text()),
                float(self.ki_l_edit.text()),
                float(self.kd_l_edit.text()),
                float(self.kp_r_edit.text()),
                float(self.ki_r_edit.text()),
                float(self.kd_r_edit.text())
            ]
            msg = f"$4;{';'.join(f'{c:.6f}' for c in coeffs)};#"
            self.ser.write(msg.encode('utf-8'))
            self.log_sent_signal.emit(f"Write PID to flash: {coeffs}")
            self.status_signal.emit("PID coefficients written to flash")
        except ValueError:
            self.status_signal.emit("Error: invalid number in PID fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _get_current_pid(self):
        """Request current PID coefficients from device ($5)"""
        if not self.ser or not self.ser.is_open:
            self.status_signal.emit("Error: serial port not open")
            return
        try:
            msg = "$5;#"
            self.ser.write(msg.encode('utf-8'))
            self.log_sent_signal.emit(f"Get PID: {msg}")
            self.status_signal.emit("PID request sent")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_stop(self):
        """Send emergency stop ($1;0.0;0.0;# )"""
        if not self.ser or not self.ser.is_open:
            self.status_signal.emit("Error: serial port not open")
            return
        try:
            msg = "$1;0.0;0.0;#"
            self.ser.write(msg.encode('utf-8'))
            self.log_sent_signal.emit("STOP sent")
            self.status_signal.emit("STOP command sent")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _toggle_console(self, checked):
        if checked:
            self.console_dock.show()
        else:
            self.console_dock.hide()

    @Slot()
    def _clear_sent_log(self):
        self.sent_text.clear()

    @Slot()
    def _clear_received_log(self):
        self.received_text.clear()

    # ----------------------------------------------------------------------
    # Logging helpers (called via signals from any thread)
    # ----------------------------------------------------------------------
    @Slot(str)
    def _append_sent_log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.sent_text.append(f"[{timestamp}] {message}")

    @Slot(str)
    def _append_received_log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.received_text.append(f"[{timestamp}] {message}")

    @Slot(list)
    def _update_current_pid_display(self, values):
        """Update the labels showing current PID coefficients"""
        self.curr_kp_l.setText(f"{values[0]:.3f}")
        self.curr_ki_l.setText(f"{values[1]:.3f}")
        self.curr_kd_l.setText(f"{values[2]:.3f}")
        self.curr_kp_r.setText(f"{values[3]:.3f}")
        self.curr_ki_r.setText(f"{values[4]:.3f}")
        self.curr_kd_r.setText(f"{values[5]:.3f}")
        self.current_pid_values = values

    # ----------------------------------------------------------------------
    # Serial reading thread (same logic as original)
    # ----------------------------------------------------------------------
    def _read_from_serial_optimized(self):
        serial_buffer = ''
        last_processing_time = 0
        processing_interval = 0.05

        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.5)
                continue

            try:
                current_time = time.time()
                if current_time - last_processing_time < processing_interval:
                    time.sleep(0.01)
                    continue

                last_processing_time = current_time

                if self.ser.in_waiting > 0:
                    data_bytes = self.ser.read(self.ser.in_waiting)
                    data_str = data_bytes.decode('utf-8', errors='ignore')
                    serial_buffer += data_str

                    while True:
                        start_idx = serial_buffer.find('$')
                        if start_idx == -1:
                            serial_buffer = ''
                            break

                        end_idx = serial_buffer.find('#', start_idx + 1)
                        if end_idx == -1:
                            serial_buffer = serial_buffer[start_idx:]
                            break

                        full_message = serial_buffer[start_idx:end_idx + 1]

                        if full_message.startswith('$') and full_message.endswith('#'):
                            self.received_queue.put(full_message)

                            data = self._parse_received_message_fast(full_message)
                            if data:
                                if 'v_linear_x' in data:
                                    self._add_data_point(data)
                                if 'pid_values' in data:
                                    self.pid_values_signal.emit(data['pid_values'])

                        serial_buffer = serial_buffer[end_idx + 1:]
                        if not serial_buffer:
                            break

            except Exception as e:
                time.sleep(0.1)

    def _parse_received_message_fast(self, message):
        """Parse incoming message, return dict with velocity data or pid_values"""
        if not message.startswith('$') or not message.endswith(';#'):
            return None
        try:
            parts = message[1:-2].split(';')
            if not parts:
                return None
            msg_type = parts[0]

            if msg_type == '1' and len(parts) >= 8:
                return {
                    'v_linear_x': float(parts[4]),
                    'v_angular_z': float(parts[5]),
                    'v_left': float(parts[6]),
                    'v_right': float(parts[7])
                }
            elif msg_type == '2' and len(parts) >= 7:
                pid_vals = [float(x) for x in parts[1:7]]
                return {'pid_values': pid_vals}
        except (ValueError, IndexError):
            return None
        return None

    def _add_data_point(self, data):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.v_linear_x_data.append(data.get('v_linear_x', 0))
        self.v_angular_z_data.append(data.get('v_angular_z', 0))
        self.v_left_data.append(data.get('v_left', 0))
        self.v_right_data.append(data.get('v_right', 0))

    # ----------------------------------------------------------------------
    # GUI periodic updates
    # ----------------------------------------------------------------------
    @Slot()
    def _update_gui(self):
        """Process incoming messages and update log"""
        processed = 0
        while not self.received_queue.empty() and processed < 10:
            try:
                msg = self.received_queue.get_nowait()
                self.log_received_signal.emit(msg)
                processed += 1
            except queue.Empty:
                break

    @Slot()
    def _update_plots_if_needed(self):
        """Redraw plots if enough time has elapsed"""
        current_time = time.time()
        if current_time - self.last_plot_update < 0.05:
            return
        self.last_plot_update = current_time

        if not self.time_data:
            return

        if self.control_preset == 0:
            self.line1.set_data(self.time_data, self.v_left_data)
            self.line2.set_data(self.time_data, self.v_right_data)
        else:
            self.line1.set_data(self.time_data, self.v_linear_x_data)
            self.line2.set_data(self.time_data, self.v_angular_z_data)

        for ax in [self.ax1, self.ax2]:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    # ----------------------------------------------------------------------
    # Close event
    # ----------------------------------------------------------------------
    def closeEvent(self, event):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Serial Bridge GUI with PID tuning")
    parser.add_argument('-p', '--port', type=str, default=None, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument('-b', '--baudrate', type=int, default=None, help="Baudrate (e.g., 115200)")
    args = parser.parse_args()

    serial_port = args.port if args.port else cfg.SERIAL_PORT
    baudrate = args.baudrate if args.baudrate else cfg.BAUDRATE

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = app.palette()
    palette.setColor(palette.ColorRole.Window, QColor(30, 30, 30))
    palette.setColor(palette.ColorRole.WindowText, QColor(255, 140, 0))
    palette.setColor(palette.ColorRole.Base, QColor(20, 20, 20))
    palette.setColor(palette.ColorRole.Text, QColor(255, 140, 0))
    palette.setColor(palette.ColorRole.Button, QColor(40, 40, 40))
    palette.setColor(palette.ColorRole.ButtonText, QColor(255, 140, 0))
    palette.setColor(palette.ColorRole.Highlight, QColor(200, 110, 0))
    palette.setColor(palette.ColorRole.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    window = SerialBridgeGUI(serial_port, baudrate)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()