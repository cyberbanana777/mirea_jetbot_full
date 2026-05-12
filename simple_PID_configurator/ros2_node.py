#!/usr/bin/env python3
"""
Serial Bridge GUI – PySide6 (ROS2 edition, OLED-friendly dark hacker theme)
Copyright (c) 2026 Alice Zenina & Alexander Grachev, RTU MIREA (Russia)
SPDX-License-Identifier: MIT

Использование:
  python3 fast_gui_pyside.py [-p ignored] [-b ignored]

Теперь:
- Отправка команд через топик serial/to (формат $...;#)
- Приём одометрии из /odom
- Приём скоростей колёс из /sensors/wheel/left/speed и /sensors/wheel/right/speed
- Приём PID‑коэффициентов из /PIDcoeffs
- Сырой лог сообщений из serial/from
- Поток-спиннер ROS2 для безопасности GUI
"""

import sys
import time
import argparse
from collections import deque
import threading
import math

# ---------- ROS2 ----------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

# ---------- PySide6 ----------
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QTextEdit, QDockWidget,
    QStatusBar, QMenuBar, QMenu
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtGui import QAction, QFont, QColor

# ---------- Matplotlib ----------
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
plt.style.use('dark_background')


class SerialBridgeGUI(QMainWindow):
    """
    PySide6 GUI, теперь полностью поверх ROS2.
    Оригинальная логика тем и оформления сохранена.
    """

    # Сигналы для потокобезопасного обновления GUI
    log_received_signal = Signal(str)
    log_sent_signal = Signal(str)
    status_signal = Signal(str)
    pid_values_signal = Signal(list)

    # Новые сигналы для обработанных данных
    odom_updated = Signal(float, float, float, float, float)  # x, y, θ, vx, vth
    left_speed_updated = Signal(float)
    right_speed_updated = Signal(float)

    def __init__(self, serial_port=None, baudrate=None):
        super().__init__()
        self.running = True

        # Хранилище для графиков
        self.max_data_points = 200
        self.time_data = deque(maxlen=self.max_data_points)
        self.v_linear_x_data = deque(maxlen=self.max_data_points)
        self.v_angular_z_data = deque(maxlen=self.max_data_points)
        self.v_left_data = deque(maxlen=self.max_data_points)
        self.v_right_data = deque(maxlen=self.max_data_points)
        self.start_time = time.time()

        # PID
        self.last_coefficients = [0.0] * 6
        self.current_pid_values = [0.0] * 6

        # Пресет управления и графиков (0 – колёса, 1 – платформа)
        self.control_preset = 0

        # ---------- Инициализация ROS2 ----------
        self.node = rclpy.create_node('gui_controller')

        # Паблишер для отправки команд (формат $...;#)
        self.cmd_pub = self.node.create_publisher(String, '/low_level/serial/cmd', 10)

        # Подписчики на данные
        self.node.create_subscription(
            String, '/low_level/serial/feedback', self._raw_msg_callback, 10)
        self.node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.node.create_subscription(
            Float32, '/sensors/wheel/left/speed', self._left_speed_callback, 10)
        self.node.create_subscription(
            Float32, '/sensors/wheel/right/speed', self._right_speed_callback, 10)
        self.node.create_subscription(
            String, '/PIDcoeffs', self._pid_coeff_callback, 10)

        # Поток‑спиннер (вариант 1)
        self.spin_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
        self.spin_thread.start()

        # ---------- Построение интерфейса (сохранено расположение из обновлённого оригинала) ----------
        self.setWindowTitle("ESP32 Serial Bridge – ROS2 Mode")
        self.setGeometry(100, 100, 1600, 1000)

        self._apply_dark_theme()
        self._create_actions()
        self._create_menu_bar()
        self._create_central_widget()
        self._create_console_dock()
        self._create_status_bar()

        # Подключаем сигналы к слотам
        self.log_received_signal.connect(self._append_received_log)
        self.log_sent_signal.connect(self._append_sent_log)
        self.status_signal.connect(self.statusBar().showMessage)
        self.pid_values_signal.connect(self._update_current_pid_display)
        self.odom_updated.connect(self._on_odometry_received)
        self.left_speed_updated.connect(self._on_left_speed)
        self.right_speed_updated.connect(self._on_right_speed)

        # Таймер для прорисовки графиков
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots_if_needed)
        self.plot_timer.start(50)
        self.last_plot_update = 0

    # ----------------------------------------------------------------------
    # ROS‑колбеки (выполняются в потоке спиннера, только эмитят сигналы)
    # ----------------------------------------------------------------------
    def _raw_msg_callback(self, msg: String):
        """Сырые сообщения из serial/from для лога."""
        self.log_received_signal.emit(msg.data)

    def _odom_callback(self, msg: Odometry):
        """Обработка одометрии: извлекаем позицию и скорости."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Преобразование кватерниона в угол рысканья
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        vx = msg.twist.twist.linear.x
        vth = msg.twist.twist.angular.z

        self.odom_updated.emit(x, y, theta, vx, vth)

    def _left_speed_callback(self, msg: Float32):
        self.left_speed_updated.emit(msg.data)

    def _right_speed_callback(self, msg: Float32):
        self.right_speed_updated.emit(msg.data)

    def _pid_coeff_callback(self, msg: String):
        """Парсим строку с PID коэффициентами (Kp_L;Ki_L;Kd_L;Kp_R;Ki_R;Kd_R)."""
        try:
            parts = msg.data.split(';')
            if len(parts) == 6:
                vals = [float(p) for p in parts]
                self.pid_values_signal.emit(vals)
            else:
                self.status_signal.emit(f"Некорректный формат PID: {msg.data}")
        except ValueError as e:
            self.status_signal.emit(f"Ошибка парсинга PID: {e}")

    # ----------------------------------------------------------------------
    # Поток‑спиннер ROS2
    # ----------------------------------------------------------------------
    def _ros_spin_thread(self):
        rclpy.spin(self.node)

    # ----------------------------------------------------------------------
    # Слоты, обновляющие данные для графиков (главный поток)
    # ----------------------------------------------------------------------
    @Slot(float, float, float, float, float)
    def _on_odometry_received(self, x, y, theta, vx, vth):
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.v_linear_x_data.append(vx)
        self.v_angular_z_data.append(vth)
        # Виджеты одометрии
        self.odom_x_label.setText(f"{x:.3f} м")
        self.odom_y_label.setText(f"{y:.3f} м")
        self.odom_theta_label.setText(f"{math.degrees(theta):.1f}°")

    @Slot(float)
    def _on_left_speed(self, speed):
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.v_left_data.append(speed)

    @Slot(float)
    def _on_right_speed(self, speed):
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.v_right_data.append(speed)

    # ----------------------------------------------------------------------
    # Тёмная тема (полностью из оригинала)
    # ----------------------------------------------------------------------
    def _apply_dark_theme(self):
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

    # ----------------------------------------------------------------------
    # Построение UI (в точности как в обновлённом оригинале)
    # ----------------------------------------------------------------------
    def _create_actions(self):
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

        # PID‑настройки и текущие значения
        top_layout = QHBoxLayout()
        self._build_pid_setup_group(top_layout)
        self._build_current_pid_group(top_layout)
        main_layout.addLayout(top_layout)

        # Кнопки PID
        main_layout.addLayout(self._build_pid_buttons_row())

        # Группа команд (поля ввода, Send, STOP и переключатель пресета)
        self._build_command_group(main_layout)

        # Новая группа одометрии
        self._build_odometry_group(main_layout)

        # Графики
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
        """Группа команд: поля ввода, Send, STOP в ряд, под ними переключатель пресета."""
        group = QGroupBox("Command")
        layout = QVBoxLayout(group)

        # Ряд 1: метки, поля ввода, кнопки Send и STOP
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

        # Ряд 2: кнопка переключения пресета (по центру)
        row2 = QHBoxLayout()
        row2.addStretch()
        self.preset_btn = QPushButton("Switch to Linear/Angular Control")
        self.preset_btn.clicked.connect(self._toggle_control_preset)
        row2.addWidget(self.preset_btn)
        row2.addStretch()
        layout.addLayout(row2)

        parent_layout.addWidget(group)

    def _build_odometry_group(self, parent_layout):
        """Новая группа для отображения одометрии."""
        group = QGroupBox("Odometry (from /odom)")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("X:"))
        self.odom_x_label = QLabel("0.000 м")
        layout.addWidget(self.odom_x_label)

        layout.addWidget(QLabel("Y:"))
        self.odom_y_label = QLabel("0.000 м")
        layout.addWidget(self.odom_y_label)

        layout.addWidget(QLabel("θ:"))
        self.odom_theta_label = QLabel("0.0°")
        layout.addWidget(self.odom_theta_label)

        parent_layout.addWidget(group)

    def _update_command_fields(self):
        if self.control_preset == 0:
            self.cmd_label1.setText("Left Wheel:")
            self.cmd_label2.setText("Right Wheel:")
        else:
            self.cmd_label1.setText("Linear Velocity:")
            self.cmd_label2.setText("Angular Velocity:")
        self.cmd_edit1.setText("0.0")
        self.cmd_edit2.setText("0.0")

    def _build_plots(self, parent_layout):
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
        self.statusBar().showMessage("Ready – ROS2 mode")

    # ----------------------------------------------------------------------
    # Слоты для кнопок (отправка команд через ROS2)
    # ----------------------------------------------------------------------
    @Slot()
    def _copy_current_to_setup(self):
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
        self.control_preset = 1 - self.control_preset
        if self.control_preset == 0:
            self.preset_btn.setText("Switch to Linear/Angular Control")
        else:
            self.preset_btn.setText("Switch to Wheel Control")
        self._apply_preset_labels()

    @Slot()
    def _send_command(self):
        try:
            val1 = float(self.cmd_edit1.text())
            val2 = float(self.cmd_edit2.text())
            if self.control_preset == 0:
                msg = f"$2;{val1:.3f};{val2:.3f};#"
                self.log_sent_signal.emit(f"Wheel: left={val1:.2f}, right={val2:.2f}")
            else:
                msg = f"$1;{val1:.3f};{val2:.3f};#"
                self.log_sent_signal.emit(f"Velocity: linear={val1:.2f}, angular={val2:.2f}")
            self.cmd_pub.publish(String(data=msg))
            self.status_signal.emit("Command sent")
        except ValueError:
            self.status_signal.emit("Error: invalid number in command fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_pid_coefficients(self):
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
            self.cmd_pub.publish(String(data=msg))
            self.log_sent_signal.emit(f"PID: {coeffs}")
            self.status_signal.emit("PID coefficients sent")
        except ValueError:
            self.status_signal.emit("Error: invalid number in PID fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_write_flash(self):
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
            self.cmd_pub.publish(String(data=msg))
            self.log_sent_signal.emit(f"Write PID to flash: {coeffs}")
            self.status_signal.emit("PID coefficients written to flash")
        except ValueError:
            self.status_signal.emit("Error: invalid number in PID fields")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _get_current_pid(self):
        try:
            msg = "$5;#"
            self.cmd_pub.publish(String(data=msg))
            self.log_sent_signal.emit(f"Get PID: {msg}")
            self.status_signal.emit("PID request sent")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    @Slot()
    def _send_stop(self):
        try:
            msg = "$1;0.0;0.0;#"
            self.cmd_pub.publish(String(data=msg))
            self.log_sent_signal.emit("STOP sent")
            self.status_signal.emit("STOP command sent")
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")

    # ----------------------------------------------------------------------
    # Консольные слоты
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
        self.curr_kp_l.setText(f"{values[0]:.3f}")
        self.curr_ki_l.setText(f"{values[1]:.3f}")
        self.curr_kd_l.setText(f"{values[2]:.3f}")
        self.curr_kp_r.setText(f"{values[3]:.3f}")
        self.curr_ki_r.setText(f"{values[4]:.3f}")
        self.curr_kd_r.setText(f"{values[5]:.3f}")
        self.current_pid_values = values

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
    # Обновление графиков
    # ----------------------------------------------------------------------
    @Slot()
    def _update_plots_if_needed(self):
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
    # Закрытие приложения
    # ----------------------------------------------------------------------
    def closeEvent(self, event):
        self.running = False
        # Корректное завершение ROS2
        self.node.destroy_node()
        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Serial Bridge GUI (ROS2 edition)")
    parser.add_argument('-p', '--port', type=str, help="Ignored (kept for compatibility)")
    parser.add_argument('-b', '--baudrate', type=int, help="Ignored (kept for compatibility)")
    args, _ = parser.parse_known_args()

    # Инициализация ROS2 (должен быть запущен ROS‑мастер)
    rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Дополнительная палитра для гарантии тёмного вида (можно оставить)
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

    window = SerialBridgeGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()