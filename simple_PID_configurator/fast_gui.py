#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import serial
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import time
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import style
style.use('ggplot')

import config as cfg

class SerialBridgeGUI:
    """
    Оптимизированная GUI для Serial Bridge с ESP32
    """

    def __init__(self, serial_port='/dev/esp32', baudrate=115200):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.running = True
        
        # Store last sent coefficients for tab 3
        self.last_coefficients = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
        self.current_pid_values = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
        
        # Оптимизированные структуры данных
        self.max_data_points = 200
        self.time_data = deque(maxlen=self.max_data_points)
        self.v_linear_x_data = deque(maxlen=self.max_data_points)
        self.v_angular_z_data = deque(maxlen=self.max_data_points)
        self.v_left_data = deque(maxlen=self.max_data_points)
        self.v_right_data = deque(maxlen=self.max_data_points)
        
        self.start_time = time.time()
        self.last_plot_update = 0
        self.plot_update_interval = 0.05  # Высокая частота обновления 50мс
        
        # Кэш для линий графиков
        self.plot_lines = {}
        
        # Initialize serial connection
        self.init_serial()
        
        # Create GUI
        self.root = tk.Tk()
        self.root.title("ESP32 Serial Bridge - Optimized")
        self.root.geometry("1920x1080")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Queue for thread-safe communication
        self.received_queue = queue.Queue()
        
        # Create GUI elements
        self.create_widgets()
        
        # Start threads
        self.start_threads()
        
        # Start GUI update loop
        self.update_gui()

    def init_serial(self):
        """Initialize serial connection"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            print(f"Serial port {self.serial_port} opened at {self.baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening port {self.serial_port}: {e}")
            self.ser = None

    def create_widgets(self):
        """Create all GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=0)  # Notebook
        main_frame.rowconfigure(1, weight=1)  # Logs and STOP
        main_frame.rowconfigure(2, weight=0)  # Status
        
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        # Create frames for each tab
        self.tab1_frame = ttk.Frame(self.notebook, padding="5")
        self.tab2_frame = ttk.Frame(self.notebook, padding="5")
        self.tab3_frame = ttk.Frame(self.notebook, padding="5")
        self.plots_frame = ttk.Frame(self.notebook, padding="5")
        
        self.notebook.add(self.tab1_frame, text="Velocity Control")
        self.notebook.add(self.tab2_frame, text="Wheel Control") 
        self.notebook.add(self.tab3_frame, text="PID Tuning")
        self.notebook.add(self.plots_frame, text="All Plots")
        
        # Configure tab frames to use full width
        for tab_frame in [self.tab1_frame, self.tab2_frame, self.tab3_frame, self.plots_frame]:
            tab_frame.columnconfigure(0, weight=1)
            tab_frame.rowconfigure(1, weight=1)  # Give weight to plot rows
        
        self.create_plots_content()
        
        # Create content for each tab
        self.create_tab1_content()
        self.create_tab2_content()
        self.create_tab3_content()
        
        # Log frames with STOP button in between
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)  # Sent log
        log_frame.columnconfigure(1, weight=0)  # STOP button
        log_frame.columnconfigure(2, weight=1)  # Received log
        log_frame.rowconfigure(0, weight=1)
        
        # Sent messages log
        sent_frame = ttk.LabelFrame(log_frame, text="Sent", padding="3")
        sent_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 3))
        sent_frame.columnconfigure(0, weight=1)
        sent_frame.rowconfigure(0, weight=1)
        
        self.sent_text = scrolledtext.ScrolledText(sent_frame, height=12, state=tk.DISABLED, font=("Courier", 9))
        self.sent_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # STOP button frame (between logs)
        stop_frame = ttk.Frame(log_frame)
        stop_frame.grid(row=0, column=1, sticky=(tk.N, tk.S), padx=10)
        stop_frame.rowconfigure(0, weight=1)
        stop_frame.rowconfigure(1, weight=0)
        stop_frame.rowconfigure(2, weight=1)
        
        # Large red STOP button
        self.stop_button = tk.Button(
            stop_frame, 
            text="STOP", 
            command=self.send_stop_message,
            bg="red", 
            fg="white", 
            font=("Arial", 20, "bold"),
            width=8,
            height=4
        )
        self.stop_button.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=10)
        
        # Received messages log
        received_frame = ttk.LabelFrame(log_frame, text="Received", padding="3")
        received_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(3, 0))
        received_frame.columnconfigure(0, weight=1)
        received_frame.rowconfigure(0, weight=1)
        
        self.received_text = scrolledtext.ScrolledText(received_frame, height=12, state=tk.DISABLED, font=("Courier", 9))
        self.received_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status bar
        status_frame = ttk.Frame(main_frame)
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        self.status_label = ttk.Label(status_frame, text="Ready")
        self.status_label.pack(side=tk.LEFT)
        
        # Clear buttons
        clear_sent_button = ttk.Button(status_frame, text="Clear Sent", command=self.clear_sent_log)
        clear_sent_button.pack(side=tk.RIGHT, padx=(3, 0))
        
        clear_received_button = ttk.Button(status_frame, text="Clear Received", command=self.clear_received_log)
        clear_received_button.pack(side=tk.RIGHT)

    def create_tab1_content(self):
        """Create content for tab 1: Velocity Control - Full Width"""
        # Main container with two rows: controls and plots
        main_container = ttk.Frame(self.tab1_frame)
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_container.columnconfigure(0, weight=1)
        main_container.rowconfigure(0, weight=0)  # Controls
        main_container.rowconfigure(1, weight=1)  # Plots
        
        # Controls frame - full width
        control_frame = ttk.LabelFrame(main_container, text="Velocity Control", padding="15")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 5))
        control_frame.columnconfigure(0, weight=0)  # Input fields column
        control_frame.columnconfigure(1, weight=0)  # Button column
        control_frame.columnconfigure(2, weight=1)  # Spacer
        
        # Title
        title = ttk.Label(control_frame, text="Velocity Control", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, columnspan=3, pady=(0, 15))
        
        # Input fields in one column
        linear_label = ttk.Label(control_frame, text="Linear velocity:", font=("Arial", 11))
        linear_label.grid(row=1, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.linear_entry = ttk.Entry(control_frame, width=12, font=("Arial", 11))
        self.linear_entry.grid(row=1, column=0, sticky=tk.E, padx=(120, 0), pady=8)
        self.linear_entry.insert(0, "0.0")
        
        angular_label = ttk.Label(control_frame, text="Angular velocity:", font=("Arial", 11))
        angular_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.angular_entry = ttk.Entry(control_frame, width=12, font=("Arial", 11))
        self.angular_entry.grid(row=2, column=0, sticky=tk.E, padx=(120, 0), pady=8)
        self.angular_entry.insert(0, "0.0")
        
        # Green Send button with larger font next to input fields
        send_button = tk.Button(
            control_frame, 
            text="Send Velocity", 
            command=self.send_tab1_message,
            bg="#4CAF50",  # Green color
            fg="white",
            font=("Arial", 11, "bold"),
            width=15,
            height=2
        )
        send_button.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(20, 0), pady=8)
        
        # Plots for tab 1 - full width
        self.create_tab1_plots(main_container)

    def create_tab1_plots(self, parent_frame):
        """Create plots for tab 1: Velocity Control - Full Width"""
        try:
            plots_frame = ttk.Frame(parent_frame)
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots - larger size for full width
            self.fig_tab1 = Figure(figsize=(12, 5))
            
            # Create 2 subplots for velocity
            self.ax1_tab1 = self.fig_tab1.add_subplot(121)
            self.ax2_tab1 = self.fig_tab1.add_subplot(122)
            
            # Optimized plot settings for better performance
            self.ax1_tab1.set_title('Linear Velocity (v_linear_x)', fontsize=12)
            self.ax1_tab1.grid(True, alpha=0.3)
            line1, = self.ax1_tab1.plot([], [], 'b-', linewidth=1.5)
            self.plot_lines['v_linear_x'] = line1
            
            self.ax2_tab1.set_title('Angular Velocity (v_angular_z)', fontsize=12)
            self.ax2_tab1.grid(True, alpha=0.3)
            line2, = self.ax2_tab1.plot([], [], 'r-', linewidth=1.5)
            self.plot_lines['v_angular_z'] = line2
            
            self.fig_tab1.tight_layout()
            
            # Create canvas
            self.canvas_tab1 = FigureCanvasTkAgg(self.fig_tab1, plots_frame)
            self.canvas_tab1.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab1 plots: {e}")

    def create_tab2_content(self):
        """Create content for tab 2: Wheel Control - Full Width"""
        # Main container with two rows: controls and plots
        main_container = ttk.Frame(self.tab2_frame)
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_container.columnconfigure(0, weight=1)
        main_container.rowconfigure(0, weight=0)  # Controls
        main_container.rowconfigure(1, weight=1)  # Plots
        
        # Controls frame - full width
        control_frame = ttk.LabelFrame(main_container, text="Wheel Control", padding="15")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 5))
        control_frame.columnconfigure(0, weight=0)  # Input fields column
        control_frame.columnconfigure(1, weight=0)  # Button column
        control_frame.columnconfigure(2, weight=1)  # Spacer
        
        # Title
        title = ttk.Label(control_frame, text="Wheel Control", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, columnspan=3, pady=(0, 15))
        
        # Input fields in one column
        left_label = ttk.Label(control_frame, text="Left wheel:", font=("Arial", 11))
        left_label.grid(row=1, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.left_entry = ttk.Entry(control_frame, width=12, font=("Arial", 11))
        self.left_entry.grid(row=1, column=0, sticky=tk.E, padx=(120, 0), pady=8)
        self.left_entry.insert(0, "0.0")
        
        right_label = ttk.Label(control_frame, text="Right wheel:", font=("Arial", 11))
        right_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.right_entry = ttk.Entry(control_frame, width=12, font=("Arial", 11))
        self.right_entry.grid(row=2, column=0, sticky=tk.E, padx=(120, 0), pady=8)
        self.right_entry.insert(0, "0.0")
        
        # Green Send button with larger font next to input fields
        send_button = tk.Button(
            control_frame, 
            text="Send Wheel", 
            command=self.send_tab2_message,
            bg="#4CAF50",  # Green color
            fg="white",
            font=("Arial", 11, "bold"),
            width=15,
            height=2
        )
        send_button.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(20, 0), pady=8)
        
        # Plots for tab 2 - full width
        self.create_tab2_plots(main_container)

    def create_tab2_plots(self, parent_frame):
        """Create plots for tab 2: Wheel Control - Full Width"""
        try:
            plots_frame = ttk.Frame(parent_frame)
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots - larger size for full width
            self.fig_tab2 = Figure(figsize=(12, 5))
            
            # Create 2 subplots for wheel velocities
            self.ax1_tab2 = self.fig_tab2.add_subplot(121)
            self.ax2_tab2 = self.fig_tab2.add_subplot(122)
            
            # Optimized plot settings for better performance
            self.ax1_tab2.set_title('Left Wheel Velocity (v_left)', fontsize=12)
            self.ax1_tab2.grid(True, alpha=0.3)
            line1, = self.ax1_tab2.plot([], [], 'g-', linewidth=1.5)
            self.plot_lines['v_left'] = line1
            
            self.ax2_tab2.set_title('Right Wheel Velocity (v_right)', fontsize=12)
            self.ax2_tab2.grid(True, alpha=0.3)
            line2, = self.ax2_tab2.plot([], [], 'm-', linewidth=1.5)
            self.plot_lines['v_right'] = line2
            
            self.fig_tab2.tight_layout()
            
            # Create canvas
            self.canvas_tab2 = FigureCanvasTkAgg(self.fig_tab2, plots_frame)
            self.canvas_tab2.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab2 plots: {e}")

    def create_tab3_content(self):
        """Create content for tab 3: PID Tuning - Full Width"""
        main_container = ttk.Frame(self.tab3_frame)
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_container.columnconfigure(0, weight=1)
        main_container.rowconfigure(0, weight=0)  # Controls
        main_container.rowconfigure(1, weight=1)  # Plots
        
        # Controls container with two columns
        controls_container = ttk.Frame(main_container)
        controls_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 5))
        controls_container.columnconfigure(0, weight=1)  # Setup column
        controls_container.columnconfigure(1, weight=1)  # Current values column
        
        # Left column - PID Setup
        setup_frame = ttk.LabelFrame(controls_container, text="PID Setup", padding="15")
        setup_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        setup_frame.columnconfigure(0, weight=1)  # Left PID column
        setup_frame.columnconfigure(1, weight=1)  # Right PID column
        
        # Title for setup
        setup_title = ttk.Label(setup_frame, text="PID Coefficients Setup", font=("Arial", 14, "bold"))
        setup_title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        # Left PID setup
        left_pid_frame = ttk.LabelFrame(setup_frame, text="Left Motor PID", padding="10")
        left_pid_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        left_pid_frame.columnconfigure(1, weight=1)
        
        ttk.Label(left_pid_frame, text="Kp_L:", font=("Arial", 11)).grid(row=0, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(left_pid_frame, text="Ki_L:", font=("Arial", 11)).grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(left_pid_frame, text="Kd_L:", font=("Arial", 11)).grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        
        # Right PID setup
        right_pid_frame = ttk.LabelFrame(setup_frame, text="Right Motor PID", padding="10")
        right_pid_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        right_pid_frame.columnconfigure(1, weight=1)
        
        ttk.Label(right_pid_frame, text="Kp_R:", font=("Arial", 11)).grid(row=0, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(right_pid_frame, text="Ki_R:", font=("Arial", 11)).grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(right_pid_frame, text="Kd_R:", font=("Arial", 11)).grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        
        # Create PID entries for left and right
        self.pid_entries = []
        pid_labels = ['Kp_L', 'Ki_L', 'Kd_L', 'Kp_R', 'Ki_R', 'Kd_R']
        
        # Left PID entries
        for i in range(3):
            entry = ttk.Entry(left_pid_frame, width=12, font=("Arial", 11))
            entry.grid(row=i, column=1, sticky=(tk.W, tk.E), pady=6)
            entry.insert(0, "0.0")
            self.pid_entries.append(entry)
        
        # Right PID entries
        for i in range(3):
            entry = ttk.Entry(right_pid_frame, width=12, font=("Arial", 11))
            entry.grid(row=i, column=1, sticky=(tk.W, tk.E), pady=6)
            entry.insert(0, "0.0")
            self.pid_entries.append(entry)
        
        # Button container for Send and Write to Flash
        button_container = ttk.Frame(setup_frame)
        button_container.grid(row=2, column=0, columnspan=2, pady=15)
        button_container.columnconfigure(0, weight=1)
        button_container.columnconfigure(1, weight=1)
        
        # Green Send PID button
        send_button = tk.Button(
            button_container, 
            text="Send PID Coefficients", 
            command=self.send_tab3_message,
            bg="#4CAF50",  # Green color
            fg="white",
            font=("Arial", 11, "bold"),
            width=20,
            height=2
        )
        send_button.grid(row=0, column=0, padx=(0, 5), sticky='ew')
        
        # Orange Write to Flash button
        write_flash_button = tk.Button(
            button_container, 
            text="Write to Flash", 
            command=self.send_write_flash_message,
            bg="#FF9800",  # Orange color
            fg="white",
            font=("Arial", 11, "bold"),
            width=20,
            height=2
        )
        write_flash_button.grid(row=0, column=1, padx=(5, 0), sticky='ew')
        
        # Right column - Current PID values
        current_frame = ttk.LabelFrame(controls_container, text="Current PID Values", padding="15")
        current_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        current_frame.columnconfigure(0, weight=1)  # Left PID column
        current_frame.columnconfigure(1, weight=1)  # Right PID column
        
        # Title for current values
        current_title = ttk.Label(current_frame, text="Current PID Values", font=("Arial", 14, "bold"))
        current_title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        # Green Get current PID button
        get_pid_button = tk.Button(
            current_frame, 
            text="Get Current PID", 
            command=self.get_current_pid,
            bg="#4CAF50",  # Green color
            fg="white",
            font=("Arial", 11, "bold"),
            width=20,
            height=2
        )
        get_pid_button.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Current Left PID values
        current_left_frame = ttk.LabelFrame(current_frame, text="Left Motor", padding="10")
        current_left_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        current_left_frame.columnconfigure(1, weight=1)
        
        ttk.Label(current_left_frame, text="Kp_L:", font=("Arial", 11)).grid(row=0, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(current_left_frame, text="Ki_L:", font=("Arial", 11)).grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(current_left_frame, text="Kd_L:", font=("Arial", 11)).grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        
        # Current Right PID values
        current_right_frame = ttk.LabelFrame(current_frame, text="Right Motor", padding="10")
        current_right_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        current_right_frame.columnconfigure(1, weight=1)
        
        ttk.Label(current_right_frame, text="Kp_R:", font=("Arial", 11)).grid(row=0, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(current_right_frame, text="Ki_R:", font=("Arial", 11)).grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        ttk.Label(current_right_frame, text="Kd_R:", font=("Arial", 11)).grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=6)
        
        # Current PID value displays
        self.current_pid_labels = []
        
        # Left current values
        for i in range(3):
            value = ttk.Label(current_left_frame, text="0.0", foreground="blue", font=("Arial", 11))
            value.grid(row=i, column=1, sticky=tk.W, pady=6)
            self.current_pid_labels.append(value)
        
        # Right current values
        for i in range(3):
            value = ttk.Label(current_right_frame, text="0.0", foreground="blue", font=("Arial", 11))
            value.grid(row=i, column=1, sticky=tk.W, pady=6)
            self.current_pid_labels.append(value)
        
        # Plots for tab 3 - full width
        self.create_tab3_plots(main_container)

    def send_write_flash_message(self):
        """Send message to write PID coefficients to flash"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            coefficients = []
            for entry in self.pid_entries:
                coefficients.append(float(entry.get().strip()))
            
            formatted_message = f"$4;{';'.join(map(str, coefficients))};#"
            
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message(f"Write PID to flash: {coefficients}")
            self.update_status("PID coefficients written to flash")
            
        except ValueError:
            self.update_status("Error: invalid number")
        except Exception as e:
            self.update_status(f"Error: {e}")

    def create_tab3_plots(self, parent_frame):
        """Create plots for tab 3: PID Tuning - Full Width"""
        try:
            plots_frame = ttk.LabelFrame(parent_frame, text="Wheel Velocity Plots", padding="5")
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            self.fig_tab3 = Figure(figsize=(12, 5))
            
            self.ax1_tab3 = self.fig_tab3.add_subplot(121)
            self.ax2_tab3 = self.fig_tab3.add_subplot(122)
            
            # Optimized plot settings for better performance
            self.ax1_tab3.set_title('Left Wheel Velocity (v_left)', fontsize=12)
            self.ax1_tab3.grid(True, alpha=0.3)
            line1, = self.ax1_tab3.plot([], [], 'g-', linewidth=1.5)
            self.plot_lines['v_left_tab3'] = line1
            
            self.ax2_tab3.set_title('Right Wheel Velocity (v_right)', fontsize=12)
            self.ax2_tab3.grid(True, alpha=0.3)
            line2, = self.ax2_tab3.plot([], [], 'm-', linewidth=1.5)
            self.plot_lines['v_right_tab3'] = line2
            
            self.fig_tab3.tight_layout()
            
            self.canvas_tab3 = FigureCanvasTkAgg(self.fig_tab3, plots_frame)
            self.canvas_tab3.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab3 plots: {e}")

    def get_current_pid(self):
        """Send request to get current PID values"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            formatted_message = "$5;#"
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message(f"Get PID: {formatted_message}")
            self.update_status("PID request sent")
        except Exception as e:
            self.update_status(f"Error: {e}")

    def update_current_pid_display(self):
        """Update the display of current PID values"""
        for i, label in enumerate(self.current_pid_labels):
            label.config(text=f"{self.current_pid_values[i]}")

    def create_plots_content(self):
        """Create content for plots tab"""
        try:
            # Create matplotlib figures
            self.create_all_plots()
        except Exception as e:
            print(f"Error creating plots: {e}")

    def create_all_plots(self):
        """Create all matplotlib plots - только скорости"""
        try:
            plots_container = ttk.Frame(self.plots_frame)
            plots_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.plots_frame.rowconfigure(0, weight=1)
            self.plots_frame.columnconfigure(0, weight=1)
            
            # Уменьшено количество графиков - только 4 графика скоростей
            self.fig_all = Figure(figsize=(14, 8))
            
            # Создаем 4 субплотов для данных скоростей (убраны x, y, theta)
            axes = [
                self.fig_all.add_subplot(221),  # v_linear_x(t)
                self.fig_all.add_subplot(222),  # v_angular_z(t)
                self.fig_all.add_subplot(223),  # velocity_left_wheel(t)
                self.fig_all.add_subplot(224),  # velocity_right_wheel(t)
            ]
            
            titles = [
                'Linear Velocity (v_linear_x)', 
                'Angular Velocity (v_angular_z)', 
                'Left Wheel Velocity (v_left)', 
                'Right Wheel Velocity (v_right)'
            ]
            colors = ['b-', 'r-', 'g-', 'm-']
            line_keys = ['v_linear_x_all', 'v_angular_z_all', 'v_left_all', 'v_right_all']
            
            for i, (ax, title, color, key) in enumerate(zip(axes, titles, colors, line_keys)):
                ax.set_title(title, fontsize=12)
                ax.grid(True, alpha=0.3)
                line, = ax.plot([], [], color, linewidth=1.5)
                self.plot_lines[key] = line
            
            self.fig_all.tight_layout()
            
            self.canvas_all = FigureCanvasTkAgg(self.fig_all, plots_container)
            self.canvas_all.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        except Exception as e:
            print(f"Error in create_all_plots: {e}")

    def update_plots_optimized(self, data_dict):
        """Оптимизированное обновление графиков"""
        current_time = time.time()
        if current_time - self.last_plot_update < self.plot_update_interval:
            return
            
        self.last_plot_update = current_time
        
        try:
            # Обновляем данные линий без полной перерисовки
            plot_data_mapping = {
                'v_linear_x': (self.time_data, self.v_linear_x_data),
                'v_angular_z': (self.time_data, self.v_angular_z_data),
                'v_left': (self.time_data, self.v_left_data),
                'v_right': (self.time_data, self.v_right_data),
                'v_left_tab3': (self.time_data, self.v_left_data),
                'v_right_tab3': (self.time_data, self.v_right_data),
                'v_linear_x_all': (self.time_data, self.v_linear_x_data),
                'v_angular_z_all': (self.time_data, self.v_angular_z_data),
                'v_left_all': (self.time_data, self.v_left_data),
                'v_right_all': (self.time_data, self.v_right_data),
            }
            
            for line_key, (x_data, y_data) in plot_data_mapping.items():
                if line_key in self.plot_lines and x_data and y_data:
                    self.plot_lines[line_key].set_data(list(x_data), list(y_data))
            
            # Автомасштабирование только для активных графиков
            current_tab = self.notebook.index(self.notebook.select())
            
            if current_tab == 0:  # Tab 1
                for ax in [self.ax1_tab1, self.ax2_tab1]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab1.draw_idle()
                
            elif current_tab == 1:  # Tab 2
                for ax in [self.ax1_tab2, self.ax2_tab2]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab2.draw_idle()
                
            elif current_tab == 2:  # Tab 3
                for ax in [self.ax1_tab3, self.ax2_tab3]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab3.draw_idle()
                
            elif current_tab == 3:  # All plots tab
                for ax in self.fig_all.axes:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_all.draw_idle()
                
        except Exception as e:
            print(f"Error updating plots: {e}")

    def clear_all_plots(self):
        """Clear all plot data"""
        try:
            # Очищаем все deque
            for data in [self.time_data, self.v_linear_x_data, self.v_angular_z_data, 
                        self.v_left_data, self.v_right_data]:
                data.clear()
            
            # Обновляем графики с пустыми данными
            for line in self.plot_lines.values():
                line.set_data([], [])
            
            # Перерисовываем все канвасы
            for canvas in [self.canvas_tab1, self.canvas_tab2, self.canvas_tab3, self.canvas_all]:
                if canvas:
                    canvas.draw_idle()
                    
        except Exception as e:
            print(f"Error clearing plots: {e}")

    def send_stop_message(self):
        """Send STOP message to stop the robot"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            formatted_message = "$1;0.0;0.0;#"
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message("STOP: $1;0.0;0.0;#")
            self.update_status("STOP command sent")
            
            # Visual feedback
            original_bg = self.stop_button.cget('bg')
            self.stop_button.config(bg='darkred')
            self.root.after(200, lambda: self.stop_button.config(bg=original_bg))
            
        except Exception as e:
            self.update_status(f"Error: {e}")

    def send_tab1_message(self):
        """Send message for tab 1: Velocity Control"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            linear = float(self.linear_entry.get().strip())
            angular = float(self.angular_entry.get().strip())
            formatted_message = f"$1;{linear};{angular};#"
            
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message(f"Vel: {linear}, {angular}")
            self.update_status("Velocity sent")
            
        except ValueError:
            self.update_status("Error: invalid number")
        except Exception as e:
            self.update_status(f"Error: {e}")

    def send_tab2_message(self):
        """Send message for tab 2: Wheel Control"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            left = float(self.left_entry.get().strip())
            right = float(self.right_entry.get().strip())
            formatted_message = f"$2;{left};{right};#"
            
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message(f"Wheel: $2;{left};{right};#")
            self.update_status("Wheel command sent")
            
        except ValueError:
            self.update_status("Error: invalid number")
        except Exception as e:
            self.update_status(f"Error: {e}")

    def send_tab3_message(self):
        """Send message for tab 3: PID Tuning"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            coefficients = []
            for entry in self.pid_entries:
                coefficients.append(float(entry.get().strip()))
            
            formatted_message = f"$3;{';'.join(map(str, coefficients))};#"
            
            self.ser.write(formatted_message.encode('utf-8'))
            self.log_sent_message(f"PID: {coefficients}")
            self.update_status("PID coefficients sent")
            
        except ValueError:
            self.update_status("Error: invalid number")
        except Exception as e:
            self.update_status(f"Error: {e}")

    def log_sent_message(self, message):
        """Быстрое логирование отправленных сообщений"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        self.sent_text.config(state=tk.NORMAL)
        self.sent_text.insert(tk.END, formatted_message)
        if float(self.sent_text.index('end-1c').split('.')[0]) > 1000:
            self.sent_text.delete(1.0, 2.0)
        self.sent_text.see(tk.END)
        self.sent_text.config(state=tk.DISABLED)

    def log_received_message(self, message):
        """Быстрое логирование принятых сообщений"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        self.received_text.config(state=tk.NORMAL)
        self.received_text.insert(tk.END, formatted_message)
        if float(self.received_text.index('end-1c').split('.')[0]) > 1000:
            self.received_text.delete(1.0, 2.0)
        self.received_text.see(tk.END)
        self.received_text.config(state=tk.DISABLED)

    def clear_sent_log(self):
        """Clear the sent messages log"""
        self.sent_text.config(state=tk.NORMAL)
        self.sent_text.delete(1.0, tk.END)
        self.sent_text.config(state=tk.DISABLED)

    def clear_received_log(self):
        """Clear the received messages log"""
        self.received_text.config(state=tk.NORMAL)
        self.received_text.delete(1.0, tk.END)
        self.received_text.config(state=tk.DISABLED)

    def update_status(self, message):
        """Update status label"""
        self.status_label.config(text=message)

    def start_threads(self):
        """Start background threads"""
        self.read_thread = threading.Thread(target=self.read_from_serial_optimized)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_from_serial_optimized(self):
        """Оптимизированное чтение из последовательного порта"""
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
                    
                    # Быстрая обработка буфера
                    while True:
                        start_index = serial_buffer.find('$')
                        if start_index == -1:
                            serial_buffer = ''
                            break
                        
                        end_index = serial_buffer.find('#', start_index + 1)
                        if end_index == -1:
                            serial_buffer = serial_buffer[start_index:]
                            break
                        
                        full_message = serial_buffer[start_index:end_index + 1]
                        
                        if full_message.startswith('$') and full_message.endswith('#'):
                            self.received_queue.put(full_message)
                            
                            # Быстрый парсинг для графиков
                            plot_data = self.parse_received_message_fast(full_message)
                            if plot_data:
                                self.add_data_point(plot_data)
                                self.update_plots_optimized(plot_data)
                        
                        serial_buffer = serial_buffer[end_index + 1:]
                        if not serial_buffer:
                            break

            except Exception as e:
                time.sleep(0.1)

    def parse_received_message_fast(self, message):
        """Быстрый парсинг сообщений без regex"""
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
                self.current_pid_values = [f"{float(x):.3f}" for x in parts[1:7]]
                self.update_current_pid_display()
                
        except (ValueError, IndexError):
            return None
            
        return None

    def add_data_point(self, data_dict):
        """Быстрое добавление точки данных"""
        current_time = time.time() - self.start_time
        
        self.time_data.append(current_time)
        self.v_linear_x_data.append(data_dict.get('v_linear_x', 0))
        self.v_angular_z_data.append(data_dict.get('v_angular_z', 0))
        self.v_left_data.append(data_dict.get('v_left', 0))
        self.v_right_data.append(data_dict.get('v_right', 0))

    def update_gui(self):
        """Оптимизированное обновление GUI"""
        processed = 0
        while not self.received_queue.empty() and processed < 10:
            try:
                message = self.received_queue.get_nowait()
                self.log_received_message(message)
                processed += 1
            except queue.Empty:
                break
        
        if self.running:
            self.root.after(150, self.update_gui)

    def on_closing(self):
        """Handle window closing"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

    def run(self):
        """Start the GUI application"""
        self.root.mainloop()


def main():
    serial_port = cfg.SERIAL_PORT
    baudrate = cfg.BAUDRATE
    
    app = SerialBridgeGUI(serial_port, baudrate)
    app.run()


if __name__ == '__main__':
    main()