#!/usr/bin/env python3

import serial
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import time
import re
from collections import deque

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from matplotlib import style
    style.use('ggplot')
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

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
        self.x_data = deque(maxlen=self.max_data_points)
        self.y_data = deque(maxlen=self.max_data_points)
        self.theta_data = deque(maxlen=self.max_data_points)
        self.v_linear_x_data = deque(maxlen=self.max_data_points)
        self.v_angular_z_data = deque(maxlen=self.max_data_points)
        self.v_left_data = deque(maxlen=self.max_data_points)
        self.v_right_data = deque(maxlen=self.max_data_points)
        
        self.start_time = time.time()
        self.last_plot_update = 0
        self.plot_update_interval = 0.3  # Обновлять графики раз в 300мс
        
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
                timeout=0.1  # Уменьшен timeout
            )
            print(f"Serial port {self.serial_port} opened at {self.baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening port {self.serial_port}: {e}")
            self.ser = None

    def create_widgets(self):
        """Create all GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="5")  # Уменьшен padding
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        # Create frames for each tab
        self.tab1_frame = ttk.Frame(self.notebook, padding="5")
        self.tab2_frame = ttk.Frame(self.notebook, padding="5")
        self.tab3_frame = ttk.Frame(self.notebook, padding="5")
        
        self.notebook.add(self.tab1_frame, text="Velocity Control")
        self.notebook.add(self.tab2_frame, text="Wheel Control") 
        self.notebook.add(self.tab3_frame, text="PID Tuning")
        
        # Add plots tab only if matplotlib is available
        if MATPLOTLIB_AVAILABLE:
            self.plots_frame = ttk.Frame(self.notebook, padding="5")
            self.notebook.add(self.plots_frame, text="All Plots")
            self.plots_frame.columnconfigure(0, weight=1)
            self.plots_frame.rowconfigure(0, weight=1)
            self.create_plots_content()
        
        # Configure each tab frame
        self.tab1_frame.columnconfigure(1, weight=1)
        self.tab2_frame.columnconfigure(1, weight=1)
        self.tab3_frame.columnconfigure(1, weight=1)
        
        # Create content for each tab
        self.create_tab1_content()
        self.create_tab2_content()
        self.create_tab3_content()
        
        # Log frames - simplified
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.columnconfigure(1, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Sent messages log
        sent_frame = ttk.LabelFrame(log_frame, text="Sent", padding="3")
        sent_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 3))
        sent_frame.columnconfigure(0, weight=1)
        sent_frame.rowconfigure(0, weight=1)
        
        self.sent_text = scrolledtext.ScrolledText(sent_frame, height=12, state=tk.DISABLED, font=("Courier", 9))
        self.sent_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Received messages log
        received_frame = ttk.LabelFrame(log_frame, text="Received", padding="3")
        received_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(3, 0))
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
        """Create content for tab 1: Velocity Control"""
        control_frame = ttk.Frame(self.tab1_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 5))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="Velocity Control", font=("Arial", 12, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Input fields
        linear_label = ttk.Label(control_frame, text="linear:")
        linear_label.grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=4)
        
        self.linear_entry = ttk.Entry(control_frame, width=15)
        self.linear_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=4)
        self.linear_entry.insert(0, "0.0")
        
        angular_label = ttk.Label(control_frame, text="angular:")
        angular_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=4)
        
        self.angular_entry = ttk.Entry(control_frame, width=15)
        self.angular_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=4)
        self.angular_entry.insert(0, "0.0")
        
        # Send button
        send_button = ttk.Button(control_frame, text="Send Velocity", command=self.send_tab1_message)
        send_button.grid(row=3, column=0, columnspan=2, pady=8)
        
        # Plots for tab 1
        if MATPLOTLIB_AVAILABLE:
            self.create_tab1_plots()

    def create_tab1_plots(self):
        """Create plots for tab 1: Velocity Control"""
        try:
            plots_frame = ttk.LabelFrame(self.tab1_frame, text="Velocity Plots", padding="5")
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.tab1_frame.rowconfigure(1, weight=1)
            self.tab1_frame.columnconfigure(0, weight=1)
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots
            self.fig_tab1 = Figure(figsize=(8, 3))  # Уменьшен размер
            
            # Create 2 subplots for velocity
            self.ax1_tab1 = self.fig_tab1.add_subplot(121)
            self.ax2_tab1 = self.fig_tab1.add_subplot(122)
            
            # Initial empty plots
            self.ax1_tab1.set_title('v_linear_x(t)')
            self.ax1_tab1.grid(True)
            line1, = self.ax1_tab1.plot([], [], 'b-')
            self.plot_lines['v_linear_x'] = line1
            
            self.ax2_tab1.set_title('v_angular_z(t)')
            self.ax2_tab1.grid(True)
            line2, = self.ax2_tab1.plot([], [], 'r-')
            self.plot_lines['v_angular_z'] = line2
            
            self.fig_tab1.tight_layout()
            
            # Create canvas
            self.canvas_tab1 = FigureCanvasTkAgg(self.fig_tab1, plots_frame)
            self.canvas_tab1.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab1 plots: {e}")

    def create_tab2_content(self):
        """Create content for tab 2: Wheel Control"""
        control_frame = ttk.Frame(self.tab2_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 5))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="Wheel Control", font=("Arial", 12, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Input fields
        left_label = ttk.Label(control_frame, text="left:")
        left_label.grid(row=1, column=0, sticky=tk.W, padx=(0, 5), pady=4)
        
        self.left_entry = ttk.Entry(control_frame, width=15)
        self.left_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=4)
        self.left_entry.insert(0, "0.0")
        
        right_label = ttk.Label(control_frame, text="right:")
        right_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 5), pady=4)
        
        self.right_entry = ttk.Entry(control_frame, width=15)
        self.right_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=4)
        self.right_entry.insert(0, "0.0")
        
        # Send button
        send_button = ttk.Button(control_frame, text="Send Wheel", command=self.send_tab2_message)
        send_button.grid(row=3, column=0, columnspan=2, pady=8)
        
        # Plots for tab 2
        if MATPLOTLIB_AVAILABLE:
            self.create_tab2_plots()

    def create_tab2_plots(self):
        """Create plots for tab 2: Wheel Control"""
        try:
            plots_frame = ttk.LabelFrame(self.tab2_frame, text="Wheel Plots", padding="5")
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.tab2_frame.rowconfigure(1, weight=1)
            self.tab2_frame.columnconfigure(0, weight=1)
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots
            self.fig_tab2 = Figure(figsize=(8, 3))
            
            # Create 2 subplots for wheel velocities
            self.ax1_tab2 = self.fig_tab2.add_subplot(121)
            self.ax2_tab2 = self.fig_tab2.add_subplot(122)
            
            # Initial empty plots
            self.ax1_tab2.set_title('v_left(t)')
            self.ax1_tab2.grid(True)
            line1, = self.ax1_tab2.plot([], [], 'g-')
            self.plot_lines['v_left'] = line1
            
            self.ax2_tab2.set_title('v_right(t)')
            self.ax2_tab2.grid(True)
            line2, = self.ax2_tab2.plot([], [], 'm-')
            self.plot_lines['v_right'] = line2
            
            self.fig_tab2.tight_layout()
            
            # Create canvas
            self.canvas_tab2 = FigureCanvasTkAgg(self.fig_tab2, plots_frame)
            self.canvas_tab2.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab2 plots: {e}")

    def create_tab3_content(self):
        """Create content for tab 3: PID Tuning"""
        main_tab3_frame = ttk.Frame(self.tab3_frame)
        main_tab3_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.tab3_frame.columnconfigure(0, weight=1)
        self.tab3_frame.rowconfigure(0, weight=1)
        main_tab3_frame.columnconfigure(0, weight=1)
        main_tab3_frame.columnconfigure(1, weight=1)
        main_tab3_frame.rowconfigure(0, weight=0)
        main_tab3_frame.rowconfigure(1, weight=1)
        
        # Left column - Controls
        control_frame = ttk.LabelFrame(main_tab3_frame, text="PID Setup", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="PID Tuning", font=("Arial", 12, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Compact PID input fields
        pid_labels = ['Kp_L:', 'Ki_L:', 'Kd_L:', 'Kp_R:', 'Ki_R:', 'Kd_R:']
        self.pid_entries = []
        
        for i, label_text in enumerate(pid_labels):
            label = ttk.Label(control_frame, text=label_text)
            label.grid(row=i+1, column=0, sticky=tk.W, padx=(0, 5), pady=2)
            
            entry = ttk.Entry(control_frame, width=12)
            entry.grid(row=i+1, column=1, sticky=(tk.W, tk.E), pady=2)
            entry.insert(0, "0.0")
            self.pid_entries.append(entry)
        
        # Send button
        send_button = ttk.Button(control_frame, text="Send PID", command=self.send_tab3_message)
        send_button.grid(row=7, column=0, columnspan=2, pady=8)
        
        # Right column - Current PID values
        current_pid_frame = ttk.LabelFrame(main_tab3_frame, text="Current PID", padding="10")
        current_pid_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        current_pid_frame.columnconfigure(1, weight=1)
        
        current_title = ttk.Label(current_pid_frame, text="Current Values", font=("Arial", 12, "bold"))
        current_title.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Get current PID button
        get_pid_button = ttk.Button(current_pid_frame, text="Get PID", command=self.get_current_pid)
        get_pid_button.grid(row=1, column=0, columnspan=2, pady=8)
        
        # Current PID value displays
        self.current_pid_labels = []
        
        for i, label_text in enumerate(pid_labels):
            label = ttk.Label(current_pid_frame, text=label_text)
            label.grid(row=i+2, column=0, sticky=tk.W, padx=(0, 5), pady=1)
            
            value = ttk.Label(current_pid_frame, text="0.0", foreground="blue")
            value.grid(row=i+2, column=1, sticky=tk.W, pady=1)
            self.current_pid_labels.append(value)
        
        # Plots for tab 3
        if MATPLOTLIB_AVAILABLE:
            plots_frame = ttk.LabelFrame(main_tab3_frame, text="Wheel Plots", padding="5")
            plots_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(5, 0))
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            self.create_tab3_plots(plots_frame)

    def create_tab3_plots(self, parent_frame):
        """Create plots for tab 3: PID Tuning"""
        try:
            self.fig_tab3 = Figure(figsize=(8, 2.5))
            
            self.ax1_tab3 = self.fig_tab3.add_subplot(121)
            self.ax2_tab3 = self.fig_tab3.add_subplot(122)
            
            self.ax1_tab3.set_title('v_left(t)')
            self.ax1_tab3.grid(True)
            line1, = self.ax1_tab3.plot([], [], 'g-')
            self.plot_lines['v_left_tab3'] = line1
            
            self.ax2_tab3.set_title('v_right(t)')
            self.ax2_tab3.grid(True)
            line2, = self.ax2_tab3.plot([], [], 'm-')
            self.plot_lines['v_right_tab3'] = line2
            
            self.fig_tab3.tight_layout()
            
            self.canvas_tab3 = FigureCanvasTkAgg(self.fig_tab3, parent_frame)
            self.canvas_tab3.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab3 plots: {e}")

    def get_current_pid(self):
        """Send request to get current PID values"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Error: serial port not open")
            return
            
        try:
            formatted_message = "$4;#"
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
        if not MATPLOTLIB_AVAILABLE:
            return
            
        try:
            # Create matplotlib figures
            self.create_all_plots()
        except Exception as e:
            print(f"Error creating plots: {e}")

    def create_all_plots(self):
        """Create all matplotlib plots"""
        try:
            plots_container = ttk.Frame(self.plots_frame)
            plots_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.plots_frame.rowconfigure(0, weight=1)
            self.plots_frame.columnconfigure(0, weight=1)
            
            self.fig_all = Figure(figsize=(12, 8))
            
            # Create 7 subplots for all data
            axes = [
                self.fig_all.add_subplot(421),  # x(t)
                self.fig_all.add_subplot(422),  # y(t)
                self.fig_all.add_subplot(423),  # theta(t)
                self.fig_all.add_subplot(424),  # v_linear_x(t)
                self.fig_all.add_subplot(425),  # v_angular_z(t)
                self.fig_all.add_subplot(426),  # velocity_left_wheel(t)
                self.fig_all.add_subplot(427),  # velocity_right_wheel(t)
            ]
            
            titles = ['x(t)', 'y(t)', 'theta(t)', 'v_linear_x(t)', 'v_angular_z(t)', 'v_left(t)', 'v_right(t)']
            colors = ['b-', 'r-', 'g-', 'c-', 'y-', 'm-', 'k-']
            line_keys = ['x', 'y', 'theta', 'v_linear_x', 'v_angular_z', 'v_left_all', 'v_right_all']
            
            for i, (ax, title, color, key) in enumerate(zip(axes, titles, colors, line_keys)):
                ax.set_title(title)
                ax.grid(True)
                line, = ax.plot([], [], color)
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
                'x': (self.time_data, self.x_data),
                'y': (self.time_data, self.y_data),
                'theta': (self.time_data, self.theta_data),
                'v_left_all': (self.time_data, self.v_left_data),
                'v_right_all': (self.time_data, self.v_right_data),
            }
            
            for line_key, (x_data, y_data) in plot_data_mapping.items():
                if line_key in self.plot_lines and x_data and y_data:
                    self.plot_lines[line_key].set_data(list(x_data), list(y_data))
            
            # Автомасштабирование только для активных графиков
            if self.notebook.index(self.notebook.select()) == 0:  # Tab 1
                for ax in [self.ax1_tab1, self.ax2_tab1]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab1.draw_idle()
                
            elif self.notebook.index(self.notebook.select()) == 1:  # Tab 2
                for ax in [self.ax1_tab2, self.ax2_tab2]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab2.draw_idle()
                
            elif self.notebook.index(self.notebook.select()) == 2:  # Tab 3
                for ax in [self.ax1_tab3, self.ax2_tab3]:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_tab3.draw_idle()
                
            elif self.notebook.index(self.notebook.select()) == 3:  # All plots tab
                for ax in self.fig_all.axes:
                    ax.relim()
                    ax.autoscale_view()
                self.canvas_all.draw_idle()
                
        except Exception as e:
            print(f"Error updating plots: {e}")

    def clear_all_plots(self):
        """Clear all plot data"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        try:
            # Очищаем все deque
            for data in [self.time_data, self.x_data, self.y_data, self.theta_data, 
                        self.v_linear_x_data, self.v_angular_z_data, self.v_left_data, self.v_right_data]:
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

    def parse_received_message_fast(self, message):
        """Быстрый парсинг сообщений без regex"""
        if not message.startswith('$') or not message.endswith(';#'):
            return None
            
        try:
            # Убираем $ и ;# и разбиваем по ;
            parts = message[1:-2].split(';')
            if not parts:
                return None
                
            msg_type = parts[0]
            
            if msg_type == '1' and len(parts) >= 8:  # Regular feedback
                return {
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'theta': float(parts[3]),
                    'v_linear_x': float(parts[4]),
                    'v_angular_z': float(parts[5]),
                    'v_left': float(parts[6]),
                    'v_right': float(parts[7])
                }
            elif msg_type == '2' and len(parts) >= 7:  # PID values
                self.current_pid_values = [f"{float(x):.3f}" for x in parts[1:7]]
                self.update_current_pid_display()
                
        except (ValueError, IndexError):
            return None
            
        return None

    def add_data_point(self, data_dict):
        """Быстрое добавление точки данных"""
        current_time = time.time() - self.start_time
        
        self.time_data.append(current_time)
        self.x_data.append(data_dict.get('x', 0))
        self.y_data.append(data_dict.get('y', 0))
        self.theta_data.append(data_dict.get('theta', 0))
        self.v_linear_x_data.append(data_dict.get('v_linear_x', 0))
        self.v_angular_z_data.append(data_dict.get('v_angular_z', 0))
        self.v_left_data.append(data_dict.get('v_left', 0))
        self.v_right_data.append(data_dict.get('v_right', 0))

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
            self.log_sent_message(f"Wheel: {left}, {right}")
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
        # Ограничиваем длину лога для производительности
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
        # Ограничиваем длину лога
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
        processing_interval = 0.05  # Обрабатывать не чаще чем раз в 50мс
        
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

    def update_gui(self):
        """Оптимизированное обновление GUI"""
        # Обрабатываем не более 10 сообщений за раз
        processed = 0
        while not self.received_queue.empty() and processed < 10:
            try:
                message = self.received_queue.get_nowait()
                self.log_received_message(message)
                processed += 1
            except queue.Empty:
                break
        
        # Следующее обновление через 150мс
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
    serial_port = '/dev/esp32'
    baudrate = 115200
    
    app = SerialBridgeGUI(serial_port, baudrate)
    app.run()

if __name__ == '__main__':
    main()