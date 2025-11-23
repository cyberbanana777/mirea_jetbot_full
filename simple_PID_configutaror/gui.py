#!/usr/bin/env python3

import serial
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext
import queue
import time
import re

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from matplotlib import style
    style.use('ggplot')
    MATPLOTLIB_AVAILABLE = True
except ImportError as e:
    print(f"Matplotlib import error: {e}")
    MATPLOTLIB_AVAILABLE = False
except Exception as e:
    print(f"Matplotlib setup error: {e}")
    MATPLOTLIB_AVAILABLE = False

class SerialBridgeGUI:
    """
    GUI for Serial Bridge with ESP32 - Multi-tab version with plots
    """

    def __init__(self, serial_port='/dev/esp32', baudrate=115200):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.running = True
        
        # Store last sent coefficients for tab 3
        self.last_coefficients = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
        self.current_pid_values = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
        
        # Data for plotting
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        self.v_linear_x_data = []
        self.v_angular_z_data = []
        self.v_left_data = []
        self.v_right_data = []
        self.start_time = time.time()
        self.max_data_points = 100  # Limit data points to prevent memory issues
        
        # Initialize serial connection
        self.init_serial()
        
        # Create GUI
        self.root = tk.Tk()
        self.root.title("ESP32 Serial Bridge - Multi Message Types with Plots")
        self.root.geometry("1920x1080")  # Full HD resolution
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
                timeout=0.5
            )
            print(f"Serial port {self.serial_port} opened at {self.baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening port {self.serial_port}: {e}")
            # Create a dummy serial object to avoid errors
            self.ser = None

    def create_widgets(self):
        """Create all GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights for Full HD
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)  # Logs area
        
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Create frames for each tab
        self.tab1_frame = ttk.Frame(self.notebook, padding="10")
        self.tab2_frame = ttk.Frame(self.notebook, padding="10")
        self.tab3_frame = ttk.Frame(self.notebook, padding="10")
        
        self.notebook.add(self.tab1_frame, text="Velocity Control")
        self.notebook.add(self.tab2_frame, text="Wheel Control") 
        self.notebook.add(self.tab3_frame, text="PID Tuning")
        
        # Add plots tab only if matplotlib is available
        if MATPLOTLIB_AVAILABLE:
            self.plots_frame = ttk.Frame(self.notebook, padding="10")
            self.notebook.add(self.plots_frame, text="All Plots")
            self.plots_frame.columnconfigure(0, weight=1)
            self.plots_frame.rowconfigure(0, weight=1)
            self.create_plots_content()
        else:
            print("Matplotlib not available - plots tab disabled")
        
        # Configure each tab frame
        self.tab1_frame.columnconfigure(1, weight=1)
        self.tab2_frame.columnconfigure(1, weight=1)
        self.tab3_frame.columnconfigure(1, weight=1)
        
        # Create content for each tab
        self.create_tab1_content()
        self.create_tab2_content()
        self.create_tab3_content()
        
        # Log frames - improved for Full HD
        log_frame = ttk.Frame(main_frame)
        log_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.columnconfigure(1, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Sent messages log
        sent_frame = ttk.LabelFrame(log_frame, text="Отправленные сообщения", padding="5")
        sent_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        sent_frame.columnconfigure(0, weight=1)
        sent_frame.rowconfigure(0, weight=1)
        
        self.sent_text = scrolledtext.ScrolledText(sent_frame, width=80, height=20, state=tk.DISABLED, font=("Courier", 10))
        self.sent_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Received messages log
        received_frame = ttk.LabelFrame(log_frame, text="Принятые сообщения", padding="5")
        received_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        received_frame.columnconfigure(0, weight=1)
        received_frame.rowconfigure(0, weight=1)
        
        self.received_text = scrolledtext.ScrolledText(received_frame, width=80, height=20, state=tk.DISABLED, font=("Courier", 10))
        self.received_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status bar
        status_frame = ttk.Frame(main_frame)
        status_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(10, 0))
        
        self.status_label = ttk.Label(status_frame, text="Готов к работе")
        self.status_label.pack(side=tk.LEFT)
        
        # Clear buttons
        clear_sent_button = ttk.Button(status_frame, text="Очистить отправленные", command=self.clear_sent_log)
        clear_sent_button.pack(side=tk.RIGHT, padx=(5, 0))
        
        clear_received_button = ttk.Button(status_frame, text="Очистить принятые", command=self.clear_received_log)
        clear_received_button.pack(side=tk.RIGHT)

    def create_tab1_content(self):
        """Create content for tab 1: Velocity Control"""
        # Control frame
        control_frame = ttk.Frame(self.tab1_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="Управление скоростями", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        desc = ttk.Label(control_frame, text="Формат: $1;linear_velocity;angular_velocity;#", font=("Arial", 10))
        desc.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Input fields
        linear_label = ttk.Label(control_frame, text="linear_velocity:", font=("Arial", 10))
        linear_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.linear_entry = ttk.Entry(control_frame, width=25, font=("Arial", 10))
        self.linear_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=8)
        self.linear_entry.insert(0, "0.0")
        
        angular_label = ttk.Label(control_frame, text="angular_velocity:", font=("Arial", 10))
        angular_label.grid(row=3, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.angular_entry = ttk.Entry(control_frame, width=25, font=("Arial", 10))
        self.angular_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=8)
        self.angular_entry.insert(0, "0.0")
        
        # Send button
        send_button = ttk.Button(control_frame, text="Отправить команду скоростей", command=self.send_tab1_message)
        send_button.grid(row=4, column=0, columnspan=2, pady=15)
        
        # Plots for tab 1
        if MATPLOTLIB_AVAILABLE:
            self.create_tab1_plots()

    def create_tab1_plots(self):
        """Create plots for tab 1: Velocity Control"""
        try:
            plots_frame = ttk.LabelFrame(self.tab1_frame, text="Графики скоростей", padding="10")
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.tab1_frame.rowconfigure(1, weight=1)
            self.tab1_frame.columnconfigure(0, weight=1)
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots
            self.fig_tab1 = Figure(figsize=(10, 5))
            
            # Create 2 subplots for velocity
            self.ax1_tab1 = self.fig_tab1.add_subplot(121)  # v_linear_x(t)
            self.ax2_tab1 = self.fig_tab1.add_subplot(122)  # v_angular_z(t)
            
            # Set titles and labels
            self.ax1_tab1.set_title('v_linear_x(t)', fontsize=12)
            self.ax1_tab1.set_ylabel('v_linear_x', fontsize=10)
            self.ax1_tab1.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab1.grid(True)
            
            self.ax2_tab1.set_title('v_angular_z(t)', fontsize=12)
            self.ax2_tab1.set_ylabel('v_angular_z', fontsize=10)
            self.ax2_tab1.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab1.grid(True)
            
            self.fig_tab1.tight_layout()
            
            # Create canvas
            self.canvas_tab1 = FigureCanvasTkAgg(self.fig_tab1, plots_frame)
            self.canvas_tab1.draw()
            self.canvas_tab1.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab1 plots: {e}")

    def create_tab2_content(self):
        """Create content for tab 2: Wheel Control"""
        # Control frame
        control_frame = ttk.Frame(self.tab2_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="Управление колесами", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        desc = ttk.Label(control_frame, text="Формат: $2;velocity_left_wheel;velocity_right_wheel;#", font=("Arial", 10))
        desc.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Input fields
        left_label = ttk.Label(control_frame, text="velocity_left_wheel:", font=("Arial", 10))
        left_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.left_entry = ttk.Entry(control_frame, width=25, font=("Arial", 10))
        self.left_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=8)
        self.left_entry.insert(0, "0.0")
        
        right_label = ttk.Label(control_frame, text="velocity_right_wheel:", font=("Arial", 10))
        right_label.grid(row=3, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.right_entry = ttk.Entry(control_frame, width=25, font=("Arial", 10))
        self.right_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=8)
        self.right_entry.insert(0, "0.0")
        
        # Send button
        send_button = ttk.Button(control_frame, text="Отправить команду колес", command=self.send_tab2_message)
        send_button.grid(row=4, column=0, columnspan=2, pady=15)
        
        # Plots for tab 2
        if MATPLOTLIB_AVAILABLE:
            self.create_tab2_plots()

    def create_tab2_plots(self):
        """Create plots for tab 2: Wheel Control"""
        try:
            plots_frame = ttk.LabelFrame(self.tab2_frame, text="Графики скоростей колес", padding="10")
            plots_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.tab2_frame.rowconfigure(1, weight=1)
            self.tab2_frame.columnconfigure(0, weight=1)
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            
            # Create figure with subplots
            self.fig_tab2 = Figure(figsize=(10, 5))
            
            # Create 2 subplots for wheel velocities
            self.ax1_tab2 = self.fig_tab2.add_subplot(121)  # velocity_left_wheel(t)
            self.ax2_tab2 = self.fig_tab2.add_subplot(122)  # velocity_right_wheel(t)
            
            # Set titles and labels
            self.ax1_tab2.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax1_tab2.set_ylabel('v_left', fontsize=10)
            self.ax1_tab2.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab2.grid(True)
            
            self.ax2_tab2.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax2_tab2.set_ylabel('v_right', fontsize=10)
            self.ax2_tab2.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab2.grid(True)
            
            self.fig_tab2.tight_layout()
            
            # Create canvas
            self.canvas_tab2 = FigureCanvasTkAgg(self.fig_tab2, plots_frame)
            self.canvas_tab2.draw()
            self.canvas_tab2.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab2 plots: {e}")

    def create_tab3_content(self):
        """Create content for tab 3: PID Tuning"""
        # Main frame with two columns
        main_tab3_frame = ttk.Frame(self.tab3_frame)
        main_tab3_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.tab3_frame.columnconfigure(0, weight=1)
        self.tab3_frame.rowconfigure(0, weight=1)
        main_tab3_frame.columnconfigure(0, weight=1)
        main_tab3_frame.columnconfigure(1, weight=1)
        main_tab3_frame.rowconfigure(0, weight=0)  # Top part - fixed height
        main_tab3_frame.rowconfigure(1, weight=1)  # Plots - expandable
        
        # Left column - Controls
        control_frame = ttk.LabelFrame(main_tab3_frame, text="Настройка PID коэффициентов", padding="15")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        title = ttk.Label(control_frame, text="Настройка PID коэффициентов", font=("Arial", 14, "bold"))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        desc = ttk.Label(control_frame, text="Формат: $3;Kp_L;Ki_L;Kd_L;Kp_R;Ki_R;Kd_R;#", font=("Arial", 10))
        desc.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Input fields - в порядке PID
        kp_l_label = ttk.Label(control_frame, text="Kp_L:", font=("Arial", 10))
        kp_l_label.grid(row=2, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.kp_l_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.kp_l_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=8)
        self.kp_l_entry.insert(0, "0.0")
        
        ki_l_label = ttk.Label(control_frame, text="Ki_L:", font=("Arial", 10))
        ki_l_label.grid(row=3, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.ki_l_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.ki_l_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=8)
        self.ki_l_entry.insert(0, "0.0")
        
        kd_l_label = ttk.Label(control_frame, text="Kd_L:", font=("Arial", 10))
        kd_l_label.grid(row=4, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.kd_l_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.kd_l_entry.grid(row=4, column=1, sticky=(tk.W, tk.E), pady=8)
        self.kd_l_entry.insert(0, "0.0")
        
        kp_r_label = ttk.Label(control_frame, text="Kp_R:", font=("Arial", 10))
        kp_r_label.grid(row=5, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.kp_r_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.kp_r_entry.grid(row=5, column=1, sticky=(tk.W, tk.E), pady=8)
        self.kp_r_entry.insert(0, "0.0")
        
        ki_r_label = ttk.Label(control_frame, text="Ki_R:", font=("Arial", 10))
        ki_r_label.grid(row=6, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.ki_r_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.ki_r_entry.grid(row=6, column=1, sticky=(tk.W, tk.E), pady=8)
        self.ki_r_entry.insert(0, "0.0")
        
        kd_r_label = ttk.Label(control_frame, text="Kd_R:", font=("Arial", 10))
        kd_r_label.grid(row=7, column=0, sticky=tk.W, padx=(0, 10), pady=8)
        
        self.kd_r_entry = ttk.Entry(control_frame, width=20, font=("Arial", 10))
        self.kd_r_entry.grid(row=7, column=1, sticky=(tk.W, tk.E), pady=8)
        self.kd_r_entry.insert(0, "0.0")
        
        # Send button
        send_button = ttk.Button(control_frame, text="Отправить PID коэффициенты", command=self.send_tab3_message)
        send_button.grid(row=8, column=0, columnspan=2, pady=15)
        
        # Last sent coefficients display - переделано в формат PID для каждого регулятора
        last_coef_frame = ttk.LabelFrame(control_frame, text="Последние отправленные коэффициенты", padding="10")
        last_coef_frame.grid(row=9, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(20, 0))
        last_coef_frame.columnconfigure(0, weight=1)
        last_coef_frame.columnconfigure(1, weight=1)
        last_coef_frame.columnconfigure(2, weight=1)
        last_coef_frame.columnconfigure(3, weight=1)
        last_coef_frame.columnconfigure(4, weight=1)
        last_coef_frame.columnconfigure(5, weight=1)
        
        # Заголовки для левого регулятора
        left_reg_label = ttk.Label(last_coef_frame, text="Левый регулятор:", font=("Arial", 10, "bold"))
        left_reg_label.grid(row=0, column=0, columnspan=3, sticky=tk.W, padx=5, pady=5)
        
        # Заголовки для правого регулятора
        right_reg_label = ttk.Label(last_coef_frame, text="Правый регулятор:", font=("Arial", 10, "bold"))
        right_reg_label.grid(row=0, column=3, columnspan=3, sticky=tk.W, padx=5, pady=5)
        
        # Заголовки PID
        p_label = ttk.Label(last_coef_frame, text="P", font=("Arial", 9))
        p_label.grid(row=1, column=0, padx=5, pady=2)
        
        i_label = ttk.Label(last_coef_frame, text="I", font=("Arial", 9))
        i_label.grid(row=1, column=1, padx=5, pady=2)
        
        d_label = ttk.Label(last_coef_frame, text="D", font=("Arial", 9))
        d_label.grid(row=1, column=2, padx=5, pady=2)
        
        p_label_r = ttk.Label(last_coef_frame, text="P", font=("Arial", 9))
        p_label_r.grid(row=1, column=3, padx=5, pady=2)
        
        i_label_r = ttk.Label(last_coef_frame, text="I", font=("Arial", 9))
        i_label_r.grid(row=1, column=4, padx=5, pady=2)
        
        d_label_r = ttk.Label(last_coef_frame, text="D", font=("Arial", 9))
        d_label_r.grid(row=1, column=5, padx=5, pady=2)
        
        # Create display labels for last coefficients в порядке PID
        self.last_coef_labels = []
        # Левый регулятор P, I, D
        kp_l_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        kp_l_display.grid(row=2, column=0, padx=5, pady=2)
        self.last_coef_labels.append(kp_l_display)
        
        ki_l_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        ki_l_display.grid(row=2, column=1, padx=5, pady=2)
        self.last_coef_labels.append(ki_l_display)
        
        kd_l_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        kd_l_display.grid(row=2, column=2, padx=5, pady=2)
        self.last_coef_labels.append(kd_l_display)
        
        # Правый регулятор P, I, D
        kp_r_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        kp_r_display.grid(row=2, column=3, padx=5, pady=2)
        self.last_coef_labels.append(kp_r_display)
        
        ki_r_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        ki_r_display.grid(row=2, column=4, padx=5, pady=2)
        self.last_coef_labels.append(ki_r_display)
        
        kd_r_display = ttk.Label(last_coef_frame, text="0.0", font=("Arial", 9))
        kd_r_display.grid(row=2, column=5, padx=5, pady=2)
        self.last_coef_labels.append(kd_r_display)
        
        # Right column - Current PID values
        current_pid_frame = ttk.LabelFrame(main_tab3_frame, text="Текущие значения PID регуляторов", padding="15")
        current_pid_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        current_pid_frame.columnconfigure(1, weight=1)
        
        current_title = ttk.Label(current_pid_frame, text="Текущие значения PID", font=("Arial", 14, "bold"))
        current_title.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        current_desc = ttk.Label(current_pid_frame, text="Полученные от ESP32 (msg_type=2)", font=("Arial", 10))
        current_desc.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Get current PID button - перемещена в правую колонку
        get_pid_button = ttk.Button(current_pid_frame, text="Получить текущие значения регуляторов", command=self.get_current_pid)
        get_pid_button.grid(row=2, column=0, columnspan=2, pady=15)
        
        # Current PID value displays в порядке PID
        self.current_pid_labels = []
        
        # Левый регулятор
        left_reg_label = ttk.Label(current_pid_frame, text="Левый регулятор:", font=("Arial", 10, "bold"))
        left_reg_label.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(10, 5))
        
        kp_l_label = ttk.Label(current_pid_frame, text="Kp_L:", font=("Arial", 10))
        kp_l_label.grid(row=4, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        kp_l_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        kp_l_value.grid(row=4, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(kp_l_value)
        
        ki_l_label = ttk.Label(current_pid_frame, text="Ki_L:", font=("Arial", 10))
        ki_l_label.grid(row=5, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        ki_l_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        ki_l_value.grid(row=5, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(ki_l_value)
        
        kd_l_label = ttk.Label(current_pid_frame, text="Kd_L:", font=("Arial", 10))
        kd_l_label.grid(row=6, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        kd_l_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        kd_l_value.grid(row=6, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(kd_l_value)
        
        # Правый регулятор
        right_reg_label = ttk.Label(current_pid_frame, text="Правый регулятор:", font=("Arial", 10, "bold"))
        right_reg_label.grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=(10, 5))
        
        kp_r_label = ttk.Label(current_pid_frame, text="Kp_R:", font=("Arial", 10))
        kp_r_label.grid(row=8, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        kp_r_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        kp_r_value.grid(row=8, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(kp_r_value)
        
        ki_r_label = ttk.Label(current_pid_frame, text="Ki_R:", font=("Arial", 10))
        ki_r_label.grid(row=9, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        ki_r_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        ki_r_value.grid(row=9, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(ki_r_value)
        
        kd_r_label = ttk.Label(current_pid_frame, text="Kd_R:", font=("Arial", 10))
        kd_r_label.grid(row=10, column=0, sticky=tk.W, padx=(0, 10), pady=5)
        
        kd_r_value = ttk.Label(current_pid_frame, text="0.0", font=("Arial", 10, "bold"), foreground="blue")
        kd_r_value.grid(row=10, column=1, sticky=tk.W, pady=5)
        self.current_pid_labels.append(kd_r_value)
        
        # Plots for tab 3
        if MATPLOTLIB_AVAILABLE:
            plots_frame = ttk.LabelFrame(main_tab3_frame, text="Графики скоростей колес", padding="10")
            plots_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
            plots_frame.columnconfigure(0, weight=1)
            plots_frame.rowconfigure(0, weight=1)
            self.create_tab3_plots(plots_frame)

    def create_tab3_plots(self, parent_frame):
        """Create plots for tab 3: PID Tuning"""
        try:
            # Create figure with subplots - уменьшен размер для экономии места
            self.fig_tab3 = Figure(figsize=(10, 3))
            
            # Create 2 subplots for wheel velocities
            self.ax1_tab3 = self.fig_tab3.add_subplot(121)  # velocity_left_wheel(t)
            self.ax2_tab3 = self.fig_tab3.add_subplot(122)  # velocity_right_wheel(t)
            
            # Set titles and labels
            self.ax1_tab3.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax1_tab3.set_ylabel('v_left', fontsize=10)
            self.ax1_tab3.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab3.grid(True)
            
            self.ax2_tab3.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax2_tab3.set_ylabel('v_right', fontsize=10)
            self.ax2_tab3.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab3.grid(True)
            
            self.fig_tab3.tight_layout()
            
            # Create canvas
            self.canvas_tab3 = FigureCanvasTkAgg(self.fig_tab3, parent_frame)
            self.canvas_tab3.draw()
            self.canvas_tab3.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        except Exception as e:
            print(f"Error creating tab3 plots: {e}")

    def get_current_pid(self):
        """Send request to get current PID values"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Ошибка: последовательный порт не открыт")
            return
            
        try:
            # Format message as $4;#
            formatted_message = "$4;#"
            
            # Send to ESP32
            self.ser.write(formatted_message.encode('utf-8'))
            
            # Log the sent message
            self.log_sent_message(f"Get PID Request: {formatted_message}")
            
            self.update_status("Запрос текущих PID значений отправлен")
            
        except Exception as e:
            self.log_sent_message(f"Ошибка отправки: {e}")
            self.update_status(f"Ошибка: {e}")

    def update_current_pid_display(self):
        """Update the display of current PID values"""
        for i, label in enumerate(self.current_pid_labels):
            label.config(text=f"{self.current_pid_values[i]}")

    def create_plots_content(self):
        """Create content for plots tab"""
        if not MATPLOTLIB_AVAILABLE:
            error_label = ttk.Label(self.plots_frame, text="Графики недоступны: Matplotlib не установлен или произошла ошибка", foreground="red")
            error_label.grid(row=0, column=0, pady=20)
            return
            
        try:
            title = ttk.Label(self.plots_frame, text="Все графики данных с ESP32", font=("Arial", 14, "bold"))
            title.grid(row=0, column=0, pady=(0, 10))
            
            # Create matplotlib figures
            self.create_all_plots()
            
            # Clear plots button
            clear_plots_button = ttk.Button(self.plots_frame, text="Очистить все графики", command=self.clear_all_plots)
            clear_plots_button.grid(row=2, column=0, pady=10)
        except Exception as e:
            error_label = ttk.Label(self.plots_frame, text=f"Ошибка создания графиков: {e}", foreground="red")
            error_label.grid(row=0, column=0, pady=20)
            print(f"Error creating plots: {e}")

    def create_all_plots(self):
        """Create all matplotlib plots"""
        try:
            # Create a frame for plots
            plots_container = ttk.Frame(self.plots_frame)
            plots_container.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
            self.plots_frame.rowconfigure(1, weight=1)
            self.plots_frame.columnconfigure(0, weight=1)
            
            # Create figure with subplots - larger for Full HD
            self.fig_all = Figure(figsize=(14, 10))
            
            # Create 7 subplots for all data
            self.ax1_all = self.fig_all.add_subplot(421)  # x(t)
            self.ax2_all = self.fig_all.add_subplot(422)  # y(t)
            self.ax3_all = self.fig_all.add_subplot(423)  # theta(t)
            self.ax4_all = self.fig_all.add_subplot(424)  # v_linear_x(t)
            self.ax5_all = self.fig_all.add_subplot(425)  # v_angular_z(t)
            self.ax6_all = self.fig_all.add_subplot(426)  # velocity_left_wheel(t)
            self.ax7_all = self.fig_all.add_subplot(427)  # velocity_right_wheel(t)
            
            # Set titles and labels with larger fonts
            self.ax1_all.set_title('x(t)', fontsize=12)
            self.ax1_all.set_ylabel('x', fontsize=10)
            self.ax1_all.grid(True)
            
            self.ax2_all.set_title('y(t)', fontsize=12)
            self.ax2_all.set_ylabel('y', fontsize=10)
            self.ax2_all.grid(True)
            
            self.ax3_all.set_title('theta(t)', fontsize=12)
            self.ax3_all.set_ylabel('theta', fontsize=10)
            self.ax3_all.grid(True)
            
            self.ax4_all.set_title('v_linear_x(t)', fontsize=12)
            self.ax4_all.set_ylabel('v_linear_x', fontsize=10)
            self.ax4_all.grid(True)
            
            self.ax5_all.set_title('v_angular_z(t)', fontsize=12)
            self.ax5_all.set_ylabel('v_angular_z', fontsize=10)
            self.ax5_all.grid(True)
            
            self.ax6_all.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax6_all.set_ylabel('v_left', fontsize=10)
            self.ax6_all.grid(True)
            
            self.ax7_all.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax7_all.set_ylabel('v_right', fontsize=10)
            self.ax7_all.set_xlabel('Time (s)', fontsize=10)
            self.ax7_all.grid(True)
            
            self.fig_all.tight_layout()
            
            # Create canvas
            self.canvas_all = FigureCanvasTkAgg(self.fig_all, plots_container)
            self.canvas_all.draw()
            self.canvas_all.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        except Exception as e:
            print(f"Error in create_all_plots: {e}")
            raise

    def update_all_plots(self, data_dict):
        """Update all plots with new data"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        try:
            current_time = time.time() - self.start_time
            
            # Add new data points
            self.time_data.append(current_time)
            self.x_data.append(data_dict.get('x', 0))
            self.y_data.append(data_dict.get('y', 0))
            self.theta_data.append(data_dict.get('theta', 0))
            self.v_linear_x_data.append(data_dict.get('v_linear_x', 0))
            self.v_angular_z_data.append(data_dict.get('v_angular_z', 0))
            self.v_left_data.append(data_dict.get('v_left', 0))
            self.v_right_data.append(data_dict.get('v_right', 0))
            
            # Limit data points
            if len(self.time_data) > self.max_data_points:
                self.time_data.pop(0)
                self.x_data.pop(0)
                self.y_data.pop(0)
                self.theta_data.pop(0)
                self.v_linear_x_data.pop(0)
                self.v_angular_z_data.pop(0)
                self.v_left_data.pop(0)
                self.v_right_data.pop(0)
            
            # Update tab1 plots (Velocity Control)
            self.ax1_tab1.clear()
            self.ax1_tab1.plot(self.time_data, self.v_linear_x_data, 'b-')
            self.ax1_tab1.set_title('v_linear_x(t)', fontsize=12)
            self.ax1_tab1.set_ylabel('v_linear_x', fontsize=10)
            self.ax1_tab1.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab1.grid(True)
            
            self.ax2_tab1.clear()
            self.ax2_tab1.plot(self.time_data, self.v_angular_z_data, 'r-')
            self.ax2_tab1.set_title('v_angular_z(t)', fontsize=12)
            self.ax2_tab1.set_ylabel('v_angular_z', fontsize=10)
            self.ax2_tab1.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab1.grid(True)
            
            self.fig_tab1.tight_layout()
            self.canvas_tab1.draw()
            
            # Update tab2 plots (Wheel Control)
            self.ax1_tab2.clear()
            self.ax1_tab2.plot(self.time_data, self.v_left_data, 'g-')
            self.ax1_tab2.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax1_tab2.set_ylabel('v_left', fontsize=10)
            self.ax1_tab2.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab2.grid(True)
            
            self.ax2_tab2.clear()
            self.ax2_tab2.plot(self.time_data, self.v_right_data, 'm-')
            self.ax2_tab2.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax2_tab2.set_ylabel('v_right', fontsize=10)
            self.ax2_tab2.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab2.grid(True)
            
            self.fig_tab2.tight_layout()
            self.canvas_tab2.draw()
            
            # Update tab3 plots (PID Tuning)
            self.ax1_tab3.clear()
            self.ax1_tab3.plot(self.time_data, self.v_left_data, 'g-')
            self.ax1_tab3.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax1_tab3.set_ylabel('v_left', fontsize=10)
            self.ax1_tab3.set_xlabel('Time (s)', fontsize=10)
            self.ax1_tab3.grid(True)
            
            self.ax2_tab3.clear()
            self.ax2_tab3.plot(self.time_data, self.v_right_data, 'm-')
            self.ax2_tab3.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax2_tab3.set_ylabel('v_right', fontsize=10)
            self.ax2_tab3.set_xlabel('Time (s)', fontsize=10)
            self.ax2_tab3.grid(True)
            
            self.fig_tab3.tight_layout()
            self.canvas_tab3.draw()
            
            # Update all plots tab
            self.ax1_all.clear()
            self.ax1_all.plot(self.time_data, self.x_data, 'b-')
            self.ax1_all.set_title('x(t)', fontsize=12)
            self.ax1_all.set_ylabel('x', fontsize=10)
            self.ax1_all.grid(True)
            
            self.ax2_all.clear()
            self.ax2_all.plot(self.time_data, self.y_data, 'r-')
            self.ax2_all.set_title('y(t)', fontsize=12)
            self.ax2_all.set_ylabel('y', fontsize=10)
            self.ax2_all.grid(True)
            
            self.ax3_all.clear()
            self.ax3_all.plot(self.time_data, self.theta_data, 'g-')
            self.ax3_all.set_title('theta(t)', fontsize=12)
            self.ax3_all.set_ylabel('theta', fontsize=10)
            self.ax3_all.grid(True)
            
            self.ax4_all.clear()
            self.ax4_all.plot(self.time_data, self.v_linear_x_data, 'c-')
            self.ax4_all.set_title('v_linear_x(t)', fontsize=12)
            self.ax4_all.set_ylabel('v_linear_x', fontsize=10)
            self.ax4_all.grid(True)
            
            self.ax5_all.clear()
            self.ax5_all.plot(self.time_data, self.v_angular_z_data, 'y-')
            self.ax5_all.set_title('v_angular_z(t)', fontsize=12)
            self.ax5_all.set_ylabel('v_angular_z', fontsize=10)
            self.ax5_all.grid(True)
            
            self.ax6_all.clear()
            self.ax6_all.plot(self.time_data, self.v_left_data, 'm-')
            self.ax6_all.set_title('velocity_left_wheel(t)', fontsize=12)
            self.ax6_all.set_ylabel('v_left', fontsize=10)
            self.ax6_all.grid(True)
            
            self.ax7_all.clear()
            self.ax7_all.plot(self.time_data, self.v_right_data, 'k-')
            self.ax7_all.set_title('velocity_right_wheel(t)', fontsize=12)
            self.ax7_all.set_ylabel('v_right', fontsize=10)
            self.ax7_all.set_xlabel('Time (s)', fontsize=10)
            self.ax7_all.grid(True)
            
            self.fig_all.tight_layout()
            self.canvas_all.draw()
            
        except Exception as e:
            print(f"Error updating plots: {e}")

    def clear_all_plots(self):
        """Clear all plot data"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        try:
            self.time_data = []
            self.x_data = []
            self.y_data = []
            self.theta_data = []
            self.v_linear_x_data = []
            self.v_angular_z_data = []
            self.v_left_data = []
            self.v_right_data = []
            
            # Redraw empty plots
            self.update_all_plots({
                'x': 0, 'y': 0, 'theta': 0, 
                'v_linear_x': 0, 'v_angular_z': 0,
                'v_left': 0, 'v_right': 0
            })
            
        except Exception as e:
            print(f"Error clearing plots: {e}")

    def parse_received_message(self, message):
        """Parse received message to extract data for plots or PID values"""
        # Parse message format: $msg_type;x_pos;y_pos;theta;v_linear_x;v_angular_z;velocity_left_wheel;velocity_right_wheel;#
        # Or PID values: $2;Kp_L;Ki_L;Kd_L;Kp_R;Ki_R;Kd_R;#
        numbers = re.findall(r"[-+]?\d*\.\d+|\d+", message)
        
        if len(numbers) >= 8:  # msg_type + 7 data fields (feedback)
            try:
                msg_type = int(numbers[0])
                if msg_type == 1:  # Regular feedback
                    data_dict = {
                        'x': float(numbers[1]),
                        'y': float(numbers[2]),
                        'theta': float(numbers[3]),
                        'v_linear_x': float(numbers[4]),
                        'v_angular_z': float(numbers[5]),
                        'v_left': float(numbers[6]),
                        'v_right': float(numbers[7])
                    }
                    return data_dict
                elif msg_type == 2:  # PID values feedback
                    # Update current PID values в порядке PID
                    self.current_pid_values = [
                        f"{float(numbers[1]):.3f}",  # Kp_L
                        f"{float(numbers[2]):.3f}",  # Ki_L
                        f"{float(numbers[3]):.3f}",  # Kd_L
                        f"{float(numbers[4]):.3f}",  # Kp_R
                        f"{float(numbers[5]):.3f}",  # Ki_R
                        f"{float(numbers[6]):.3f}"   # Kd_R
                    ]
                    self.update_current_pid_display()
                    return None
            except (ValueError, IndexError) as e:
                print(f"Error parsing message: {e}")
        
        return None

    def send_tab1_message(self):
        """Send message for tab 1: Velocity Control"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Ошибка: последовательный порт не открыт")
            return
            
        try:
            # Convert to float to ensure numeric values
            linear = float(self.linear_entry.get().strip())
            angular = float(self.angular_entry.get().strip())
            
            # Format message as $1;linear_velocity;angular_velocity;#
            formatted_message = f"$1;{linear};{angular};#"
            
            # Send to ESP32
            self.ser.write(formatted_message.encode('utf-8'))
            
            # Log the sent message
            self.log_sent_message(f"Velocity Command: {formatted_message}")
            
            self.update_status("Команда скоростей отправлена")
            
        except ValueError:
            self.log_sent_message("Ошибка: нечисловое значение в поле linear_velocity или angular_velocity")
            self.update_status("Ошибка: введите числовые значения")
        except Exception as e:
            self.log_sent_message(f"Ошибка отправки: {e}")
            self.update_status(f"Ошибка: {e}")

    def send_tab2_message(self):
        """Send message for tab 2: Wheel Control"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Ошибка: последовательный порт не открыт")
            return
            
        try:
            # Convert to float to ensure numeric values
            left = float(self.left_entry.get().strip())
            right = float(self.right_entry.get().strip())
            
            # Format message as $2;velocity_left_wheel;velocity_right_wheel;#
            formatted_message = f"$2;{left};{right};#"
            
            # Send to ESP32
            self.ser.write(formatted_message.encode('utf-8'))
            
            # Log the sent message
            self.log_sent_message(f"Wheel Command: {formatted_message}")
            
            self.update_status("Команда колес отправлена")
            
        except ValueError:
            self.log_sent_message("Ошибка: нечисловое значение в поле velocity_left_wheel или velocity_right_wheel")
            self.update_status("Ошибка: введите числовые значения")
        except Exception as e:
            self.log_sent_message(f"Ошибка отправки: {e}")
            self.update_status(f"Ошибка: {e}")

    def send_tab3_message(self):
        """Send message for tab 3: PID Tuning"""
        if not self.ser or not self.ser.is_open:
            self.update_status("Ошибка: последовательный порт не открыт")
            return
            
        try:
            # Collect all coefficient values and convert to float в порядке PID
            kp_l = float(self.kp_l_entry.get().strip())
            ki_l = float(self.ki_l_entry.get().strip())
            kd_l = float(self.kd_l_entry.get().strip())
            kp_r = float(self.kp_r_entry.get().strip())
            ki_r = float(self.ki_r_entry.get().strip())
            kd_r = float(self.kd_r_entry.get().strip())
            
            # Update last coefficients в порядке PID
            self.last_coefficients = [f"{kp_l:.3f}", f"{ki_l:.3f}", f"{kd_l:.3f}", f"{kp_r:.3f}", f"{ki_r:.3f}", f"{kd_r:.3f}"]
            self.update_coefficient_display()
            
            # Format message as $3;Kp_L;Ki_L;Kd_L;Kp_R;Ki_R;Kd_R;#
            formatted_message = f"$3;{kp_l};{ki_l};{kd_l};{kp_r};{ki_r};{kd_r};#"
            
            # Send to ESP32
            self.ser.write(formatted_message.encode('utf-8'))
            
            # Log the sent message
            self.log_sent_message(f"PID Coefficients: {formatted_message}")
            
            self.update_status("PID коэффициенты отправлены")
            
        except ValueError:
            self.log_sent_message("Ошибка: нечисловое значение в одном из полей коэффициентов")
            self.update_status("Ошибка: введите числовые значения")
        except Exception as e:
            self.log_sent_message(f"Ошибка отправки: {e}")
            self.update_status(f"Ошибка: {e}")

    def update_coefficient_display(self):
        """Update the display of last sent coefficients in tab 3"""
        for i, label in enumerate(self.last_coef_labels):
            label.config(text=f"{self.last_coefficients[i]}")

    def log_sent_message(self, message):
        """Add message to sent messages log"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        self.sent_text.config(state=tk.NORMAL)
        self.sent_text.insert(tk.END, formatted_message)
        self.sent_text.see(tk.END)
        self.sent_text.config(state=tk.DISABLED)

    def log_received_message(self, message):
        """Add message to received messages log"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        self.received_text.config(state=tk.NORMAL)
        self.received_text.insert(tk.END, formatted_message)
        self.received_text.see(tk.END)
        self.received_text.config(state=tk.DISABLED)

    def clear_sent_log(self):
        """Clear the sent messages log"""
        self.sent_text.config(state=tk.NORMAL)
        self.sent_text.delete(1.0, tk.END)
        self.sent_text.config(state=tk.DISABLED)
        self.update_status("Лог отправленных сообщений очищен")

    def clear_received_log(self):
        """Clear the received messages log"""
        self.received_text.config(state=tk.NORMAL)
        self.received_text.delete(1.0, tk.END)
        self.received_text.config(state=tk.DISABLED)
        self.update_status("Лог принятых сообщений очищен")

    def update_status(self, message):
        """Update status label"""
        self.status_label.config(text=message)

    def start_threads(self):
        """Start background threads"""
        # Start thread for reading from serial port
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_from_serial(self):
        """Worker thread: reads data from serial port and adds to queue"""
        serial_buffer = ''
        
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(1)
                continue
                
            try:
                # Read all available data
                if self.ser.in_waiting > 0:
                    data_bytes = self.ser.read(self.ser.in_waiting)

                    try:
                        data_str = data_bytes.decode('utf-8', errors='ignore')
                        serial_buffer += data_str
                        
                        # Process all complete messages in buffer
                        while True:
                            # Find message start
                            start_index = serial_buffer.find('$')
                            if start_index == -1:
                                # No message start found, clear buffer
                                serial_buffer = ''
                                break
                            
                            # Find message end after start
                            end_index = serial_buffer.find('#', start_index + 1)
                            if end_index == -1:
                                # Message not complete, keep everything after last $ in buffer
                                serial_buffer = serial_buffer[start_index:]
                                break
                            
                            # Extract complete message
                            full_message = serial_buffer[start_index:end_index + 1]
                            
                            # Validate message format
                            if full_message.startswith('$') and full_message.endswith('#'):
                                message_text = full_message.strip()
                                # Put message in queue for GUI thread
                                self.received_queue.put(f"{message_text}")
                                
                                # Try to parse for plots or PID values
                                plot_data = self.parse_received_message(message_text)
                                if plot_data and MATPLOTLIB_AVAILABLE:
                                    self.update_all_plots(plot_data)
                            else:
                                self.received_queue.put(f"Некорректный формат: {full_message}")
                            
                            # Remove processed message from buffer
                            remaining_data = serial_buffer[end_index + 1:]
                            serial_buffer = remaining_data
                            
                            # Exit loop if no data left in buffer
                            if not serial_buffer:
                                break

                    except UnicodeDecodeError:
                        self.received_queue.put("Ошибка: не удалось декодировать данные")

            except serial.SerialException as e:
                self.received_queue.put(f"Ошибка чтения из порта: {e}")
                time.sleep(1)
            except Exception as e:
                self.received_queue.put(f"Неожиданная ошибка: {e}")
                time.sleep(1)

    def update_gui(self):
        """Update GUI from queue (called periodically)"""
        # Process all messages in queue
        while not self.received_queue.empty():
            try:
                message = self.received_queue.get_nowait()
                self.log_received_message(message)
            except queue.Empty:
                break
        
        # Schedule next update
        if self.running:
            self.root.after(100, self.update_gui)

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
    """Main function to start the application"""
    
    # You can modify these parameters as needed
    serial_port = '/dev/esp32'  # Change to your serial port
    baudrate = 115200
    
    app = SerialBridgeGUI(serial_port, baudrate)
    app.run()

if __name__ == '__main__':
    main()