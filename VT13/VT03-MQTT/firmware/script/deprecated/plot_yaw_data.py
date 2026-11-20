#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Yaw Motor Data Plotter
Reads CSV data from serial port and plots in real-time
Usage: python plot_yaw_data.py [COM_PORT] [BAUD_RATE]
Example: python plot_yaw_data.py COM3 115200
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# Default serial port settings
DEFAULT_BAUD = 115200

# Data buffer size (number of points to keep)
BUFFER_SIZE = 1000


class YawDataPlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # Data buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.target_angle = deque(maxlen=BUFFER_SIZE)
        self.current_angle = deque(maxlen=BUFFER_SIZE)
        self.speed_rpm = deque(maxlen=BUFFER_SIZE)
        self.cmd_speed_to_current = deque(maxlen=BUFFER_SIZE)
        self.cmd_angle_to_speed = deque(maxlen=BUFFER_SIZE)
        self.rate_input = deque(maxlen=BUFFER_SIZE)
        self.error = deque(maxlen=BUFFER_SIZE)
        self.g_gz = deque(maxlen=BUFFER_SIZE)
        self.c_gz = deque(maxlen=BUFFER_SIZE)

        self.start_time = None

        # 4x2 subplots layout
        self.fig, self.axes = plt.subplots(4, 2, figsize=(14, 13))
        self.fig.suptitle('Yaw Motor Real-time Data', fontsize=14, fontweight='bold')

        # Assign subplots
        self.ax1 = self.axes[0, 0]  # Angle comparison
        self.ax2 = self.axes[0, 1]  # Speed
        self.ax3 = self.axes[1, 0]  # cmd_speed_to_current
        self.ax4 = self.axes[1, 1]  # cmd_angle_to_speed
        self.ax5 = self.axes[2, 0]  # Input rate
        self.ax6 = self.axes[2, 1]  # Error
        self.ax7 = self.axes[3, 0]  # Gyro g_gz
        self.ax8 = self.axes[3, 1]  # Gyro c_gz

        # Plot lines
        self.line1_target, = self.ax1.plot([], [], 'b-', label='Target Angle', linewidth=1.5)
        self.line1_current, = self.ax1.plot([], [], 'r-', label='Current Angle', linewidth=1.5)
        self.line2, = self.ax2.plot([], [], 'g-', label='Speed (RPM)', linewidth=1.5)
        self.line3_cmd1, = self.ax3.plot([], [], 'm-', label='cmd_speed_to_current', linewidth=1.5)
        self.line4_cmd2, = self.ax4.plot([], [], 'c-', label='cmd_angle_to_speed', linewidth=1.5)
        self.line5_rate, = self.ax5.plot([], [], 'y-', label='Rate Input', linewidth=1.5)
        self.line6_err, = self.ax6.plot([], [], 'orange', label='Error', linewidth=1.5)
        self.line7_g, = self.ax7.plot([], [], 'purple', label='g_gz', linewidth=1.5)
        self.line8_c, = self.ax8.plot([], [], 'brown', label='c_gz', linewidth=1.5)

        # Configure all axes
        axes_list = [
            (self.ax1, 'Angle (Target vs Current)', 'Angle'),
            (self.ax2, 'Motor Speed (RPM)', 'RPM'),
            (self.ax3, 'Command: Speed → Current', 'Value'),
            (self.ax4, 'Command: Angle → Speed', 'Value'),
            (self.ax5, 'Input Rate', 'Normalized Rate'),
            (self.ax6, 'Angle Error', 'Error'),
            (self.ax7, 'Gyro g_gz', 'Angular Velocity'),
            (self.ax8, 'Gyro c_gz', 'Angular Velocity'),
        ]

        for ax, title, ylabel in axes_list:
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True)

        plt.tight_layout()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False

    def parse_csv_line(self, line):
        # Format: GIM,timestamp,YAW_CSV,target_angle,current_angle,speed_rpm,cmd_current,cmd_speed,rate,error,g_gz,c_gz
        if 'YAW_CSV' not in line:
            return None

        # Clean up line: find "GIM" and extract from there (removes binary garbage prefix)
        gim_idx = line.find('GIM')
        if gim_idx == -1:
            return None

        clean_line = line[gim_idx:]
        parts = clean_line.split(',')

        if len(parts) < 12:  # GIM + timestamp + YAW_CSV + at least 9 data fields
            return None

        # Check if this is the correct format (parts[0]='GIM', parts[2]='YAW_CSV')
        if parts[0] != 'GIM' or parts[2] != 'YAW_CSV':
            return None

        try:
            return {
                'timestamp': int(parts[1]),
                'target_angle': float(parts[3]),
                'current_angle': float(parts[4]),
                'speed_rpm': int(parts[5]),
                'cmd_speed_to_current': float(parts[6]),
                'cmd_angle_to_speed': float(parts[7]),
                'rate_input': float(parts[8]),
                'error': float(parts[9]),
                'g_gz': float(parts[10]),
                'c_gz': float(parts[11]),
            }
        except (ValueError, IndexError):
            # Parsing failed (likely truncated or malformed data)
            return None

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Try to parse, skip if it fails (e.g., binary vision data)
                data = self.parse_csv_line(line)
                if data:
                    if self.start_time is None:
                        self.start_time = data['timestamp']
                    rel_time = (data['timestamp'] - self.start_time) / 1000.0

                    self.time_data.append(rel_time)
                    self.target_angle.append(data['target_angle'])
                    self.current_angle.append(data['current_angle'])
                    self.speed_rpm.append(data['speed_rpm'])
                    self.cmd_speed_to_current.append(data['cmd_speed_to_current'])
                    self.cmd_angle_to_speed.append(data['cmd_angle_to_speed'])
                    self.rate_input.append(data['rate_input'])
                    self.error.append(data['error'])
                    self.g_gz.append(data['g_gz'])
                    self.c_gz.append(data['c_gz'])
        except Exception as e:
            print(f"Error reading serial data: {e}")

    def update_plot(self, frame):
        """Update plot dynamically with auto-scaling"""
        self.read_serial_data()
        if not self.time_data:
            return []

        t = list(self.time_data)

        # Update data lines
        self.line1_target.set_data(t, list(self.target_angle))
        self.line1_current.set_data(t, list(self.current_angle))
        self.line2.set_data(t, list(self.speed_rpm))
        self.line3_cmd1.set_data(t, list(self.cmd_speed_to_current))
        self.line4_cmd2.set_data(t, list(self.cmd_angle_to_speed))
        self.line5_rate.set_data(t, list(self.rate_input))
        self.line6_err.set_data(t, list(self.error))
        self.line7_g.set_data(t, list(self.g_gz))
        self.line8_c.set_data(t, list(self.c_gz))

        # Auto-scale axes dynamically
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6, self.ax7, self.ax8]:
            ax.relim()        # Recompute limits
            ax.autoscale_view()  # Apply new limits

        return [
            self.line1_target, self.line1_current, self.line2,
            self.line3_cmd1, self.line4_cmd2, self.line5_rate,
            self.line6_err, self.line7_g, self.line8_c
        ]

    def run(self):
        if not self.connect_serial():
            return
        print("Starting real-time plotter. Close the window to stop.")
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        try:
            plt.show()
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port closed.")


def auto_detect_serial_port():
    """Automatically detect available serial ports and let user choose"""
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found!")
        return None

    # Filter for USB serial ports (common patterns)
    usb_ports = [p for p in ports if 'usb' in p.device.lower() or 'USB' in p.description]

    # Use all ports if no USB ports found
    available_ports = usb_ports if usb_ports else ports

    if len(available_ports) == 1:
        selected_port = available_ports[0].device
        print(f"Auto-detected serial port: {selected_port}")
        print(f"Description: {available_ports[0].description}")
        return selected_port

    # Multiple ports found, let user choose
    print("Available serial ports:")
    for i, port in enumerate(available_ports, 1):
        print(f"  {i}. {port.device} - {port.description}")

    try:
        choice = input(f"Select port (1-{len(available_ports)}) or press Enter for first port: ").strip()
        if not choice:
            selected_port = available_ports[0].device
        else:
            idx = int(choice) - 1
            if 0 <= idx < len(available_ports):
                selected_port = available_ports[idx].device
            else:
                print("Invalid choice, using first port")
                selected_port = available_ports[0].device

        print(f"Selected: {selected_port}")
        return selected_port
    except (ValueError, KeyboardInterrupt):
        print("\nUsing first available port")
        return available_ports[0].device


def main():
    # Auto-detect port if not specified in command line
    port = None
    baudrate = DEFAULT_BAUD

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])

    # If port not specified, auto-detect
    if port is None:
        port = auto_detect_serial_port()
        if port is None:
            print("Failed to detect serial port. Please specify manually.")
            print("Usage: python plot_yaw_data.py [COM_PORT] [BAUD_RATE]")
            return

    print(f"\nYaw Motor Data Plotter")
    print(f"Port: {port}, Baudrate: {baudrate}")
    print(f"Looking for CSV data with format: GIM,timestamp,YAW_CSV,...\n")

    plotter = YawDataPlotter(port, baudrate)
    plotter.run()


if __name__ == '__main__':
    main()

