#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Pitch Motor Data Plotter
Reads CSV data from serial port and plots in real-time.

Firmware print format:

    USB_CDC_Printf("PITCH_CSV,%lu,%.2f,%.2f,%d,%.2f,%.2f,%.2f\r\n",
                   HAL_GetTick(),
                   c->angle_target,
                   current_angle,
                   c->speed_rpm,
                   cmd,
                   error,
                   rate_normalized * 300.0f);

So each line is:

PITCH_CSV,timestamp_ms,target_angle,current_angle,speed_rpm,cmd,error,rate_scaled
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# Default serial port settings
# On Windows, something like: "COM6"
# On macOS, something like: "/dev/tty.usbmodemXXXX"
DEFAULT_PORT = "/dev/tty.usbmodem3064356030341"
DEFAULT_BAUD = 115200

# Data buffer size (number of points to keep)
BUFFER_SIZE = 1000


class PitchDataPlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # Data buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.target_angle = deque(maxlen=BUFFER_SIZE)
        self.current_angle = deque(maxlen=BUFFER_SIZE)
        self.speed_rpm = deque(maxlen=BUFFER_SIZE)
        self.cmd = deque(maxlen=BUFFER_SIZE)
        self.error = deque(maxlen=BUFFER_SIZE)
        self.rate_scaled = deque(maxlen=BUFFER_SIZE)

        self.start_time = None

        # 3x2 subplots layout (5 plots + 1 unused)
        self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 9))
        self.fig.suptitle('Pitch Motor Real-time Data', fontsize=14, fontweight='bold')

        # Assign subplots
        self.ax1 = self.axes[0, 0]  # Angle comparison
        self.ax2 = self.axes[0, 1]  # Speed
        self.ax3 = self.axes[1, 0]  # cmd (controller output)
        self.ax4 = self.axes[1, 1]  # Error
        self.ax5 = self.axes[2, 0]  # Rate input (scaled)
        self.ax6 = self.axes[2, 1]  # Unused for now

        # Plot lines
        self.line1_target, = self.ax1.plot([], [], 'b-', label='Target Angle', linewidth=1.5)
        self.line1_current, = self.ax1.plot([], [], 'r-', label='Current Angle', linewidth=1.5)
        self.line2_speed, = self.ax2.plot([], [], 'g-', label='Speed (RPM)', linewidth=1.5)
        self.line3_cmd, = self.ax3.plot([], [], 'm-', label='cmd', linewidth=1.5)
        self.line4_err, = self.ax4.plot([], [], 'orange', label='Error', linewidth=1.5)
        self.line5_rate, = self.ax5.plot([], [], 'y-', label='Rate (norm * 300)', linewidth=1.5)

        # Configure axes
        axes_list = [
            (self.ax1, 'Angle (Target vs Current)', 'Angle (deg or rad)'),
            (self.ax2, 'Motor Speed (RPM)', 'RPM'),
            (self.ax3, 'Controller Output (cmd)', 'cmd'),
            (self.ax4, 'Angle Error', 'Error'),
            (self.ax5, 'Rate Input (scaled)', 'rate * 300'),
        ]

        for ax, title, ylabel in axes_list:
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True)

        # Hide the unused last axis
        self.ax6.axis('off')

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
        """
        Expect lines like:
        PITCH_CSV,123456,10.00,9.80,150,123.45,-0.23,90.00
        """
        if not line.startswith('PITCH_CSV'):
            return None
        try:
            parts = line.strip().split(',')
            # prefix + 7 fields = 8 total
            if len(parts) < 8:
                return None

            return {
                'timestamp': int(parts[1]),
                'target_angle': float(parts[2]),
                'current_angle': float(parts[3]),
                'speed_rpm': int(parts[4]),
                'cmd': float(parts[5]),
                'error': float(parts[6]),
                'rate_scaled': float(parts[7]),
            }
        except (ValueError, IndexError):
            return None

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                data = self.parse_csv_line(line)
                if data:
                    if self.start_time is None:
                        self.start_time = data['timestamp']
                    rel_time = (data['timestamp'] - self.start_time) / 1000.0  # ms -> s

                    self.time_data.append(rel_time)
                    self.target_angle.append(data['target_angle'])
                    self.current_angle.append(data['current_angle'])
                    self.speed_rpm.append(data['speed_rpm'])
                    self.cmd.append(data['cmd'])
                    self.error.append(data['error'])
                    self.rate_scaled.append(data['rate_scaled'])
        except Exception as e:
            print(f"Error reading serial data: {e}")

    def update_plot(self, frame):
        """Update plot dynamically with auto-scaling."""
        self.read_serial_data()
        if not self.time_data:
            return

        t = list(self.time_data)

        # Update data lines
        self.line1_target.set_data(t, list(self.target_angle))
        self.line1_current.set_data(t, list(self.current_angle))
        self.line2_speed.set_data(t, list(self.speed_rpm))
        self.line3_cmd.set_data(t, list(self.cmd))
        self.line4_err.set_data(t, list(self.error))
        self.line5_rate.set_data(t, list(self.rate_scaled))

        # Auto-scale axes dynamically
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.relim()
            ax.autoscale_view()

        return [
            self.line1_target, self.line1_current,
            self.line2_speed, self.line3_cmd,
            self.line4_err, self.line5_rate
        ]

    def run(self):
        if not self.connect_serial():
            return
        print("Starting real-time pitch plotter. Close the window to stop.")
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        try:
            plt.show()
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port closed.")


def main():
    port = DEFAULT_PORT
    baudrate = DEFAULT_BAUD
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])

    print("Pitch Motor Data Plotter")
    print(f"Port: {port}, Baudrate: {baudrate}")
    print("Looking for CSV data with prefix 'PITCH_CSV'\n")

    plotter = PitchDataPlotter(port, baudrate)
    plotter.run()


if __name__ == '__main__':
    main()
