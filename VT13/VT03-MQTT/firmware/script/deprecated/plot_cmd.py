#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Yaw Monitor (3 separate graphs)
Reads CSV from serial:
YAW_CSV,timestamp,yaw_raw,vision_yaw,yaw_rate
"""
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

DEFAULT_PORT = 'COM6'
DEFAULT_BAUD = 115200
BUFFER_SIZE = 1000

class YawSimplePlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # Buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.yaw_raw = deque(maxlen=BUFFER_SIZE)
        self.vision_yaw = deque(maxlen=BUFFER_SIZE)
        self.yaw_rate = deque(maxlen=BUFFER_SIZE)

        self.start_time = None

        # 3 stacked plots (time on x axis)
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.suptitle('Yaw Monitor (Raw / Vision / Rate)', fontsize=14, fontweight='bold')

        self.ax_raw = self.axes[0]
        self.ax_vision = self.axes[1]
        self.ax_rate = self.axes[2]

        # Lines
        self.line_raw, = self.ax_raw.plot([], [], label='yaw_raw', color='tab:blue', linewidth=1.5)
        self.line_vision, = self.ax_vision.plot([], [], label='vision_yaw', color='tab:orange', linewidth=1.5)
        self.line_rate, = self.ax_rate.plot([], [], label='yaw_rate', color='tab:green', linewidth=1.5)

        # Axis settings
        settings = [
            (self.ax_raw, "IMU Raw Yaw", "Yaw (deg)"),
            (self.ax_vision, "Vision Yaw", "Yaw (deg)"),
            (self.ax_rate, "Yaw Rate", "deg/s"),
        ]

        for ax, title, ylabel in settings:
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.grid(True)
            ax.legend(loc='upper right')

        self.ax_rate.set_xlabel("Time (s)")
        plt.tight_layout(rect=[0, 0, 1, 0.97])

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            return False

    def parse_csv_line(self, line):
        """
        Expected format:
        YAW_CSV,timestamp,yaw_raw,vision_yaw,yaw_rate
        timestamp is ms (uint32), yaw_* are floats
        """
        if not line:
            return None
        s = line.strip()
        parts = s.split(',')
        if len(parts) < 5:
            return None
        if not parts[0].upper().startswith("YAW_CSV"):
            return None
        try:
            timestamp = int(parts[1])
            yaw_raw = float(parts[2])
            vision_yaw = float(parts[3])
            yaw_rate = float(parts[4])
            return {'timestamp': timestamp, 'yaw_raw': yaw_raw, 'vision_yaw': vision_yaw, 'yaw_rate': yaw_rate}
        except ValueError:
            return None

    def read_serial_data(self):
        if self.ser is None or not getattr(self.ser, "is_open", False):
            return
        try:
            # read lines until timeout returns no data; readline() respects the serial timeout
            while True:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode('utf-8', errors='ignore')
                data = self.parse_csv_line(line)
                if data:
                    if self.start_time is None:
                        self.start_time = data['timestamp']
                    # convert ms -> seconds relative to start
                    t = (data['timestamp'] - self.start_time) / 1000.0
                    self.time_data.append(t)
                    self.yaw_raw.append(data['yaw_raw'])
                    self.vision_yaw.append(data['vision_yaw'])
                    self.yaw_rate.append(data['yaw_rate'])
        except Exception as e:
            print(f"Serial read error: {e}")

    def update_plot(self, *args):
        self.read_serial_data()
        if not self.time_data:
            return []
        t = list(self.time_data)
        self.line_raw.set_data(t, list(self.yaw_raw))
        self.line_vision.set_data(t, list(self.vision_yaw))
        self.line_rate.set_data(t, list(self.yaw_rate))

        for ax in (self.ax_raw, self.ax_vision, self.ax_rate):
            ax.relim()
            ax.autoscale_view()
        return [self.line_raw, self.line_vision, self.line_rate]

    def run(self):
        if not self.connect_serial():
            return
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,
            blit=False,
            cache_frame_data=False
        )
        plt.show()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial closed.")

def main():
    port = DEFAULT_PORT
    baud = DEFAULT_BAUD
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])
    plotter = YawSimplePlotter(port, baud)
    plotter.run()

if __name__ == "__main__":
    main()
