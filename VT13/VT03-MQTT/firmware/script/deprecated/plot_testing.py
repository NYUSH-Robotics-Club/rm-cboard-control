#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Chassis Command Plotter
Reads serial output and plots chassis command signals in real-time.

Firmware print format:

    USB_CDC_Printf("[CHASSIS_CMD] en=%d vx=%.2f vy=%.2f wz=%.2f\r\n",
                   (int)s_last_cmd.enabled, s_last_cmd.vx, s_last_cmd.vy, s_last_cmd.wz);

Example line:

    [CHASSIS_CMD] en=1 vx=0.23 vy=-0.41 wz=0.05
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys
import re
import time

# Default serial port settings
DEFAULT_PORT = "/dev/tty.usbmodem3088357D30341"
DEFAULT_BAUD = 115200

# Data buffer size (number of points to keep)
BUFFER_SIZE = 1000

# Regex to parse: [CHASSIS_CMD] en=1 vx=0.23 vy=-0.41 wz=0.05
CHASSIS_RE = re.compile(
    r"^\[CHASSIS_CMD\]\s+en=(\d+)\s+vx=([-\d\.]+)\s+vy=([-\d\.]+)\s+wz=([-\d\.]+)\s*$"
)


class ChassisCmdPlotter:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # Data buffers
        self.time_data = deque(maxlen=BUFFER_SIZE)
        self.vx_data = deque(maxlen=BUFFER_SIZE)
        self.vy_data = deque(maxlen=BUFFER_SIZE)
        self.wz_data = deque(maxlen=BUFFER_SIZE)
        self.en_data = deque(maxlen=BUFFER_SIZE)

        self.start_time = None

        # 2x2 layout (vx, vy, wz, enabled)
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 8))
        self.fig.suptitle('Chassis Command Real-time Data', fontsize=14, fontweight='bold')

        self.ax_vx = self.axes[0, 0]
        self.ax_vy = self.axes[0, 1]
        self.ax_wz = self.axes[1, 0]
        self.ax_en = self.axes[1, 1]

        # Plot lines
        self.line_vx, = self.ax_vx.plot([], [], label='vx', linewidth=1.5)
        self.line_vy, = self.ax_vy.plot([], [], label='vy', linewidth=1.5)
        self.line_wz, = self.ax_wz.plot([], [], label='wz', linewidth=1.5)
        self.line_en, = self.ax_en.plot([], [], label='enabled', linewidth=1.5)

        # Configure axes
        axes_list = [
            (self.ax_vx, "vx (forward)", "vx"),
            (self.ax_vy, "vy (strafe)", "vy"),
            (self.ax_wz, "wz (spin)", "wz"),
            (self.ax_en, "enabled", "en (0/1)"),
        ]

        for ax, title, ylabel in axes_list:
            ax.set_title(title)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True)

        # For enabled plot, keep a stable y-range
        self.ax_en.set_ylim(-0.2, 1.2)

        plt.tight_layout()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False

    def parse_line(self, line: str):
        """
        Expect lines like:
            [CHASSIS_CMD] en=1 vx=0.23 vy=-0.41 wz=0.05
        """
        m = CHASSIS_RE.match(line.strip())
        if not m:
            return None
        try:
            return {
                "en": int(m.group(1)),
                "vx": float(m.group(2)),
                "vy": float(m.group(3)),
                "wz": float(m.group(4)),
            }
        except ValueError:
            return None

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore")
                data = self.parse_line(line)
                if data:
                    now = time.time()
                    if self.start_time is None:
                        self.start_time = now
                    rel_time = now - self.start_time

                    self.time_data.append(rel_time)
                    self.en_data.append(data["en"])
                    self.vx_data.append(data["vx"])
                    self.vy_data.append(data["vy"])
                    self.wz_data.append(data["wz"])
        except Exception as e:
            print(f"Error reading serial data: {e}")

    def update_plot(self, frame):
        self.read_serial_data()
        if not self.time_data:
            return

        t = list(self.time_data)

        # Update lines
        self.line_vx.set_data(t, list(self.vx_data))
        self.line_vy.set_data(t, list(self.vy_data))
        self.line_wz.set_data(t, list(self.wz_data))
        self.line_en.set_data(t, list(self.en_data))

        # Auto-scale command axes, keep enabled stable
        for ax in [self.ax_vx, self.ax_vy, self.ax_wz]:
            ax.relim()
            ax.autoscale_view()

        # Keep x-limits synced across all plots if we have enough points
        if len(t) > 1:
            xmin, xmax = t[0], t[-1]
            for ax in [self.ax_vx, self.ax_vy, self.ax_wz, self.ax_en]:
                ax.set_xlim(xmin, xmax)

            # Show enabled in title for quick glance
            self.fig.suptitle(
                f"Chassis Command Real-time Data (enabled={self.en_data[-1]})",
                fontsize=14,
                fontweight="bold"
            )

        return [self.line_vx, self.line_vy, self.line_wz, self.line_en]

    def run(self):
        if not self.connect_serial():
            return
        print("Starting real-time chassis cmd plotter. Close the window to stop.")
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

    print("Chassis Cmd Plotter")
    print(f"Port: {port}, Baudrate: {baudrate}")
    print("Looking for lines with prefix '[CHASSIS_CMD]'\n")

    plotter = ChassisCmdPlotter(port, baudrate)
    plotter.run()


if __name__ == "__main__":
    main()
