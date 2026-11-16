#!/usr/bin/env python3
"""Read-only parser for the current C-board USB CDC Seasky telemetry."""

import math
import struct
import threading
import time
from dataclasses import dataclass


SOF = 0xA5
TELEMETRY_CMD = 0x0002
TELEMETRY_DATA_LENGTH = 14  # flags (2) + three float32 values (12).
TELEMETRY_FRAME_LENGTH = TELEMETRY_DATA_LENGTH + 8


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x31 if value & 0x80 else 0)) & 0xFF
    return value


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xA001 if value & 1 else 0)
    return value


@dataclass(frozen=True)
class GyroTelemetry:
    # Current firmware sends g_gz, g_gx, g_gy: angular rates in rad/s.
    gyro_z_rad_s: float
    gyro_x_rad_s: float
    gyro_y_rad_s: float
    flags: int


class SeaskyTelemetryParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            del self.buffer[:-TELEMETRY_FRAME_LENGTH]
        frames = []
        while self.buffer:
            index = self.buffer.find(bytes((SOF,)))
            if index < 0:
                self.buffer.clear()
                break
            if index:
                del self.buffer[:index]
            if len(self.buffer) < 4:
                break
            if crc8(self.buffer[:3]) != self.buffer[3]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            length = self.buffer[1] | (self.buffer[2] << 8)
            if length != TELEMETRY_DATA_LENGTH:
                del self.buffer[0]
                continue
            if len(self.buffer) < TELEMETRY_FRAME_LENGTH:
                break
            packet = bytes(self.buffer[:TELEMETRY_FRAME_LENGTH])
            if crc16(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:TELEMETRY_FRAME_LENGTH]
            if int.from_bytes(packet[4:6], "little") != TELEMETRY_CMD:
                continue
            rates = struct.unpack_from("<fff", packet, 8)
            if not all(math.isfinite(value) for value in rates):
                continue
            frames.append(GyroTelemetry(*rates, int.from_bytes(packet[6:8], "little")))
        return frames


class CBoardReader:
    """Only reads USB CDC. It never writes a byte to the board."""

    def __init__(self, port):
        self.port = port
        self.parser = SeaskyTelemetryParser()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.status = "waiting for port"
        self.packet_count = 0
        self.last_packet_s = None
        self.latest = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def snapshot(self):
        with self.lock:
            age = time.monotonic() - self.last_packet_s if self.last_packet_s else None
            connected = age is not None and age < .5
            latest = self.latest
            return {
                "board_status": "receiving" if connected else self.status if age is None else "telemetry stale",
                "board_connected": connected,
                "board_rx_frames": self.packet_count,
                "board_crc_errors": self.parser.crc_errors,
                "board_frame_age_ms": round(age * 1000) if age is not None else None,
                "board_gyro_rad_s": ([latest.gyro_z_rad_s, latest.gyro_x_rad_s,
                                      latest.gyro_y_rad_s] if latest else None),
                "command_sent": False,
            }

    def _run(self):
        try:
            import serial
        except ImportError:
            with self.lock:
                self.status = "pyserial missing"
            return
        while not self.stop_event.is_set():
            try:
                # Opening USB CDC may reset some boards. Keep DTR and RTS inactive.
                link = serial.Serial(port=None, baudrate=115200, timeout=.1,
                                     write_timeout=.1, exclusive=True)
                link.dtr = False
                link.rts = False
                link.port = self.port
                link.open()
                with self.lock:
                    self.status = "waiting for telemetry"
                try:
                    while not self.stop_event.is_set():
                        data = link.read(128)
                        if not data:
                            continue
                        with self.lock:
                            frames = self.parser.feed(data)
                            for frame in frames:
                                self.latest = frame
                                self.last_packet_s = time.monotonic()
                                self.packet_count += 1
                finally:
                    link.close()
            except (OSError, serial.SerialException):
                with self.lock:
                    self.status = "waiting for port"
                self.stop_event.wait(1)
