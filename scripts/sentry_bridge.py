#!/usr/bin/env python3
"""Single-port sentry bridge for vision SP and radar legacy tool.

This bridge owns the real MCU USB CDC port and exposes two local POSIX PTYs:
  - vision PTY: raw SP pass-through for the vision application
  - radar PTY: legacy 19-byte radar control protocol + telemetry frames

MCU-side protocol:
  - Vision app -> MCU: SP VisionToGimbal frames (29 bytes), passed through raw
  - MCU -> Vision app: SP GimbalToVision frames (43 bytes), routed to vision PTY
  - Bridge -> MCU: SX sentry control frames (33 bytes), built from radar legacy commands + functional RobotControl
  - MCU -> Bridge: ST sentry telemetry frames (39 bytes), converted to radar telemetry/status frames

Radar-side protocol:
  - Tool -> Bridge: A5 5A + vx + vy + wz + yaw_deg + CRC8 (19 bytes)
    vx>0 means forward in the radar / yaw-forward frame, vy>0 means left
  - Tool -> Bridge: A3 + control_flags + chassis_spin_vel + scan_yaw_rate_deg_s + search_pitch_deg + CRC16 (16 bytes)
  - Bridge -> Tool: A6 6A + vx + vy + wz + reserved0 + CRC8 (19 bytes)
  - Bridge -> Tool: 5C + game_status + stage_remain_time + CRC16 (6 bytes)
  - Bridge -> Tool: 5D + robot_id + current_hp + shooter_heat + team_color + is_attacked + CRC16 (10 bytes)
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import json
import math
import os
import pty
import selectors
import struct
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:
    print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def _try_enable_ros2() -> bool:
    candidate_paths = (
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.10/dist-packages",
    )

    def _append_python_paths() -> None:
        for path in candidate_paths:
            if os.path.isdir(path) and path not in sys.path:
                sys.path.append(path)
        for path in os.environ.get("PYTHONPATH", "").split(":"):
            if path and os.path.isdir(path) and path not in sys.path:
                sys.path.append(path)

    def _do_imports() -> bool:
        try:
            global rclpy, Odometry, TransformStamped, TransformBroadcaster
            import rclpy
            from geometry_msgs.msg import TransformStamped
            from nav_msgs.msg import Odometry
            from tf2_ros import TransformBroadcaster
        except ImportError:
            return False
        return True

    _append_python_paths()
    if _do_imports():
        return True

    setup_script = "/opt/ros/humble/setup.bash"
    if not os.path.isfile(setup_script):
        return False

    probe = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
        "python3 - <<'PY'\n"
        "import json, os\n"
        "keys = ['LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'PATH']\n"
        "print(json.dumps({k: os.environ.get(k, '') for k in keys}))\n"
        "PY"
    )
    try:
        result = subprocess.run(
            ["bash", "-lc", probe],
            check=True,
            capture_output=True,
            text=True,
        )
        env_delta = json.loads(result.stdout.strip())
    except Exception:
        return False

    for key, value in env_delta.items():
        if value:
            os.environ[key] = value
    _append_python_paths()
    return _do_imports()


SP_HEADER = b"SP"
SX_HEADER = b"SX"
ST_HEADER = b"ST"

RADAR_CMD_HEADER = b"\xA5\x5A"
RADAR_TELEM_HEADER = b"\xA6\x6A"
ROBOT_CTRL_HEADER = b"\xA3"
GAME_STATUS_HEADER = b"\x5C"
ROBOT_STATUS_HEADER = b"\x5D"

SP_RX_SIZE = 29
SP_TX_SIZE = 43
SX_SIZE = 33
ST_SIZE = 39
RADAR_FRAME_SIZE = 19
ROBOT_CTRL_FRAME_SIZE = 16
GAME_STATUS_FRAME_SIZE = 6
ROBOT_STATUS_FRAME_SIZE = 10

SENTRY_EXT_FLAG_SCAN_CONTROL_VALID = 0x01
SENTRY_EXT_FLAG_STOP_GIMBAL_SCAN = 0x02
SENTRY_EXT_FLAG_SCAN_ENABLED = 0x04
SENTRY_EXT_FLAG_ALLOW_VISION_CONTROL = 0x08
SENTRY_EXT_FLAG_SEARCH_WHEN_TARGET_LOST = 0x10

AUTO_PORT_VID_PID = (0x0483, 0x00CA)
DEFAULT_VISION_LINK = "/tmp/nyush-rm-sentry-vision"
DEFAULT_RADAR_LINK = "/tmp/nyush-rm-sentry-radar"
DEFAULT_LOCK_DIR = "/tmp"
SENTRY_BRIDGE_DEBUG = os.environ.get("SENTRY_BRIDGE_DEBUG", "").lower() not in ("", "0", "false", "no")
ROS2_RUNTIME_AVAILABLE = _try_enable_ros2()
DEFAULT_CHASSIS_ODOM_TOPIC = os.environ.get("SENTRY_CHASSIS_ODOM_TOPIC", "/chassis_odom")
DEFAULT_ODOM_FRAME = os.environ.get("SENTRY_CHASSIS_ODOM_FRAME", "odom")
DEFAULT_BASE_FRAME = os.environ.get("SENTRY_CHASSIS_BASE_FRAME", "base_footprint")
DEFAULT_LIDAR_FRAME = os.environ.get("SENTRY_CHASSIS_LIDAR_FRAME", "body_feedback")


def _read_axis_sign(env_name: str, default: float) -> float:
    raw = os.environ.get(env_name, "").strip()
    if not raw:
        return default
    try:
        return -1.0 if float(raw) < 0.0 else 1.0
    except ValueError:
        return default


BRIDGE_VX_SIGN = _read_axis_sign("SENTRY_BRIDGE_VX_SIGN", 1.0)
BRIDGE_VY_SIGN = _read_axis_sign("SENTRY_BRIDGE_VY_SIGN", 1.0)


def bridge_to_mcu_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    # Keep the bridge neutral by default; use env overrides only when the
    # firmware on the robot explicitly needs an axis sign correction.
    return BRIDGE_VX_SIGN * vx, BRIDGE_VY_SIGN * vy, wz


def mcu_to_bridge_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    return bridge_to_mcu_velocity(vx, vy, wz)


def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_sx_frame(
    vx: float,
    vy: float,
    wz: float,
    control_flags: int = 0,
    scan_yaw_rate_deg_s: float = 0.0,
    search_pitch_deg: float = float("nan"),
) -> bytes:
    payload = struct.pack(
        "<2s5fB2f",
        SX_HEADER,
        vx,
        vy,
        wz,
        0.0,
        0.0,
        control_flags & 0xFF,
        scan_yaw_rate_deg_s,
        search_pitch_deg,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def parse_sx_frame(frame: bytes) -> tuple[float, float, float, float, float, int, float, float]:
    if len(frame) != SX_SIZE or frame[:2] != SX_HEADER:
        raise ValueError("invalid SX frame")
    if crc16_x25(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ValueError("invalid SX CRC16")
    (
        _,
        vx,
        vy,
        wz,
        yaw_delta,
        pitch_delta,
        control_flags,
        scan_yaw_rate_deg_s,
        search_pitch_deg,
    ) = struct.unpack("<2s5fB2f", frame[:-2])
    return (
        vx,
        vy,
        wz,
        yaw_delta,
        pitch_delta,
        control_flags,
        scan_yaw_rate_deg_s,
        search_pitch_deg,
    )


def build_st_frame(
    cmd_vx: float,
    cmd_vy: float,
    cmd_wz: float,
    real_vx: float = 0.0,
    real_vy: float = 0.0,
    real_wz: float = 0.0,
    robot_status: int = 0,
    game_status: int = 0,
    stage_remain_time: int = 0,
    robot_id: int = 0,
    current_hp: int = 0,
    shooter_heat: int = 0,
    team_color: int = 0,
    is_attacked: int = 0,
) -> bytes:
    payload = struct.pack(
        "<2s6fBBHBHHBB",
        ST_HEADER,
        cmd_vx,
        cmd_vy,
        cmd_wz,
        real_vx,
        real_vy,
        real_wz,
        robot_status,
        game_status,
        stage_remain_time,
        robot_id,
        current_hp,
        shooter_heat,
        team_color,
        is_attacked,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def parse_st_frame(
    frame: bytes,
) -> tuple[float, float, float, float, float, float, int, int, int, int, int, int, int, int]:
    if len(frame) != ST_SIZE or frame[:2] != ST_HEADER:
        raise ValueError("invalid ST frame")
    if crc16_x25(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ValueError("invalid ST CRC16")
    (
        _,
        cmd_vx,
        cmd_vy,
        cmd_wz,
        real_vx,
        real_vy,
        real_wz,
        robot_status,
        game_status,
        stage_remain_time,
        robot_id,
        current_hp,
        shooter_heat,
        team_color,
        is_attacked,
    ) = struct.unpack("<2s6fBBHBHHBB", frame[:-2])
    return (
        cmd_vx,
        cmd_vy,
        cmd_wz,
        real_vx,
        real_vy,
        real_wz,
        robot_status,
        game_status,
        stage_remain_time,
        robot_id,
        current_hp,
        shooter_heat,
        team_color,
        is_attacked,
    )


def build_radar_cmd_frame(vx: float, vy: float, wz: float, yaw_deg: float) -> bytes:
    payload = struct.pack("<2s4f", RADAR_CMD_HEADER, vx, vy, wz, yaw_deg)
    return payload + bytes([crc8(payload)])


def parse_radar_cmd_frame(frame: bytes) -> tuple[float, float, float, float]:
    if len(frame) != RADAR_FRAME_SIZE or frame[:2] != RADAR_CMD_HEADER:
        raise ValueError("invalid radar command frame")
    if crc8(frame[:-1]) != frame[-1]:
        raise ValueError("invalid radar command CRC8")
    _, vx, vy, wz, yaw_deg = struct.unpack("<2s4f", frame[:-1])
    return vx, vy, wz, yaw_deg


def build_robot_control_frame(
    control_flags: int,
    chassis_spin_vel: float = 0.0,
    scan_yaw_rate_deg_s: float = 0.0,
    search_pitch_deg: float = float("nan"),
) -> bytes:
    payload = struct.pack(
        "<BBfff",
        ROBOT_CTRL_HEADER[0],
        control_flags & 0xFF,
        chassis_spin_vel,
        scan_yaw_rate_deg_s,
        search_pitch_deg,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def parse_robot_control_frame(frame: bytes) -> tuple[int, float, float, float]:
    if len(frame) != ROBOT_CTRL_FRAME_SIZE or frame[:1] != ROBOT_CTRL_HEADER:
        raise ValueError("invalid robot control frame")
    if crc16_x25(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ValueError("invalid robot control CRC16")
    _header, control_flags, chassis_spin_vel, scan_yaw_rate_deg_s, search_pitch_deg = struct.unpack(
        "<BBfff", frame[:-2]
    )
    return int(control_flags), chassis_spin_vel, scan_yaw_rate_deg_s, search_pitch_deg


def build_radar_telem_frame(vx: float, vy: float, wz: float) -> bytes:
    payload = struct.pack("<2s4f", RADAR_TELEM_HEADER, vx, vy, wz, 0.0)
    return payload + bytes([crc8(payload)])


def build_game_status_frame(game_status: int, stage_remain_time: int) -> bytes:
    payload = struct.pack(
        "<BBH",
        GAME_STATUS_HEADER[0],
        game_status & 0xFF,
        stage_remain_time & 0xFFFF,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def build_robot_status_frame(
    robot_id: int,
    current_hp: int,
    shooter_heat: int,
    team_color: int,
    is_attacked: int,
) -> bytes:
    payload = struct.pack(
        "<BBHHBB",
        ROBOT_STATUS_HEADER[0],
        robot_id & 0xFF,
        current_hp & 0xFFFF,
        shooter_heat & 0xFFFF,
        1 if team_color else 0,
        1 if is_attacked else 0,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def same_config_float(lhs: float, rhs: float, eps: float = 1e-6) -> bool:
    if math.isnan(lhs) and math.isnan(rhs):
        return True
    if math.isnan(lhs) or math.isnan(rhs):
        return False
    return abs(lhs - rhs) <= eps


def build_dummy_sp_tx_frame() -> bytes:
    payload = struct.pack(
        "<2sB9fH",
        SP_HEADER,
        1,
        1.0,
        0.0,
        0.0,
        0.0,
        0.1,
        0.2,
        -0.1,
        -0.2,
        30.0,
        123,
    )
    return payload + struct.pack("<H", crc16_x25(payload))


def parse_sp_tx_frame(frame: bytes) -> tuple[tuple[float, float, float, float], float, float, float, float]:
    if len(frame) != SP_TX_SIZE or frame[:2] != SP_HEADER:
        raise ValueError("invalid SP TX frame")
    if crc16_x25(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ValueError("invalid SP TX CRC16")
    (
        _,
        _mode,
        q0,
        q1,
        q2,
        q3,
        yaw,
        yaw_vel,
        pitch,
        pitch_vel,
        _bullet_speed,
        _bullet_count,
    ) = struct.unpack("<2sB9fH", frame[:-2])
    return (q0, q1, q2, q3), yaw, yaw_vel, pitch, pitch_vel


class Ros2TelemetryPublisher:
    def __init__(
        self,
        odom_topic: str = DEFAULT_CHASSIS_ODOM_TOPIC,
        odom_frame: str = DEFAULT_ODOM_FRAME,
        base_frame: str = DEFAULT_BASE_FRAME,
        lidar_frame: str = DEFAULT_LIDAR_FRAME,
    ) -> None:
        self.available = ROS2_RUNTIME_AVAILABLE
        self.odom_topic = odom_topic
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.lidar_frame = lidar_frame
        self.node = None
        self.odom_pub = None
        self.tf_broadcaster = None
        self.last_mono_s: float | None = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.body_yaw = 0.0
        self.latest_vx = 0.0
        self.latest_vy = 0.0
        self.latest_wz = 0.0
        self._publish_lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._owns_rclpy = False

        if not self.available:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        self.node = rclpy.create_node("sentry_bridge_ros")
        self.odom_pub = self.node.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.publish_body_tf(0.0)
        self.publish_chassis_odom(0.0, 0.0, 0.0)
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="sentry_bridge_ros_heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def pump_ros(self) -> None:
        if self.node is None or not ROS2_RUNTIME_AVAILABLE:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _heartbeat_loop(self) -> None:
        while not self._heartbeat_stop.wait(0.05):
            self.publish_chassis_odom(self.latest_vx, self.latest_vy, self.latest_wz)

    def close(self) -> None:
        self._heartbeat_stop.set()
        if self._heartbeat_thread is not None:
            try:
                self._heartbeat_thread.join(timeout=0.5)
            except Exception:
                pass
            self._heartbeat_thread = None
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if self._owns_rclpy and ROS2_RUNTIME_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def publish_body_tf(self, yaw_rad: float) -> None:
        with self._publish_lock:
            if self.node is None or self.tf_broadcaster is None:
                return
            self.body_yaw = float(yaw_rad)
            t = TransformStamped()
            t.header.stamp = self._stamp()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.lidar_frame
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw_rad * 0.5)
            t.transform.rotation.w = math.cos(yaw_rad * 0.5)
            self.tf_broadcaster.sendTransform(t)
            self.pump_ros()

    def publish_chassis_odom(self, vx: float, vy: float, wz: float) -> None:
        with self._publish_lock:
            if self.node is None or self.odom_pub is None:
                return
            self.latest_vx = float(vx)
            self.latest_vy = float(vy)
            self.latest_wz = float(wz)
            now_mono = time.monotonic()
            if self.last_mono_s is not None:
                dt = max(0.0, min(now_mono - self.last_mono_s, 0.2))
                cos_yaw = math.cos(self.yaw)
                sin_yaw = math.sin(self.yaw)
                self.x += (cos_yaw * self.latest_vx - sin_yaw * self.latest_vy) * dt
                self.y += (sin_yaw * self.latest_vx + cos_yaw * self.latest_vy) * dt
                self.yaw += self.latest_wz * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            self.last_mono_s = now_mono

            msg = Odometry()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
            msg.twist.twist.linear.x = self.latest_vx
            msg.twist.twist.linear.y = self.latest_vy
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = self.latest_wz
            self.odom_pub.publish(msg)
            self.publish_body_tf(self.body_yaw)
            self.pump_ros()


class RadarCommandParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[str, tuple[object, ...]]]:
        events: list[tuple[str, tuple[object, ...]]] = []
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 1:
            if self.buffer[0:1] == ROBOT_CTRL_HEADER:
                if len(self.buffer) < ROBOT_CTRL_FRAME_SIZE:
                    break
                frame = bytes(self.buffer[:ROBOT_CTRL_FRAME_SIZE])
                try:
                    events.append(("robot_control", parse_robot_control_frame(frame)))
                    del self.buffer[:ROBOT_CTRL_FRAME_SIZE]
                except ValueError:
                    del self.buffer[0]
                continue

            if len(self.buffer) >= 2 and self.buffer[0:2] == RADAR_CMD_HEADER:
                if len(self.buffer) < RADAR_FRAME_SIZE:
                    break

                frame = bytes(self.buffer[:RADAR_FRAME_SIZE])
                try:
                    events.append(("radar_cmd", parse_radar_cmd_frame(frame)))
                    del self.buffer[:RADAR_FRAME_SIZE]
                except ValueError:
                    del self.buffer[0]
                continue

            del self.buffer[0]

        return events


class McuStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(
        self, data: bytes
    ) -> list[tuple[str, bytes | tuple[object, ...]]]:
        events: list[tuple[str, bytes | tuple[object, ...]]] = []
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 2:
            if self.buffer[0] != ord("S"):
                del self.buffer[0]
                continue

            frame_type = self.buffer[1]
            if frame_type == ord("P"):
                if len(self.buffer) < SP_RX_SIZE:
                    break
                if len(self.buffer) >= SP_TX_SIZE:
                    sp_tx = bytes(self.buffer[:SP_TX_SIZE])
                    if crc16_x25(sp_tx[:-2]) == struct.unpack("<H", sp_tx[-2:])[0]:
                        events.append(("sp", sp_tx))
                        del self.buffer[:SP_TX_SIZE]
                        continue
                sp_rx = bytes(self.buffer[:SP_RX_SIZE])
                if crc16_x25(sp_rx[:-2]) == struct.unpack("<H", sp_rx[-2:])[0]:
                    events.append(("sp", sp_rx))
                    del self.buffer[:SP_RX_SIZE]
                    continue
                if len(self.buffer) < SP_TX_SIZE:
                    break
                del self.buffer[0]
                continue

            if frame_type == ord("X"):
                if len(self.buffer) < SX_SIZE:
                    break
                sx_frame = bytes(self.buffer[:SX_SIZE])
                if crc16_x25(sx_frame[:-2]) == struct.unpack("<H", sx_frame[-2:])[0]:
                    events.append(("sx", sx_frame))
                    del self.buffer[:SX_SIZE]
                else:
                    del self.buffer[0]
                continue

            if frame_type == ord("T"):
                if len(self.buffer) < ST_SIZE:
                    break
                st_frame = bytes(self.buffer[:ST_SIZE])
                if crc16_x25(st_frame[:-2]) == struct.unpack("<H", st_frame[-2:])[0]:
                    events.append(("st", parse_st_frame(st_frame)))
                    del self.buffer[:ST_SIZE]
                else:
                    del self.buffer[0]
                continue

            del self.buffer[0]

        return events


def configure_pty_slave(slave_fd: int) -> None:
    tty.setraw(slave_fd)
    attrs = termios.tcgetattr(slave_fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] |= termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)


def install_pty_link(link_path: str | None, target_path: str) -> str | None:
    if not link_path:
        return None

    link_path = os.path.abspath(link_path)
    parent = os.path.dirname(link_path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    if os.path.lexists(link_path):
        if not os.path.islink(link_path):
            raise RuntimeError(f"PTY link path exists and is not a symlink: {link_path}")
        os.unlink(link_path)

    temp_link_path = f"{link_path}.tmp.{os.getpid()}"
    if os.path.lexists(temp_link_path):
        os.unlink(temp_link_path)
    os.symlink(target_path, temp_link_path)
    os.replace(temp_link_path, link_path)
    return link_path


def remove_pty_link(link_path: str | None, target_path: str) -> None:
    if not link_path or not os.path.islink(link_path):
        return
    try:
        if os.readlink(link_path) == target_path:
            os.unlink(link_path)
    except OSError:
        pass


def default_lock_path_for_port(port: str) -> str:
    port_name = os.path.basename(port) or "serial"
    safe_port_name = "".join(ch if ch.isalnum() or ch in ("-", "_", ".") else "_" for ch in port_name)
    return os.path.join(DEFAULT_LOCK_DIR, f"nyush-rm-sentry-bridge-{safe_port_name}.lock")


def acquire_port_lock(port: str):
    lock_path = default_lock_path_for_port(port)
    lock_file = open(lock_path, "a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.seek(0)
        holder = lock_file.read().strip()
        holder_hint = f" held by pid {holder}" if holder else ""
        lock_file.close()
        raise RuntimeError(
            f"bridge already running for {port}: lock {lock_path}{holder_hint}"
        ) from exc

    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(f"{os.getpid()}\n")
    lock_file.flush()
    return lock_file, lock_path


def release_port_lock(lock_file, lock_path: str | None) -> None:
    if lock_file is None:
        return
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        lock_file.close()
    except OSError:
        pass


def safe_write_fd(fd: int, data: bytes) -> bool:
    if not data:
        return True
    try:
        view = memoryview(data)
        while view:
            written = os.write(fd, view)
            view = view[written:]
        return True
    except OSError as exc:
        if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO, errno.EBADF):
            return False
        raise


def list_serial_ports() -> list:
    return sorted(comports(), key=lambda port: port.device)


def format_port_summary(port) -> str:
    details = [port.device]
    desc = getattr(port, "description", "") or ""
    if desc and desc != "n/a":
        details.append(desc)
    vid = getattr(port, "vid", None)
    pid = getattr(port, "pid", None)
    if vid is not None and pid is not None:
        details.append(f"VID:PID={vid:04X}:{pid:04X}")
    return " - ".join(details)


def port_priority(port) -> int:
    device = (getattr(port, "device", "") or "").lower()
    text = " ".join(
        str(value).lower()
        for value in (
            getattr(port, "description", ""),
            getattr(port, "product", ""),
            getattr(port, "manufacturer", ""),
            getattr(port, "interface", ""),
            getattr(port, "hwid", ""),
        )
        if value
    )

    if device.startswith("/dev/ttys"):
        return -100

    score = 0
    if device.startswith("/dev/ttyacm") or device.startswith("/dev/cu.usbmodem"):
        score += 20
    elif device.startswith("/dev/ttyusb") or device.startswith("/dev/cu.usbserial"):
        score += 10

    vid = getattr(port, "vid", None)
    pid = getattr(port, "pid", None)
    if (vid, pid) == AUTO_PORT_VID_PID:
        score += 100

    if "vision comm" in text:
        score += 80
    if "st-link" in text or "j-link" in text:
        score -= 40

    return score


def print_available_ports(stream) -> None:
    ports = list_serial_ports()
    if not ports:
        print("Available serial ports: none detected.", file=stream)
        return

    preferred_ports = [
        port
        for port in ports
        if port_priority(port) >= 0
        or getattr(port, "vid", None) is not None
        or (getattr(port, "device", "") or "").startswith(("/dev/ttyACM", "/dev/ttyUSB"))
    ]
    display_ports = preferred_ports or ports

    print("Available serial ports:", file=stream)
    for port in display_ports:
        print(f"  - {format_port_summary(port)}", file=stream)


def resolve_serial_port(port_arg: str | None) -> tuple[str, bool]:
    if port_arg and port_arg.lower() != "auto":
        return port_arg, False

    ports = list_serial_ports()
    scored_ports = sorted(
        ((port_priority(port), port.device, port) for port in ports),
        reverse=True,
    )
    positive = [(score, device, port) for score, device, port in scored_ports if score > 0]
    if positive:
        top_score = positive[0][0]
        top_matches = [(device, port) for score, device, port in positive if score == top_score]
        if len(top_matches) == 1:
            return top_matches[0][0], True
        raise RuntimeError(
            "Multiple likely MCU serial ports found; please pass --port explicitly.\n"
            + "\n".join(f"  - {format_port_summary(port)}" for _device, port in top_matches)
        )

    usb_like = [
        port
        for port in ports
        if getattr(port, "vid", None) is not None
        or (getattr(port, "device", "") or "").startswith(("/dev/ttyACM", "/dev/ttyUSB"))
    ]
    if len(usb_like) == 1:
        return usb_like[0].device, True
    if not usb_like:
        raise RuntimeError("No likely MCU serial port found.")
    raise RuntimeError("Multiple serial ports found; please pass --port explicitly.")


class RadarBridge:
    def __init__(
        self,
        port: str,
        baud: int,
        radar_timeout_s: float,
        stop_burst: int,
        vision_link_path: str | None = None,
        radar_link_path: str | None = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.radar_timeout_s = radar_timeout_s
        self.stop_burst = max(1, stop_burst)
        self.selector = selectors.DefaultSelector()
        self.lock_file = None
        self.lock_path = None
        self.serial = None
        self.vision_master = None
        self.radar_master = None
        self.vision_pty_path = ""
        self.radar_pty_path = ""
        self.vision_link_path = None
        self.radar_link_path = None
        self.mcu_parser = McuStreamParser()
        self.radar_parser = RadarCommandParser()
        self.last_radar_rx = 0.0
        self.radar_active = False
        self.latest_radar_cmd = (0.0, 0.0, 0.0)
        self.robot_control_timeout_s = 0.5
        self.robot_control_flags = 0
        self.robot_control_spin_vel = 0.0
        self.robot_control_scan_yaw_rate_deg_s = 0.0
        self.robot_control_search_pitch_deg = float("nan")
        self.last_robot_control_rx = 0.0
        self.last_sx_flags = 0
        self.last_sx_scan_yaw_rate_deg_s = float("nan")
        self.last_sx_search_pitch_deg = float("nan")
        self.last_st_rx = 0.0
        self.last_odom_fallback_pub = 0.0
        self.telemetry_pub = Ros2TelemetryPublisher()
        self.running = True
        vision_slave = None
        radar_slave = None

        try:
            self.lock_file, self.lock_path = acquire_port_lock(self.port)
            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=0)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.vision_master, vision_slave = pty.openpty()
            self.radar_master, radar_slave = pty.openpty()
            configure_pty_slave(vision_slave)
            configure_pty_slave(radar_slave)
            self.vision_pty_path = os.ttyname(vision_slave)
            self.radar_pty_path = os.ttyname(radar_slave)
            self.vision_link_path = install_pty_link(vision_link_path, self.vision_pty_path)
            self.radar_link_path = install_pty_link(radar_link_path, self.radar_pty_path)
            os.close(vision_slave)
            vision_slave = None
            os.close(radar_slave)
            radar_slave = None
            os.set_blocking(self.vision_master, False)
            os.set_blocking(self.radar_master, False)

            self.selector.register(self.serial.fileno(), selectors.EVENT_READ, "serial")
            self.selector.register(self.vision_master, selectors.EVENT_READ, "vision")
            self.selector.register(self.radar_master, selectors.EVENT_READ, "radar")
        except Exception:
            if vision_slave is not None:
                os.close(vision_slave)
            if radar_slave is not None:
                os.close(radar_slave)
            self.close()
            raise

    def close(self) -> None:
        try:
            self.send_stop_burst()
        except Exception:
            pass
        self.running = False
        self.selector.close()
        try:
            if self.serial is not None:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        remove_pty_link(self.vision_link_path, self.vision_pty_path)
        remove_pty_link(self.radar_link_path, self.radar_pty_path)
        for fd_name in ("vision_master", "radar_master"):
            fd = getattr(self, fd_name)
            try:
                if fd is not None:
                    os.close(fd)
            except OSError:
                pass
            setattr(self, fd_name, None)
        try:
            self.telemetry_pub.close()
        except Exception:
            pass
        release_port_lock(self.lock_file, self.lock_path)
        self.lock_file = None

    def print_ports(self) -> None:
        print(f"MCU serial : {self.port}")
        print(f"Vision PTY : {self.vision_pty_path}")
        if self.vision_link_path:
            print(f"Vision link: {self.vision_link_path} -> {self.vision_pty_path}")
        print(f"Radar PTY  : {self.radar_pty_path}")
        if self.radar_link_path:
            print(f"Radar link : {self.radar_link_path} -> {self.radar_pty_path}")
        print(f"Cmd sign   : vx={BRIDGE_VX_SIGN:+.0f}, vy={BRIDGE_VY_SIGN:+.0f}")
        if self.telemetry_pub.available:
            print(
                f"ROS odom   : {self.telemetry_pub.odom_topic} "
                f"({self.telemetry_pub.odom_frame} -> {self.telemetry_pub.base_frame})"
            )
        else:
            print("ROS odom   : disabled (rclpy not importable)")
        print("Press Ctrl+C to stop.")

    def send_serial(self, payload: bytes) -> None:
        if not payload or self.serial is None:
            return
        self.serial.write(payload)

    def current_sx_control(self) -> tuple[int, float, float]:
        if (time.monotonic() - self.last_robot_control_rx) > self.robot_control_timeout_s:
            return 0, 0.0, float("nan")

        return (
            int(self.robot_control_flags),
            float(self.robot_control_scan_yaw_rate_deg_s),
            float(self.robot_control_search_pitch_deg),
        )

    def emit_sx(self, vx: float, vy: float, wz: float) -> None:
        control_flags, scan_yaw_rate_deg_s, search_pitch_deg = self.current_sx_control()
        mcu_vx, mcu_vy, mcu_wz = bridge_to_mcu_velocity(vx, vy, wz)
        frame = build_sx_frame(
            mcu_vx,
            mcu_vy,
            mcu_wz,
            control_flags,
            scan_yaw_rate_deg_s,
            search_pitch_deg,
        )
        self.send_serial(frame)
        if SENTRY_BRIDGE_DEBUG:
            print(
                "[BRIDGE][emit_sx] "
                f"tool=({vx:+.3f},{vy:+.3f},{wz:+.3f}) "
                f"mcu=({mcu_vx:+.3f},{mcu_vy:+.3f},{mcu_wz:+.3f}) "
                f"flags=0x{control_flags:02x} scan_yaw={scan_yaw_rate_deg_s:+.3f} "
                f"search_pitch={search_pitch_deg:+.3f} bytes={frame.hex()}",
                flush=True,
            )
        self.last_sx_flags = control_flags
        self.last_sx_scan_yaw_rate_deg_s = scan_yaw_rate_deg_s
        self.last_sx_search_pitch_deg = search_pitch_deg

    def send_current_sx(self) -> None:
        if self.radar_active:
            vx, vy, wz = self.latest_radar_cmd
        else:
            vx, vy, wz = 0.0, 0.0, 0.0
        self.emit_sx(vx, vy, wz)

    def publish_odom_fallback(self, force: bool = False) -> None:
        if not self.telemetry_pub.available:
            return
        now = time.monotonic()
        if not force and (now - self.last_st_rx) <= 0.2:
            return
        if not force and (now - self.last_odom_fallback_pub) < 0.05:
            return
        if self.radar_active:
            vx, vy, wz = self.latest_radar_cmd
        else:
            vx, vy, wz = 0.0, 0.0, 0.0
        self.telemetry_pub.publish_chassis_odom(float(vx), float(vy), float(wz))
        self.last_odom_fallback_pub = now

    def send_stop_burst(self) -> None:
        self.latest_radar_cmd = (0.0, 0.0, 0.0)
        control_flags, scan_yaw_rate_deg_s, search_pitch_deg = self.current_sx_control()
        stop_frame = build_sx_frame(
            0.0,
            0.0,
            0.0,
            control_flags,
            scan_yaw_rate_deg_s,
            search_pitch_deg,
        )
        self.last_sx_flags = control_flags
        self.last_sx_scan_yaw_rate_deg_s = scan_yaw_rate_deg_s
        self.last_sx_search_pitch_deg = search_pitch_deg
        for _ in range(self.stop_burst):
            self.send_serial(stop_frame)
            self.serial.flush()

    def on_serial_readable(self) -> None:
        chunk = self.serial.read(self.serial.in_waiting or 1)
        if not chunk:
            return
        for kind, payload in self.mcu_parser.feed(chunk):
            if kind == "sp":
                if isinstance(payload, bytes) and len(payload) == SP_TX_SIZE:
                    try:
                        _q, yaw_rad, _yaw_vel, _pitch_rad, _pitch_vel = parse_sp_tx_frame(payload)
                        self.telemetry_pub.publish_body_tf(yaw_rad)
                    except ValueError:
                        pass
                safe_write_fd(self.vision_master, payload if isinstance(payload, bytes) else b"")
            elif kind == "st":
                (
                    cmd_vx,
                    cmd_vy,
                    cmd_wz,
                    real_vx,
                    real_vy,
                    real_wz,
                    _robot_status,
                    game_status,
                    stage_remain_time,
                    robot_id,
                    current_hp,
                    shooter_heat,
                    team_color,
                    is_attacked,
                ) = payload  # type: ignore[misc]
                tool_vx, tool_vy, tool_wz = mcu_to_bridge_velocity(cmd_vx, cmd_vy, cmd_wz)
                odom_vx, odom_vy, odom_wz = mcu_to_bridge_velocity(real_vx, real_vy, real_wz)
                self.last_st_rx = time.monotonic()
                self.telemetry_pub.publish_chassis_odom(odom_vx, odom_vy, odom_wz)
                safe_write_fd(self.radar_master, build_radar_telem_frame(tool_vx, tool_vy, tool_wz))
                safe_write_fd(
                    self.radar_master,
                    build_game_status_frame(int(game_status), int(stage_remain_time)),
                )
                safe_write_fd(
                    self.radar_master,
                    build_robot_status_frame(
                        int(robot_id),
                        int(current_hp),
                        int(shooter_heat),
                        int(team_color),
                        int(is_attacked),
                    ),
                )

    def on_vision_readable(self) -> None:
        try:
            chunk = os.read(self.vision_master, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO, errno.EBADF):
                return
            raise
        if chunk:
            self.send_serial(chunk)

    def on_radar_readable(self) -> None:
        try:
            chunk = os.read(self.radar_master, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO, errno.EBADF):
                if self.radar_active:
                    self.send_stop_burst()
                    self.radar_active = False
                return
            raise
        if not chunk:
            return

        for kind, payload in self.radar_parser.feed(chunk):
            if kind == "radar_cmd":
                vx, vy, wz, _yaw_deg = payload
                if SENTRY_BRIDGE_DEBUG:
                    print(
                        f"[BRIDGE][rx_radar_cmd] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}",
                        flush=True,
                    )
                self.latest_radar_cmd = (vx, vy, wz)
                self.last_radar_rx = time.monotonic()
                self.radar_active = True
                self.send_current_sx()
            elif kind == "robot_control":
                (
                    control_flags,
                    chassis_spin_vel,
                    scan_yaw_rate_deg_s,
                    search_pitch_deg,
                ) = payload
                if SENTRY_BRIDGE_DEBUG:
                    print(
                        "[BRIDGE][rx_robot_control] "
                        f"flags=0x{int(control_flags):02x} spin={float(chassis_spin_vel):+.3f} "
                        f"scan_yaw={float(scan_yaw_rate_deg_s):+.3f} "
                        f"search_pitch={float(search_pitch_deg):+.3f}",
                        flush=True,
                    )
                self.robot_control_flags = int(control_flags)
                self.robot_control_spin_vel = float(chassis_spin_vel)
                self.robot_control_scan_yaw_rate_deg_s = float(scan_yaw_rate_deg_s)
                self.robot_control_search_pitch_deg = float(search_pitch_deg)
                self.last_robot_control_rx = time.monotonic()
                self.send_current_sx()

    def poll_timeout(self) -> None:
        if self.radar_active and (time.monotonic() - self.last_radar_rx) > self.radar_timeout_s:
            self.send_stop_burst()
            self.radar_active = False

        (
            current_flags,
            current_scan_yaw_rate_deg_s,
            current_search_pitch_deg,
        ) = self.current_sx_control()
        if (
            current_flags != self.last_sx_flags
            or not same_config_float(
                current_scan_yaw_rate_deg_s, self.last_sx_scan_yaw_rate_deg_s
            )
            or not same_config_float(
                current_search_pitch_deg, self.last_sx_search_pitch_deg
            )
        ):
            self.send_current_sx()

        self.publish_odom_fallback()

    def run(self) -> None:
        self.print_ports()
        try:
            while self.running:
                events = self.selector.select(timeout=0.05)
                for key, _mask in events:
                    source = key.data
                    if source == "serial":
                        self.on_serial_readable()
                    elif source == "vision":
                        self.on_vision_readable()
                    elif source == "radar":
                        self.on_radar_readable()
                self.poll_timeout()
        finally:
            self.close()


def run_self_test() -> int:
    radar_parser = RadarCommandParser()
    mcu_parser = McuStreamParser()

    cmd = build_radar_cmd_frame(1.25, -0.5, 0.2, 37.0)
    parsed = radar_parser.feed(cmd[:7])
    assert parsed == []
    parsed = radar_parser.feed(cmd[7:])
    assert len(parsed) == 1
    assert parsed[0][0] == "radar_cmd"
    assert all(abs(a - b) < 1e-6 for a, b in zip(parsed[0][1], (1.25, -0.5, 0.2, 37.0)))

    robot_control_flags = (
        SENTRY_EXT_FLAG_SCAN_CONTROL_VALID
        | SENTRY_EXT_FLAG_ALLOW_VISION_CONTROL
        | SENTRY_EXT_FLAG_SEARCH_WHEN_TARGET_LOST
    )
    robot_control = build_robot_control_frame(robot_control_flags, 0.5, 90.0, -8.0)
    parsed_robot_control = parse_robot_control_frame(robot_control)
    assert parsed_robot_control[0] == robot_control_flags
    assert abs(parsed_robot_control[1] - 0.5) < 1e-6
    assert abs(parsed_robot_control[2] - 90.0) < 1e-6
    assert abs(parsed_robot_control[3] + 8.0) < 1e-6
    parsed = radar_parser.feed(robot_control)
    assert len(parsed) == 1
    assert parsed[0][0] == "robot_control"
    assert parsed[0][1][0] == robot_control_flags
    assert abs(parsed[0][1][1] - 0.5) < 1e-6
    assert abs(parsed[0][1][2] - 90.0) < 1e-6
    assert abs(parsed[0][1][3] + 8.0) < 1e-6

    bad_cmd = bytearray(cmd)
    bad_cmd[-1] ^= 0x01
    assert radar_parser.feed(bytes(bad_cmd)) == []

    sx_flags = SENTRY_EXT_FLAG_SCAN_CONTROL_VALID | SENTRY_EXT_FLAG_STOP_GIMBAL_SCAN
    sx = build_sx_frame(0.4, -0.3, 12.0, sx_flags, 120.0, -5.0)
    parsed_sx = parse_sx_frame(sx)
    assert all(abs(a - b) < 1e-6 for a, b in zip(parsed_sx[:5], (0.4, -0.3, 12.0, 0.0, 0.0)))
    assert parsed_sx[5] == sx_flags
    assert abs(parsed_sx[6] - 120.0) < 1e-6
    assert abs(parsed_sx[7] + 5.0) < 1e-6

    assert bridge_to_mcu_velocity(0.8, -0.2, 1200.0) == (0.8, -0.2, 1200.0)
    assert mcu_to_bridge_velocity(0.8, -0.2, 1200.0) == (0.8, -0.2, 1200.0)

    st = build_st_frame(0.8, -0.2, 1200.0, 0.7, -0.1, 1190.0, 0x03, 0x04, 220, 107, 600, 35, 1, 1)
    assert all(
        abs(a - b) < 1e-6
        for a, b in zip(parse_st_frame(st)[:6], (0.8, -0.2, 1200.0, 0.7, -0.1, 1190.0))
    )
    assert parse_st_frame(st)[6:] == (0x03, 0x04, 220, 107, 600, 35, 1, 1)
    game_status_frame = build_game_status_frame(0x04, 220)
    assert len(game_status_frame) == GAME_STATUS_FRAME_SIZE
    assert crc16_x25(game_status_frame[:-2]) == struct.unpack("<H", game_status_frame[-2:])[0]
    robot_status_frame = build_robot_status_frame(107, 600, 35, 1, 1)
    assert len(robot_status_frame) == ROBOT_STATUS_FRAME_SIZE
    assert crc16_x25(robot_status_frame[:-2]) == struct.unpack("<H", robot_status_frame[-2:])[0]

    sp = build_dummy_sp_tx_frame()
    mixed_stream = b"\x00junk" + st[:5] + st[5:] + sp[:11] + sp[11:] + sx[:8] + sx[8:]
    events = mcu_parser.feed(mixed_stream)
    kinds = [kind for kind, _payload in events]
    assert kinds == ["st", "sp", "sx"]
    assert all(
        abs(a - b) < 1e-6 for a, b in zip(events[0][1][:6], (0.8, -0.2, 1200.0, 0.7, -0.1, 1190.0))
    )
    assert events[0][1][6:] == (0x03, 0x04, 220, 107, 600, 35, 1, 1)
    assert events[1][1] == sp
    assert events[2][1] == sx

    telem = build_radar_telem_frame(0.8, -0.2, 1200.0)
    assert len(telem) == RADAR_FRAME_SIZE
    assert telem[:2] == RADAR_TELEM_HEADER
    assert crc8(telem[:-1]) == telem[-1]

    with tempfile.TemporaryDirectory() as temp_dir:
        vision_link = os.path.join(temp_dir, "vision")
        first_target = os.path.join(temp_dir, "pts0")
        second_target = os.path.join(temp_dir, "pts1")
        assert install_pty_link(vision_link, first_target) == os.path.abspath(vision_link)
        assert os.path.islink(vision_link)
        assert os.readlink(vision_link) == first_target
        assert install_pty_link(vision_link, second_target) == os.path.abspath(vision_link)
        assert os.readlink(vision_link) == second_target
        remove_pty_link(vision_link, first_target)
        assert os.path.islink(vision_link)
        remove_pty_link(vision_link, second_target)
        assert not os.path.lexists(vision_link)

    print("sentry_bridge self-test passed")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge one MCU CDC port into separate vision and radar PTYs."
    )
    parser.add_argument(
        "--port",
        help="Real MCU serial port, for example /dev/ttyACM0. Defaults to auto-detect.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate placeholder")
    parser.add_argument(
        "--radar-timeout-ms",
        type=int,
        default=250,
        help="Send zero SX burst if radar side stays silent for this long",
    )
    parser.add_argument(
        "--stop-burst",
        type=int,
        default=8,
        help="How many zero SX frames to send on timeout/disconnect/exit",
    )
    parser.add_argument(
        "--vision-link",
        default=DEFAULT_VISION_LINK,
        help=f"Stable symlink for the vision PTY (default: {DEFAULT_VISION_LINK})",
    )
    parser.add_argument(
        "--radar-link",
        default=DEFAULT_RADAR_LINK,
        help=f"Stable symlink for the radar PTY (default: {DEFAULT_RADAR_LINK})",
    )
    parser.add_argument(
        "--no-links",
        action="store_true",
        help="Do not create stable PTY symlinks",
    )
    parser.add_argument("--self-test", action="store_true", help="Run protocol self-test and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.self_test:
        return run_self_test()

    if os.name != "posix":
        print("sentry_bridge.py currently supports POSIX PTYs only.", file=sys.stderr)
        return 2

    try:
        port, auto_selected = resolve_serial_port(args.port)
    except RuntimeError as exc:
        print(f"sentry_bridge: {exc}", file=sys.stderr)
        print_available_ports(sys.stderr)
        return 2

    if auto_selected:
        print(f"Auto-selected MCU serial port: {port}")

    vision_link = None if args.no_links else args.vision_link
    radar_link = None if args.no_links else args.radar_link

    try:
        bridge = RadarBridge(
            port=port,
            baud=args.baud,
            radar_timeout_s=max(0.01, args.radar_timeout_ms / 1000.0),
            stop_burst=args.stop_burst,
            vision_link_path=vision_link,
            radar_link_path=radar_link,
        )
    except (serial.SerialException, RuntimeError, OSError) as exc:
        print(f"sentry_bridge: failed to start bridge on {port}: {exc}", file=sys.stderr)
        print_available_ports(sys.stderr)
        return 2

    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
