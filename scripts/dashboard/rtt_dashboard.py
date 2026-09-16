#!/usr/bin/env python3
import argparse
import collections
import os
import re
import shutil
import threading
import time
import sys
from pathlib import Path
from typing import Deque, List, Optional

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from scripts.rtt_common.monitor_writer import MonitorWriter
from scripts.rtt_common.rtt_transport import (
    RTTTransport,
    RTTTransportConfig,
    choose_best_channel,
)
from scripts.rtt_common.telemetry import (
    FrameParser,
    TelemetryFrame,
)

try:
    from rich.console import Console, Group
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    RICH_AVAILABLE = True
except Exception:
    RICH_AVAILABLE = False


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def _ansi(code: str, enabled: bool) -> str:
    return f"\x1b[{code}m" if enabled else ""


def _strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def _truncate_ansi(text: str, width: int) -> str:
    if width <= 0:
        return ""
    out: List[str] = []
    vis = 0
    i = 0
    n = len(text)
    while i < n and vis < width:
        if text[i] == "\x1b":
            m = ANSI_RE.match(text, i)
            if m:
                out.append(m.group(0))
                i = m.end()
                continue
        out.append(text[i])
        vis += 1
        i += 1
    return "".join(out)


def _pad_ansi_no_truncate(text: str, width: int) -> str:
    vis = len(_strip_ansi(text))
    if vis >= width:
        return text
    return text + (" " * (width - vis))


class RTTDashboardReader:
    def __init__(
        self,
        telemetry_channel: int,
        log_channel: Optional[int],
        speed_khz: int,
        serial: Optional[str],
        device: str,
        connect_mode: str,
        poll_ms: int,
        backend: str,
        auto_channel: bool,
        elf: str,
        monitor_dir: str,
        monitor_prefix: str,
        monitor_enabled: bool,
    ) -> None:
        self.telemetry_channel = telemetry_channel
        self.log_channel = log_channel
        self.speed_khz = speed_khz
        self.serial = serial
        self.device = device
        self.connect_mode = connect_mode
        self.poll_ms = poll_ms
        self.backend = backend
        self.auto_channel = auto_channel
        self.elf = elf

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frames: Deque[TelemetryFrame] = collections.deque(maxlen=512)
        self._latest: Optional[TelemetryFrame] = None
        self._log_lines: Deque[str] = collections.deque(maxlen=1000)
        self._log_partial = ""

        self.frames_total = 0
        self.bytes_total = 0
        self.log_bytes_total = 0
        self.connect_error: Optional[str] = None
        self.channel_info: str = ""
        self.backend_in_use: str = ""
        self.parser = FrameParser()
        self.clock_offset_ms: Optional[int] = None
        self.drop_base: Optional[int] = None
        self.monitor_writer: Optional[MonitorWriter] = None
        self.monitor_path: Optional[Path] = None
        self.monitor_error: Optional[str] = None
        if monitor_enabled:
            try:
                self.monitor_writer = MonitorWriter.create(
                    directory=monitor_dir,
                    prefix=monitor_prefix,
                )
                self.monitor_path = self.monitor_writer.path
            except Exception as exc:  # pylint: disable=broad-except
                self.monitor_error = str(exc)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=False)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join()

    def drain_frames(self) -> List[TelemetryFrame]:
        with self._lock:
            items = list(self._frames)
            self._frames.clear()
        return items

    def latest(self) -> Optional[TelemetryFrame]:
        with self._lock:
            return self._latest

    def tail_logs(self, count: int) -> List[str]:
        with self._lock:
            return list(self._log_lines)[-count:]

    def _append_log_bytes(self, data: bytes) -> None:
        if not data:
            return
        text = data.decode("utf-8", errors="replace")
        chunk = self._log_partial + text
        parts = chunk.splitlines(keepends=True)
        complete: List[str] = []

        for part in parts:
            if part.endswith("\n") or part.endswith("\r"):
                complete.append(part.rstrip("\r\n"))
            else:
                self._log_partial = part
                break
        else:
            self._log_partial = ""

        if complete:
            with self._lock:
                for line in complete:
                    self._log_lines.append(line)

    def _choose_telemetry_channel(self, transport: RTTTransport) -> int:
        channels = transport.available_channels()
        if channels:
            self.channel_info = ", ".join(channels)

        if not self.auto_channel:
            return self.telemetry_channel

        for item in channels:
            if ":" in item:
                idx_text, name = item.split(":", 1)
                if name.strip().lower() == "dashboard":
                    try:
                        return int(idx_text)
                    except ValueError:
                        continue

        candidates: List[int] = []
        if channels:
            for item in channels:
                idx_text = item.split(":", 1)[0]
                try:
                    candidates.append(int(idx_text))
                except ValueError:
                    pass
        if not candidates:
            candidates = [self.telemetry_channel, 1, 0, 2, 3]

        parsers = {idx: FrameParser() for idx in candidates}

        def score_fn(idx: int, data: bytes) -> int:
            frames = parsers[idx].feed(data, int(time.time() * 1000))
            return len(data) + (len(frames) * 2000) + parsers[idx].magic_hit_count

        return choose_best_channel(transport, candidates, score_fn)

    def _run(self) -> None:
        transport = RTTTransport(
            RTTTransportConfig(
                device=self.device,
                serial=self.serial,
                speed_khz=self.speed_khz,
                connect_mode=self.connect_mode,
                backend=self.backend,
                elf=self.elf,
            )
        )

        try:
            transport.open()
            self.backend_in_use = transport.backend_in_use
            self.telemetry_channel = self._choose_telemetry_channel(transport)
            if self.channel_info:
                self.channel_info = (
                    f"{self.channel_info} using={self.telemetry_channel}"
                    f" backend={transport.backend_in_use}"
                )

            while not self._stop_event.is_set():
                data = transport.read(self.telemetry_channel, 4096)
                if data:
                    host_rx_ms = int(time.time() * 1000)
                    frames = self.parser.feed(data, host_rx_ms)
                    if self.monitor_writer is not None:
                        self.monitor_writer.write_frames(frames)
                    with self._lock:
                        self.bytes_total += len(data)
                        self.frames_total += len(frames)
                        for frame in frames:
                            if self.clock_offset_ms is None:
                                self.clock_offset_ms = (
                                    frame.host_rx_ms - frame.timestamp_ms
                                )
                            self._frames.append(frame)
                            self._latest = frame

                if self.log_channel is not None:
                    log_data = transport.read(self.log_channel, 2048)
                    if log_data:
                        self.log_bytes_total += len(log_data)
                        self._append_log_bytes(log_data)

                time.sleep(max(self.poll_ms, 1) / 1000.0)
        except Exception as exc:  # pylint: disable=broad-except
            self.connect_error = str(exc)
        finally:
            if self.monitor_writer is not None:
                self.monitor_writer.close()
            transport.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RTT dashboard/log console for NYUSH RoboMaster firmware"
    )
    parser.add_argument("--device", default="STM32F407IG", help="Target device name")
    parser.add_argument("--serial", default=None, help="Probe serial number (optional)")
    parser.add_argument(
        "--elf",
        default=os.path.join("build", "basic_framework.elf"),
        help="ELF file for RTT symbol lookup",
    )
    parser.add_argument(
        "--backend",
        choices=["auto", "pyocd", "jlink"],
        default="auto",
        help="Transport backend",
    )
    parser.add_argument(
        "--channel", type=int, default=1, help="RTT up channel for telemetry"
    )
    parser.add_argument(
        "--log-channel", type=int, default=0, help="RTT up channel for text logs"
    )
    parser.add_argument(
        "--mode",
        choices=["dashboard", "split"],
        default="dashboard",
        help="Render mode",
    )
    parser.add_argument(
        "--no-auto-channel",
        action="store_true",
        help="Disable telemetry channel auto detection",
    )
    parser.add_argument("--speed", type=int, default=4000, help="SWD speed in kHz")
    parser.add_argument(
        "--connect",
        choices=["normal", "halt", "under-reset"],
        default="normal",
        help="pyOCD connect mode",
    )
    parser.add_argument("--poll-ms", type=int, default=20, help="RTT poll interval")
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=100,
        help="Terminal refresh interval",
    )
    parser.add_argument(
        "--monitor-dir",
        default=os.path.join("monitor"),
        help="Directory for exported monitor txt files",
    )
    parser.add_argument(
        "--monitor-prefix",
        default="logger_cli",
        help="Filename prefix for exported monitor txt files",
    )
    parser.add_argument(
        "--no-monitor-save",
        action="store_true",
        help="Disable monitor txt export",
    )
    return parser.parse_args()


def _build_dashboard_lines(
    reader: RTTDashboardReader,
    color_enabled: bool,
    cols: int,
    rows: int,
    compact_header: bool = False,
) -> List[str]:
    C_DIVIDER = "95"
    C_OK = "92"
    C_WARN = "93"
    C_ERR = "91"
    C_SECTION_A = "94"
    C_SECTION_B = "96"

    def style(text: str, color: str) -> str:
        return f"{_ansi(color, color_enabled)}{text}{_ansi('0', color_enabled)}"

    def fnum(value: float, digits: int) -> str:
        text = f"{value:.{digits}f}"
        if "." in text:
            text = text.rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text

    def bit(value: bool, err_on_true: bool = False) -> str:
        if value:
            return style("1", C_ERR if err_on_true else C_OK)
        return style("0", C_OK if err_on_true else C_ERR)

    def short_chassis_mode(mode: int) -> str:
        return {
            0: "ZF",
            1: "ROT",
            2: "NF",
            3: "FGY",
        }.get(mode, f"?{mode}")

    def short_gimbal_mode(mode: int) -> str:
        return {
            0: "ZF",
            1: "FREE",
            2: "GY",
        }.get(mode, f"?{mode}")

    def short_remote_protocol(protocol: int) -> str:
        return {
            0: "UNK",
            1: "DBUS",
            2: "VT",
        }.get(protocol, f"?{protocol}")

    def short_mode_switch(raw_mode: int) -> str:
        return {
            0: "C",
            1: "N",
            2: "S",
            0xFF: "NA",
        }.get(raw_mode, f"?{raw_mode}")

    def short_bullet_speed(speed: int) -> str:
        return {
            0: "N",
            10: "B10",
            15: "S15",
            16: "B16",
            18: "S18",
            30: "S30",
        }.get(speed, f"?{speed}")

    def pack(values: List[str]) -> str:
        return "/".join(values)

    def kv(label: str, value: str) -> tuple[str, str]:
        return label, value

    latest = reader.latest()
    now_ms = int(time.time() * 1000)
    sep = f"{_ansi(C_DIVIDER, color_enabled)}{'-' * cols}{_ansi('0', color_enabled)}"
    lines: List[str] = []

    if reader.connect_error:
        lines.append(f"{style('ERROR', C_ERR)} {reader.connect_error}")
    if latest is None:
        lines.append(f"{style('WAIT', C_WARN)} valid RTT dashboard frames...")
        lines.append(
            _truncate_ansi(
                " ".join(
                    [
                        f"bytes={reader.bytes_total}",
                        f"frames={reader.frames_total}",
                        f"crc={reader.parser.crc_error_count}",
                        f"sync={reader.parser.sync_drop_count}",
                        f"magic={reader.parser.magic_hit_count}",
                        f"hdr={reader.parser.header_mismatch_count}",
                    ]
                ),
                cols,
            )
        )
        if reader.channel_info:
            lines.append(_truncate_ansi(f"chs={reader.channel_info}", cols))
        if reader.parser.last_bad_header:
            lines.append(_truncate_ansi(f"bad_hdr={reader.parser.last_bad_header}", cols))
        if reader.parser.last_bad_reason:
            lines.append(_truncate_ansi(f"bad_reason={reader.parser.last_bad_reason}", cols))
        return lines

    latency_ms = 0
    if reader.clock_offset_ms is not None:
        latency_ms = max(0, now_ms - int(latest.timestamp_ms) - reader.clock_offset_ms)
    frame_age_ms = max(0, now_ms - latest.host_rx_ms)

    if reader.drop_base is None:
        reader.drop_base = latest.telemetry_drop_count
    drop_since_attach = latest.telemetry_drop_count - reader.drop_base
    sw_l = latest.rc_switches & 0x03
    sw_r = (latest.rc_switches >> 2) & 0x03
    remote_flags = latest.remote_control_flags
    rpm_err = [
        latest.motor_target_rpm[idx] - latest.motor_rpm[idx] for idx in range(4)
    ]
    ref_on = (latest.status_flags & 0x01) != 0
    remote_on = (latest.status_flags & 0x02) != 0
    imu_on = (latest.status_flags & 0x04) != 0
    gimbal_msg_on = (latest.status_flags & 0x08) != 0
    chassis_msg_on = (latest.status_flags & 0x10) != 0
    gimbal_cmd_on = (latest.status_flags & 0x20) != 0
    chassis_cmd_on = (latest.status_flags & 0x40) != 0
    shoot_msg_on = (latest.status_flags & 0x80) != 0
    live_color = C_OK if frame_age_ms < 200 else C_WARN

    header_main = (
        f"{style('LIVE', live_color)} "
        f"age={frame_age_ms}ms lat={latency_ms}ms "
        f"seq={latest.seq} v/len={latest.version}/{latest.payload_size} "
        f"fr={reader.frames_total} bytes={reader.bytes_total}"
    )
    header_diag_parts = [
        f"diag c/s/m/h={reader.parser.crc_error_count}/{reader.parser.sync_drop_count}/{reader.parser.magic_hit_count}/{reader.parser.header_mismatch_count}",
        f"ch={reader.telemetry_channel}",
        f"heap={latest.free_heap_bytes}",
        f"drop={latest.telemetry_drop_count}(+{drop_since_attach})",
    ]
    if reader.backend_in_use:
        header_diag_parts.append(f"be={reader.backend_in_use}")
    if reader.channel_info and not compact_header:
        header_diag_parts.append(f"chs={reader.channel_info}")

    lines.append(_truncate_ansi(header_main, cols))
    lines.append(_truncate_ansi(" ".join(header_diag_parts), cols))
    lines.append(sep)

    sections = [
        (
            "System",
            C_SECTION_A,
            [
                kv("proto", f"v{latest.version} / {latest.payload_size}B / seq {latest.seq}"),
                kv("age/lat", f"{frame_age_ms}ms / {latency_ms}ms"),
                kv("time/rx", f"{latest.timestamp_ms} / {latest.host_rx_ms}"),
                kv("power", f"{fnum(latest.chassis_volt, 2)}V / {fnum(latest.referee_chassis_current * 0.001, 2)}A / {fnum(latest.chassis_power, 2)}W"),
                kv("heap/drop", f"{latest.free_heap_bytes} / {latest.telemetry_drop_count} (+{drop_since_attach})"),
            ],
        ),
        (
            "Referee",
            C_SECTION_A,
            [
                kv("game", f"0x{latest.referee_game_state_raw:02x} / type {latest.referee_game_type} / prog {latest.referee_status}"),
                kv("stage", f"{latest.referee_stage_remain_time}s / id {latest.referee_robot_id} / lv {latest.referee_robot_level}"),
                kv("hp", f"{latest.referee_current_hp} / {latest.referee_maximum_hp}"),
                kv("cool/heat", f"{latest.referee_shooter_barrel_cooling_value} / {latest.referee_shooter_barrel_heat_limit}"),
                kv("power lim", f"{latest.referee_chassis_power_limit}W"),
                kv("curr/buf", f"{fnum(latest.referee_chassis_current * 0.001, 2)}A / {latest.referee_buffer_energy}"),
                kv("pwr ctl", f"{latest.chassis_power_buffer_energy} / {fnum(latest.chassis_power_budget_w, 2)} / {fnum(latest.chassis_power_estimated_w, 2)} / {fnum(latest.chassis_power_scale, 3)} / 0x{latest.chassis_power_mode_flags:02x}"),
                kv("pm g/c/s", f"{bit((latest.referee_power_management_flags & 0x01) != 0)} / {bit((latest.referee_power_management_flags & 0x02) != 0)} / {bit((latest.referee_power_management_flags & 0x04) != 0)}"),
                kv("barrel", f"{latest.referee_shooter_17mm_1_barrel_heat} / {latest.referee_shooter_17mm_2_barrel_heat} / {latest.referee_shooter_42mm_barrel_heat}"),
                kv("shot spd", f"{fnum(latest.referee_shoot_bullet_speed_mps, 2)}m/s"),
            ],
        ),
        (
            "Command",
            C_SECTION_A,
            [
                kv("chassis", pack([fnum(latest.chassis_cmd_vx, 2), fnum(latest.chassis_cmd_vy, 2), fnum(latest.chassis_cmd_wz, 2)])),
                kv("gimbal", pack([fnum(latest.gimbal_cmd_yaw_deg, 1), fnum(latest.gimbal_cmd_pitch_deg, 1), fnum(latest.gimbal_cmd_chassis_rotate_wz, 2)])),
                kv("mode", f"{short_chassis_mode(latest.chassis_mode)} / {short_gimbal_mode(latest.gimbal_mode)}"),
                kv("shoot", f"{latest.shoot_mode} / {latest.shoot_load_mode} / {latest.shoot_lid_mode} / {latest.shoot_friction_mode}"),
                kv("bullet", f"{latest.shoot_bullet_speed} ({short_bullet_speed(latest.shoot_bullet_speed)})"),
                kv("rest/rate", f"{latest.shoot_rest_heat} / {fnum(latest.shoot_rate, 2)}Hz"),
            ],
        ),
        (
            "Motors",
            C_SECTION_A,
            [
                kv("target", pack([fnum(v, 0) for v in latest.motor_target_rpm])),
                kv("actual", pack([fnum(v, 0) for v in latest.motor_rpm])),
                kv("error", pack([fnum(v, 0) for v in rpm_err])),
                kv("bit ch/gm", f"0x{latest.motor_online_bitmap:02x} / 0x{latest.gimbal_motor_online_bitmap:02x}"),
                kv("bit sh/cn", f"0x{latest.shoot_motor_online_bitmap:02x} / 0x{latest.can_link_bitmap:02x}"),
                kv("online", f"g {bit((latest.gimbal_motor_online_bitmap & 0x01) != 0)}/{bit((latest.gimbal_motor_online_bitmap & 0x02) != 0)}  s {bit((latest.shoot_motor_online_bitmap & 0x01) != 0)}/{bit((latest.shoot_motor_online_bitmap & 0x02) != 0)}/{bit((latest.shoot_motor_online_bitmap & 0x04) != 0)}  c {bit((latest.can_link_bitmap & 0x01) != 0)}/{bit((latest.can_link_bitmap & 0x02) != 0)}"),
            ],
        ),
        (
            "Gimbal",
            C_SECTION_B,
            [
                kv("yaw a/v", f"{fnum(latest.gimbal_yaw_target_deg, 1)} / {fnum(latest.gimbal_yaw_actual_deg, 1)} @ {fnum(latest.gimbal_yaw_target_deg_s, 1)} / {fnum(latest.gimbal_yaw_actual_deg_s, 1)}"),
                kv("pit a/v", f"{fnum(latest.gimbal_pitch_target_deg, 1)} / {fnum(latest.gimbal_pitch_actual_deg, 1)} @ {fnum(latest.gimbal_pitch_target_deg_s, 1)} / {fnum(latest.gimbal_pitch_actual_deg_s, 1)}"),
                kv("enc raw", f"{latest.gimbal_yaw_encoder_raw} / {latest.gimbal_pitch_encoder_raw}"),
                kv("imu ang", pack([fnum(v, 1) for v in latest.imu_angle_deg])),
                kv("imu gyr", pack([fnum(v, 1) for v in latest.imu_gyro_deg_s])),
                kv("shoot aps", pack([fnum(latest.shoot_loader_speed_aps, 2), fnum(latest.shoot_friction_l_speed_aps, 2), fnum(latest.shoot_friction_r_speed_aps, 2)])),
            ],
        ),
        (
            "Remote",
            C_SECTION_B,
            [
                kv("proto", f"{short_remote_protocol(latest.remote_protocol)} / mode {short_mode_switch(latest.remote_mode_switch)} / sw 0x{latest.rc_switches:02x}"),
                kv("switch", f"L {sw_l} / R {sw_r}"),
                kv("btn/flag", f"0x{latest.remote_button_bits:02x} / 0x{latest.remote_control_flags:02x}"),
                kv("vision", f"{bit((remote_flags & 0x01) != 0)} / kbm {bit((remote_flags & 0x02) != 0)} / estop {bit((remote_flags & 0x04) != 0, err_on_true=True)}"),
                kv("sticks", f"{latest.rc_rocker_l_x} / {latest.rc_rocker_l_y} / {latest.rc_rocker_r_x} / {latest.rc_rocker_r_y}"),
                kv("dial", str(latest.rc_dial)),
                kv("mouse", f"{latest.mouse_x} / {latest.mouse_y} / {latest.mouse_z}"),
                kv("mouse btn", f"{bit(latest.mouse_left != 0)} / {bit(latest.mouse_right != 0)} / {bit(((latest.remote_button_bits >> 4) & 0x01) != 0)}"),
                kv("keys", f"0x{latest.key_pressed_bits:04x}"),
            ],
        ),
        (
            "Status",
            C_SECTION_B,
            [
                kv("bits", f"0x{latest.status_flags:02x}"),
                kv("ref/rc/imu", f"{bit(ref_on)} / {bit(remote_on)} / {bit(imu_on)}"),
                kv("feed", f"g {bit(gimbal_msg_on)} / c {bit(chassis_msg_on)} / s {bit(shoot_msg_on)}"),
                kv("cmd", f"g {bit(gimbal_cmd_on)} / c {bit(chassis_cmd_on)}"),
            ],
        ),
        (
            "Vision",
            C_SECTION_B,
            [
                kv("meta", f"0x{latest.vision_meta_flags:02x} / mode {latest.vision_recv_sp_mode} / {latest.vision_send_sp_mode}"),
                kv("recv yaw", pack([fnum(latest.vision_recv_yaw_raw_rad, 3), fnum(latest.vision_recv_yaw_vel_raw_rad_s, 3), fnum(latest.vision_recv_yaw_acc_raw_rad_s2, 3)])),
                kv("recv pit", pack([fnum(latest.vision_recv_pitch_raw_rad, 3), fnum(latest.vision_recv_pitch_vel_raw_rad_s, 3), fnum(latest.vision_recv_pitch_acc_raw_rad_s2, 3)])),
                kv("send q", pack([fnum(v, 3) for v in latest.vision_send_q])),
                kv("send ang", f"{fnum(latest.vision_send_yaw_raw_rad, 3)} / {fnum(latest.vision_send_pitch_raw_rad, 3)}"),
                kv("send vel", f"{fnum(latest.vision_send_yaw_vel_raw_rad_s, 3)} / {fnum(latest.vision_send_pitch_vel_raw_rad_s, 3)}"),
                kv("bullet", f"{fnum(latest.vision_send_bullet_speed_mps, 2)} / {latest.vision_send_bullet_count}"),
            ],
        ),
    ]

    body_height = max(6, rows - len(lines))
    layout_candidates: List[List[List[int]]] = []
    if cols >= 80:
        layout_candidates.append([[0, 1, 6], [2, 3, 4], [5, 7]])
    if cols >= 58:
        layout_candidates.append([[0, 1, 6, 5], [2, 3, 4, 7]])
    layout_candidates.append([[0, 1, 2, 3, 4, 5, 6, 7]])

    best_layout = layout_candidates[-1]
    best_width = cols
    best_overflow: Optional[int] = None
    min_col_width = 26

    for layout in layout_candidates:
        column_count = len(layout)
        column_width = (cols - (column_count - 1)) // column_count
        if column_width < min_col_width:
            continue
        heights = [
            sum(1 + len(sections[section_idx][2]) for section_idx in column)
            for column in layout
        ]
        overflow = max(heights) - body_height
        if overflow <= 0:
            best_layout = layout
            best_width = column_width
            best_overflow = overflow
            break
        if best_overflow is None or overflow < best_overflow:
            best_layout = layout
            best_width = column_width
            best_overflow = overflow

    label_width = 8 if best_width < 34 else 10
    column_lines: List[List[str]] = []
    for column in best_layout:
        rendered: List[str] = []
        for section_idx in column:
            title, color, section_items = sections[section_idx]
            rendered.append(style(f"[{title}]", color))
            for label, value in section_items:
                rendered.append(f"{label:<{label_width}}: {value}")
        column_lines.append(rendered)

    body_lines: List[str] = []
    divider = f"{_ansi(C_DIVIDER, color_enabled)}|{_ansi('0', color_enabled)}"
    row_count = max(len(column) for column in column_lines)
    for row_idx in range(row_count):
        parts: List[str] = []
        for col_idx, column in enumerate(column_lines):
            line = column[row_idx] if row_idx < len(column) else ""
            line = _pad_ansi_no_truncate(_truncate_ansi(line, best_width), best_width)
            parts.append(line)
            if col_idx != len(column_lines) - 1:
                parts.append(divider)
        body_lines.append("".join(parts))

    if best_overflow is not None and best_overflow > 0:
        body_lines.append(
            _truncate_ansi(
                f"{style('NOTE', C_WARN)} terminal height is tight; widen or raise rows for zero-truncation safety.",
                cols,
            )
        )
    return lines + body_lines


def _build_dashboard_renderable(
    reader: RTTDashboardReader,
    cols: int,
    rows: int,
    compact_header: bool = False,
):
    def fnum(value: float, digits: int) -> str:
        text = f"{value:.{digits}f}"
        if "." in text:
            text = text.rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text

    def short_chassis_mode(mode: int) -> str:
        return {
            0: "ZF",
            1: "ROT",
            2: "NF",
            3: "FGY",
        }.get(mode, f"?{mode}")

    def short_gimbal_mode(mode: int) -> str:
        return {
            0: "ZF",
            1: "FREE",
            2: "GY",
        }.get(mode, f"?{mode}")

    def short_remote_protocol(protocol: int) -> str:
        return {
            0: "UNK",
            1: "DBUS",
            2: "VT",
        }.get(protocol, f"?{protocol}")

    def short_mode_switch(raw_mode: int) -> str:
        return {
            0: "C",
            1: "N",
            2: "S",
            0xFF: "NA",
        }.get(raw_mode, f"?{raw_mode}")

    def short_bullet_speed(speed: int) -> str:
        return {
            0: "N",
            10: "B10",
            15: "S15",
            16: "B16",
            18: "S18",
            30: "S30",
        }.get(speed, f"?{speed}")

    def pack(values: List[str]) -> str:
        return "/".join(values)

    def kv(label: str, value):
        return label, value

    def rbit(value: bool, err_on_true: bool = False) -> Text:
        if value:
            return Text("1", style="bold red" if err_on_true else "bold green")
        return Text("0", style="bold green" if err_on_true else "bold red")

    def rjoin(*parts) -> Text:
        text = Text()
        for part in parts:
            if isinstance(part, Text):
                text.append_text(part)
            else:
                text.append(str(part))
        return text

    def make_panel(title: str, border_style: str, items: List[tuple[str, object]], label_width: int) -> Panel:
        table = Table.grid(expand=True, padding=(0, 1))
        table.add_column(style="bold cyan", no_wrap=True, width=label_width)
        table.add_column(style="white", ratio=1)
        for label, value in items:
            render_value = value if isinstance(value, Text) else Text(str(value))
            table.add_row(label, render_value)
        return Panel(
            table,
            title=title,
            border_style=border_style,
            padding=(0, 1),
        )

    latest = reader.latest()
    now_ms = int(time.time() * 1000)

    if reader.connect_error:
        return Panel(Text(reader.connect_error, style="bold red"), title="Error", border_style="red")
    if latest is None:
        waiting = Table.grid(expand=True)
        waiting.add_column(style="white")
        waiting.add_row("Waiting for valid RTT dashboard frames...")
        waiting.add_row(
            f"bytes={reader.bytes_total} frames={reader.frames_total} "
            f"crc={reader.parser.crc_error_count} sync={reader.parser.sync_drop_count} "
            f"magic={reader.parser.magic_hit_count} hdr={reader.parser.header_mismatch_count}"
        )
        if reader.channel_info:
            waiting.add_row(f"chs={reader.channel_info}")
        if reader.parser.last_bad_header:
            waiting.add_row(f"bad_hdr={reader.parser.last_bad_header}")
        if reader.parser.last_bad_reason:
            waiting.add_row(f"bad_reason={reader.parser.last_bad_reason}")
        return Panel(waiting, title="RTT Dashboard", border_style="yellow")

    latency_ms = 0
    if reader.clock_offset_ms is not None:
        latency_ms = max(0, now_ms - int(latest.timestamp_ms) - reader.clock_offset_ms)
    frame_age_ms = max(0, now_ms - latest.host_rx_ms)
    live_style = "bold green" if frame_age_ms < 200 else "bold yellow"

    if reader.drop_base is None:
        reader.drop_base = latest.telemetry_drop_count
    drop_since_attach = latest.telemetry_drop_count - reader.drop_base
    sw_l = latest.rc_switches & 0x03
    sw_r = (latest.rc_switches >> 2) & 0x03
    remote_flags = latest.remote_control_flags
    rpm_err = [
        latest.motor_target_rpm[idx] - latest.motor_rpm[idx] for idx in range(4)
    ]
    ref_on = (latest.status_flags & 0x01) != 0
    remote_on = (latest.status_flags & 0x02) != 0
    imu_on = (latest.status_flags & 0x04) != 0
    gimbal_msg_on = (latest.status_flags & 0x08) != 0
    chassis_msg_on = (latest.status_flags & 0x10) != 0
    gimbal_cmd_on = (latest.status_flags & 0x20) != 0
    chassis_cmd_on = (latest.status_flags & 0x40) != 0
    shoot_msg_on = (latest.status_flags & 0x80) != 0

    header_grid = Table.grid(expand=True, padding=(0, 1))
    header_grid.add_column(ratio=1)
    header_grid.add_column(ratio=1)
    header_grid.add_row(
        Text(
            f"LIVE age={frame_age_ms}ms lat={latency_ms}ms seq={latest.seq} "
            f"v/len={latest.version}/{latest.payload_size} fr={reader.frames_total} bytes={reader.bytes_total}",
            style=live_style,
        ),
        Text(
            f"diag c/s/m/h={reader.parser.crc_error_count}/{reader.parser.sync_drop_count}/{reader.parser.magic_hit_count}/{reader.parser.header_mismatch_count} "
            f"ch={reader.telemetry_channel} heap={latest.free_heap_bytes} drop={latest.telemetry_drop_count}(+{drop_since_attach})",
            style="dim",
        ),
    )
    if reader.channel_info and not compact_header:
        header_grid.add_row(
            Text(f"backend={reader.backend_in_use or 'n/a'}", style="dim"),
            Text(f"channels={reader.channel_info}", style="dim"),
        )

    sections = [
        (
            "System",
            "cyan",
            [
                kv("proto", f"v{latest.version} / {latest.payload_size}B / seq {latest.seq}"),
                kv("age/lat", f"{frame_age_ms}ms / {latency_ms}ms"),
                kv("time/rx", f"{latest.timestamp_ms} / {latest.host_rx_ms}"),
                kv("power", f"{fnum(latest.chassis_volt, 2)}V / {fnum(latest.referee_chassis_current * 0.001, 2)}A / {fnum(latest.chassis_power, 2)}W"),
                kv("heap/drop", f"{latest.free_heap_bytes} / {latest.telemetry_drop_count} (+{drop_since_attach})"),
            ],
        ),
        (
            "Referee",
            "cyan",
            [
                kv("game", f"0x{latest.referee_game_state_raw:02x} / type {latest.referee_game_type} / prog {latest.referee_status}"),
                kv("stage", f"{latest.referee_stage_remain_time}s / id {latest.referee_robot_id} / lv {latest.referee_robot_level}"),
                kv("hp", f"{latest.referee_current_hp} / {latest.referee_maximum_hp}"),
                kv("cool/heat", f"{latest.referee_shooter_barrel_cooling_value} / {latest.referee_shooter_barrel_heat_limit}"),
                kv("power lim", f"{latest.referee_chassis_power_limit}W"),
                kv("curr/buf", f"{fnum(latest.referee_chassis_current * 0.001, 2)}A / {latest.referee_buffer_energy}"),
                kv("pwr ctl", f"{latest.chassis_power_buffer_energy} / {fnum(latest.chassis_power_budget_w, 2)} / {fnum(latest.chassis_power_estimated_w, 2)} / {fnum(latest.chassis_power_scale, 3)} / 0x{latest.chassis_power_mode_flags:02x}"),
                kv("pm g/c/s", rjoin(rbit((latest.referee_power_management_flags & 0x01) != 0), " / ", rbit((latest.referee_power_management_flags & 0x02) != 0), " / ", rbit((latest.referee_power_management_flags & 0x04) != 0))),
                kv("barrel", f"{latest.referee_shooter_17mm_1_barrel_heat} / {latest.referee_shooter_17mm_2_barrel_heat} / {latest.referee_shooter_42mm_barrel_heat}"),
                kv("shot spd", f"{fnum(latest.referee_shoot_bullet_speed_mps, 2)}m/s"),
            ],
        ),
        (
            "Command",
            "cyan",
            [
                kv("chassis", pack([fnum(latest.chassis_cmd_vx, 2), fnum(latest.chassis_cmd_vy, 2), fnum(latest.chassis_cmd_wz, 2)])),
                kv("gimbal", pack([fnum(latest.gimbal_cmd_yaw_deg, 1), fnum(latest.gimbal_cmd_pitch_deg, 1), fnum(latest.gimbal_cmd_chassis_rotate_wz, 2)])),
                kv("mode", f"{short_chassis_mode(latest.chassis_mode)} / {short_gimbal_mode(latest.gimbal_mode)}"),
                kv("shoot", f"{latest.shoot_mode} / {latest.shoot_load_mode} / {latest.shoot_lid_mode} / {latest.shoot_friction_mode}"),
                kv("bullet", f"{latest.shoot_bullet_speed} ({short_bullet_speed(latest.shoot_bullet_speed)})"),
                kv("rest/rate", f"{latest.shoot_rest_heat} / {fnum(latest.shoot_rate, 2)}Hz"),
            ],
        ),
        (
            "Motors",
            "cyan",
            [
                kv("target", pack([fnum(v, 0) for v in latest.motor_target_rpm])),
                kv("actual", pack([fnum(v, 0) for v in latest.motor_rpm])),
                kv("error", pack([fnum(v, 0) for v in rpm_err])),
                kv("bit ch/gm", f"0x{latest.motor_online_bitmap:02x} / 0x{latest.gimbal_motor_online_bitmap:02x}"),
                kv("bit sh/cn", f"0x{latest.shoot_motor_online_bitmap:02x} / 0x{latest.can_link_bitmap:02x}"),
                kv("online", rjoin("g ", rbit((latest.gimbal_motor_online_bitmap & 0x01) != 0), "/", rbit((latest.gimbal_motor_online_bitmap & 0x02) != 0), "  s ", rbit((latest.shoot_motor_online_bitmap & 0x01) != 0), "/", rbit((latest.shoot_motor_online_bitmap & 0x02) != 0), "/", rbit((latest.shoot_motor_online_bitmap & 0x04) != 0), "  c ", rbit((latest.can_link_bitmap & 0x01) != 0), "/", rbit((latest.can_link_bitmap & 0x02) != 0))),
            ],
        ),
        (
            "Gimbal",
            "green",
            [
                kv("yaw a/v", f"{fnum(latest.gimbal_yaw_target_deg, 1)} / {fnum(latest.gimbal_yaw_actual_deg, 1)} @ {fnum(latest.gimbal_yaw_target_deg_s, 1)} / {fnum(latest.gimbal_yaw_actual_deg_s, 1)}"),
                kv("pit a/v", f"{fnum(latest.gimbal_pitch_target_deg, 1)} / {fnum(latest.gimbal_pitch_actual_deg, 1)} @ {fnum(latest.gimbal_pitch_target_deg_s, 1)} / {fnum(latest.gimbal_pitch_actual_deg_s, 1)}"),
                kv("enc raw", f"{latest.gimbal_yaw_encoder_raw} / {latest.gimbal_pitch_encoder_raw}"),
                kv("imu ang", pack([fnum(v, 1) for v in latest.imu_angle_deg])),
                kv("imu gyr", pack([fnum(v, 1) for v in latest.imu_gyro_deg_s])),
                kv("shoot aps", pack([fnum(latest.shoot_loader_speed_aps, 2), fnum(latest.shoot_friction_l_speed_aps, 2), fnum(latest.shoot_friction_r_speed_aps, 2)])),
            ],
        ),
        (
            "Remote",
            "green",
            [
                kv("proto", f"{short_remote_protocol(latest.remote_protocol)} / mode {short_mode_switch(latest.remote_mode_switch)} / sw 0x{latest.rc_switches:02x}"),
                kv("switch", f"L {sw_l} / R {sw_r}"),
                kv("btn/flag", f"0x{latest.remote_button_bits:02x} / 0x{latest.remote_control_flags:02x}"),
                kv("vision", rjoin(rbit((remote_flags & 0x01) != 0), " / kbm ", rbit((remote_flags & 0x02) != 0), " / estop ", rbit((remote_flags & 0x04) != 0, err_on_true=True))),
                kv("sticks", f"{latest.rc_rocker_l_x} / {latest.rc_rocker_l_y} / {latest.rc_rocker_r_x} / {latest.rc_rocker_r_y}"),
                kv("dial", str(latest.rc_dial)),
                kv("mouse", f"{latest.mouse_x} / {latest.mouse_y} / {latest.mouse_z}"),
                kv("mouse btn", rjoin(rbit(latest.mouse_left != 0), " / ", rbit(latest.mouse_right != 0), " / ", rbit(((latest.remote_button_bits >> 4) & 0x01) != 0))),
                kv("keys", f"0x{latest.key_pressed_bits:04x}"),
            ],
        ),
        (
            "Status",
            "green",
            [
                kv("bits", f"0x{latest.status_flags:02x}"),
                kv("ref/rc/imu", rjoin(rbit(ref_on), " / ", rbit(remote_on), " / ", rbit(imu_on))),
                kv("feed", rjoin("g ", rbit(gimbal_msg_on), " / c ", rbit(chassis_msg_on), " / s ", rbit(shoot_msg_on))),
                kv("cmd", rjoin("g ", rbit(gimbal_cmd_on), " / c ", rbit(chassis_cmd_on))),
            ],
        ),
        (
            "Vision",
            "green",
            [
                kv("meta", f"0x{latest.vision_meta_flags:02x} / mode {latest.vision_recv_sp_mode} / {latest.vision_send_sp_mode}"),
                kv("recv yaw", pack([fnum(latest.vision_recv_yaw_raw_rad, 3), fnum(latest.vision_recv_yaw_vel_raw_rad_s, 3), fnum(latest.vision_recv_yaw_acc_raw_rad_s2, 3)])),
                kv("recv pit", pack([fnum(latest.vision_recv_pitch_raw_rad, 3), fnum(latest.vision_recv_pitch_vel_raw_rad_s, 3), fnum(latest.vision_recv_pitch_acc_raw_rad_s2, 3)])),
                kv("send q", pack([fnum(v, 3) for v in latest.vision_send_q])),
                kv("send ang", f"{fnum(latest.vision_send_yaw_raw_rad, 3)} / {fnum(latest.vision_send_pitch_raw_rad, 3)}"),
                kv("send vel", f"{fnum(latest.vision_send_yaw_vel_raw_rad_s, 3)} / {fnum(latest.vision_send_pitch_vel_raw_rad_s, 3)}"),
                kv("bullet", f"{fnum(latest.vision_send_bullet_speed_mps, 2)} / {latest.vision_send_bullet_count}"),
            ],
        ),
    ]

    if cols >= 96:
        best_layout = [[0, 1, 6], [2, 3, 4], [5, 7]]
    elif cols >= 64:
        best_layout = [[0, 1, 6, 5], [2, 3, 4, 7]]
    else:
        best_layout = [[0, 1, 2, 3, 4, 5, 6, 7]]

    column_count = len(best_layout)
    label_width = 10 if cols >= 100 else 8
    grid = Table.grid(expand=True)
    for _ in range(column_count):
        grid.add_column(ratio=1)

    column_renderables = []
    for column in best_layout:
        panels = [
            make_panel(sections[idx][0], sections[idx][1], sections[idx][2], label_width)
            for idx in column
        ]
        column_renderables.append(Group(*panels))
    grid.add_row(*column_renderables)

    renderables = [Panel(header_grid, border_style="white", padding=(0, 1))]
    renderables.append(grid)
    if rows <= 22 and cols < 96:
        renderables.append(Text("Tip: wider terminal gives cleaner wrapping.", style="yellow"))
    return Group(*renderables)


def _build_split_renderable(reader: RTTDashboardReader, cols: int, rows: int):
    dashboard_width = max(60, int(cols * 0.68))
    logs_width = max(24, cols - dashboard_width - 1)
    logs = reader.tail_logs(max(rows - 5, 5))

    logs_table = Table.grid(expand=True)
    logs_table.add_column(ratio=1)
    logs_table.add_row(Text(f"log_ch={reader.log_channel} log_bytes={reader.log_bytes_total}", style="dim"))
    for line in logs:
        logs_table.add_row(Text(line, overflow="ellipsis", no_wrap=True))

    split_grid = Table.grid(expand=True)
    split_grid.add_column(ratio=dashboard_width)
    split_grid.add_column(ratio=logs_width)
    split_grid.add_row(
        _build_dashboard_renderable(reader, dashboard_width, rows, compact_header=True),
        Panel(logs_table, title="Realtime Logs", border_style="green", padding=(0, 1)),
    )
    return split_grid


def run_dashboard_terminal(reader: RTTDashboardReader, refresh_ms: int) -> int:
    color_enabled = os.getenv("NO_COLOR") is None and os.getenv("TERM", "") != "dumb"

    if RICH_AVAILABLE:
        console = Console(no_color=not color_enabled, highlight=False)
        with Live(console=console, screen=True, auto_refresh=False) as live:
            while True:
                reader.drain_frames()
                size = console.size
                live.update(
                    _build_dashboard_renderable(
                        reader,
                        max(80, size.width - 1),
                        max(20, size.height - 1),
                        compact_header=False,
                    ),
                    refresh=True,
                )
                time.sleep(max(refresh_ms, 20) / 1000.0)
        return 0

    while True:
        reader.drain_frames()
        size = shutil.get_terminal_size((120, 40))
        cols = max(80, size.columns - 1)
        rows = max(20, size.lines - 1)
        lines = _build_dashboard_lines(
            reader,
            color_enabled,
            cols,
            rows,
            compact_header=False,
        )
        output = "\n".join(lines)
        print("\x1b[2J\x1b[H" + output, end="", flush=True)
        time.sleep(max(refresh_ms, 20) / 1000.0)


def run_split_terminal(reader: RTTDashboardReader, refresh_ms: int) -> int:
    color_enabled = os.getenv("NO_COLOR") is None and os.getenv("TERM", "") != "dumb"

    if RICH_AVAILABLE:
        console = Console(no_color=not color_enabled, highlight=False)
        with Live(console=console, screen=True, auto_refresh=False) as live:
            while True:
                reader.drain_frames()
                size = console.size
                live.update(
                    _build_split_renderable(
                        reader,
                        max(100, size.width - 1),
                        max(24, size.height - 1),
                    ),
                    refresh=True,
                )
                time.sleep(max(refresh_ms, 20) / 1000.0)
        return 0

    C_SECTION_B = "96"
    C_VALUE = "97"
    C_UNIT = "90"
    C_DIVIDER = "95"

    def style(text: str, color: str) -> str:
        return f"{_ansi(color, color_enabled)}{text}{_ansi('0', color_enabled)}"

    while True:
        reader.drain_frames()

        size = shutil.get_terminal_size((140, 40))
        total_w = max(100, size.columns - 1)
        total_h = max(24, size.lines - 1)
        left_w = max(60, int(total_w * 0.65))
        right_w = max(30, total_w - left_w - 1)

        left_lines = _build_dashboard_lines(
            reader,
            color_enabled,
            left_w,
            total_h,
            compact_header=True,
        )

        logs = reader.tail_logs(total_h - 2)
        right_lines = [style("[Realtime Logs]", C_SECTION_B)]
        right_lines.append(
            f"log_ch={style(str(reader.log_channel), C_VALUE)} {style('ch', C_UNIT)} "
            f"log_bytes={style(str(reader.log_bytes_total), C_VALUE)} {style('B', C_UNIT)}"
        )
        right_lines.extend(logs)

        rows = max(len(left_lines), len(right_lines), total_h)
        merged: List[str] = []
        divider = f"{_ansi(C_DIVIDER, color_enabled)}|{_ansi('0', color_enabled)}"
        for idx in range(rows):
            left = left_lines[idx] if idx < len(left_lines) else ""
            right = right_lines[idx] if idx < len(right_lines) else ""
            merged.append(f"{_pad_ansi_no_truncate(left, left_w)}{divider}{_truncate_ansi(right, right_w)}")

        print("\x1b[2J\x1b[H" + "\n".join(merged), end="", flush=True)
        time.sleep(max(refresh_ms, 20) / 1000.0)


def main() -> int:
    args = parse_args()
    log_channel = args.log_channel if args.mode == "split" else None
    reader = RTTDashboardReader(
        telemetry_channel=args.channel,
        log_channel=log_channel,
        speed_khz=args.speed,
        serial=args.serial,
        device=args.device,
        connect_mode=args.connect,
        poll_ms=args.poll_ms,
        backend=args.backend,
        auto_channel=not args.no_auto_channel,
        elf=args.elf,
        monitor_dir=args.monitor_dir,
        monitor_prefix=args.monitor_prefix,
        monitor_enabled=not args.no_monitor_save,
    )
    if reader.monitor_path is not None:
        print(f"Monitor export: {reader.monitor_path}")
    elif reader.monitor_error:
        print(f"Monitor export disabled: {reader.monitor_error}")
    reader.start()
    try:
        if args.mode == "split":
            return run_split_terminal(reader, args.refresh_ms)
        return run_dashboard_terminal(reader, args.refresh_ms)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        reader.stop()


if __name__ == "__main__":
    raise SystemExit(main())
