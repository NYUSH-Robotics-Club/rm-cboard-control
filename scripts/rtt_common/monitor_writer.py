#!/usr/bin/env python3
from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
from typing import Iterable

from scripts.rtt_common.telemetry import TelemetryFrame

ROOT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_MONITOR_DIR = ROOT_DIR / "monitor"
RAD_TO_DEG = 57.29577951308232

MONITOR_COLUMNS = (
    "host_rx_ms",
    "timestamp_ms",
    "seq",
    "version",
    "payload_size",
    "status_flags",
    "remote_control_flags",
    "gimbal_mode",
    "gimbal_motor_online_bitmap",
    "can_link_bitmap",
    "gimbal_cmd_yaw_deg",
    "gimbal_cmd_pitch_deg",
    "gimbal_cmd_chassis_rotate_wz",
    "gimbal_yaw_target_deg",
    "gimbal_yaw_actual_deg",
    "gimbal_yaw_error_deg",
    "gimbal_yaw_target_deg_s",
    "gimbal_yaw_actual_deg_s",
    "gimbal_yaw_speed_error_deg_s",
    "gimbal_pitch_target_deg",
    "gimbal_pitch_actual_deg",
    "gimbal_pitch_error_deg",
    "gimbal_pitch_target_deg_s",
    "gimbal_pitch_actual_deg_s",
    "gimbal_pitch_speed_error_deg_s",
    "imu_pitch_deg",
    "imu_yaw_deg",
    "imu_roll_deg",
    "imu_gyro_x_deg_s",
    "imu_gyro_y_deg_s",
    "imu_gyro_z_deg_s",
    "vision_meta_flags",
    "vision_recv_sp_mode",
    "vision_send_sp_mode",
    "vision_recv_yaw_raw_rad",
    "vision_recv_yaw_raw_deg",
    "vision_recv_yaw_vel_raw_rad_s",
    "vision_recv_yaw_vel_raw_deg_s",
    "vision_recv_yaw_acc_raw_rad_s2",
    "vision_recv_yaw_acc_raw_deg_s2",
    "vision_recv_pitch_raw_rad",
    "vision_recv_pitch_raw_deg",
    "vision_recv_pitch_vel_raw_rad_s",
    "vision_recv_pitch_vel_raw_deg_s",
    "vision_recv_pitch_acc_raw_rad_s2",
    "vision_recv_pitch_acc_raw_deg_s2",
    "vision_send_q0",
    "vision_send_q1",
    "vision_send_q2",
    "vision_send_q3",
    "vision_send_yaw_raw_rad",
    "vision_send_yaw_raw_deg",
    "vision_send_yaw_vel_raw_rad_s",
    "vision_send_yaw_vel_raw_deg_s",
    "vision_send_pitch_raw_rad",
    "vision_send_pitch_raw_deg",
    "vision_send_pitch_vel_raw_rad_s",
    "vision_send_pitch_vel_raw_deg_s",
    "vision_send_bullet_speed_mps",
    "vision_send_bullet_count",
    "shoot_bullet_speed",
    "shoot_rest_heat",
    "shoot_rate",
    "shoot_loader_speed_aps",
    "shoot_friction_l_speed_aps",
    "shoot_friction_r_speed_aps",
    "chassis_cmd_vx",
    "chassis_cmd_vy",
    "chassis_cmd_wz",
    "capabilities",
    "yaw_flags",
    "pitch_flags",
    "gimbal_enabled",
    "gimbal_startup_ready",
    "chassis_motor_id_0",
    "chassis_motor_id_1",
    "chassis_motor_id_2",
    "chassis_motor_id_3",
)


def resolve_monitor_dir(path: str | Path) -> Path:
    monitor_dir = Path(path)
    if not monitor_dir.is_absolute():
        monitor_dir = ROOT_DIR / monitor_dir
    return monitor_dir


def make_monitor_path(directory: str | Path, prefix: str) -> Path:
    monitor_dir = resolve_monitor_dir(directory)
    monitor_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return monitor_dir / f"{prefix}_{stamp}.txt"


def _format_value(value) -> str:
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        if math.isinf(value):
            return "inf" if value > 0 else "-inf"
        text = f"{value:.9f}".rstrip("0").rstrip(".")
        return "0" if text in ("-0", "") else text
    return str(value)


def _frame_row_values(frame: TelemetryFrame) -> tuple[object, ...]:
    vision_send_q = list(frame.vision_send_q) + [0.0, 0.0, 0.0, 0.0]
    return (
        frame.host_rx_ms,
        frame.timestamp_ms,
        frame.seq,
        frame.version,
        frame.payload_size,
        frame.status_flags,
        frame.remote_control_flags,
        frame.gimbal_mode,
        frame.gimbal_motor_online_bitmap,
        frame.can_link_bitmap,
        frame.gimbal_cmd_yaw_deg,
        frame.gimbal_cmd_pitch_deg,
        frame.gimbal_cmd_chassis_rotate_wz,
        frame.gimbal_yaw_target_deg,
        frame.gimbal_yaw_actual_deg,
        frame.gimbal_yaw_target_deg - frame.gimbal_yaw_actual_deg,
        frame.gimbal_yaw_target_deg_s,
        frame.gimbal_yaw_actual_deg_s,
        frame.gimbal_yaw_target_deg_s - frame.gimbal_yaw_actual_deg_s,
        frame.gimbal_pitch_target_deg,
        frame.gimbal_pitch_actual_deg,
        frame.gimbal_pitch_target_deg - frame.gimbal_pitch_actual_deg,
        frame.gimbal_pitch_target_deg_s,
        frame.gimbal_pitch_actual_deg_s,
        frame.gimbal_pitch_target_deg_s - frame.gimbal_pitch_actual_deg_s,
        frame.imu_angle_deg[0],
        frame.imu_angle_deg[1],
        frame.imu_angle_deg[2],
        frame.imu_gyro_deg_s[0],
        frame.imu_gyro_deg_s[1],
        frame.imu_gyro_deg_s[2],
        frame.vision_meta_flags,
        frame.vision_recv_sp_mode,
        frame.vision_send_sp_mode,
        frame.vision_recv_yaw_raw_rad,
        frame.vision_recv_yaw_raw_rad * RAD_TO_DEG,
        frame.vision_recv_yaw_vel_raw_rad_s,
        frame.vision_recv_yaw_vel_raw_rad_s * RAD_TO_DEG,
        frame.vision_recv_yaw_acc_raw_rad_s2,
        frame.vision_recv_yaw_acc_raw_rad_s2 * RAD_TO_DEG,
        frame.vision_recv_pitch_raw_rad,
        frame.vision_recv_pitch_raw_rad * RAD_TO_DEG,
        frame.vision_recv_pitch_vel_raw_rad_s,
        frame.vision_recv_pitch_vel_raw_rad_s * RAD_TO_DEG,
        frame.vision_recv_pitch_acc_raw_rad_s2,
        frame.vision_recv_pitch_acc_raw_rad_s2 * RAD_TO_DEG,
        vision_send_q[0],
        vision_send_q[1],
        vision_send_q[2],
        vision_send_q[3],
        frame.vision_send_yaw_raw_rad,
        frame.vision_send_yaw_raw_rad * RAD_TO_DEG,
        frame.vision_send_yaw_vel_raw_rad_s,
        frame.vision_send_yaw_vel_raw_rad_s * RAD_TO_DEG,
        frame.vision_send_pitch_raw_rad,
        frame.vision_send_pitch_raw_rad * RAD_TO_DEG,
        frame.vision_send_pitch_vel_raw_rad_s,
        frame.vision_send_pitch_vel_raw_rad_s * RAD_TO_DEG,
        frame.vision_send_bullet_speed_mps,
        frame.vision_send_bullet_count,
        frame.shoot_bullet_speed,
        frame.shoot_rest_heat,
        frame.shoot_rate,
        frame.shoot_loader_speed_aps,
        frame.shoot_friction_l_speed_aps,
        frame.shoot_friction_r_speed_aps,
        frame.chassis_cmd_vx,
        frame.chassis_cmd_vy,
        frame.chassis_cmd_wz,
        frame.capabilities,
        frame.yaw_flags,
        frame.pitch_flags,
        frame.gimbal_enabled,
        frame.gimbal_startup_ready,
        frame.chassis_motor_ids[0],
        frame.chassis_motor_ids[1],
        frame.chassis_motor_ids[2],
        frame.chassis_motor_ids[3],
    )


class MonitorWriter:
    def __init__(self, output_path: str | Path) -> None:
        self.path = Path(output_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8", buffering=1)
        self._handle.write("\t".join(MONITOR_COLUMNS) + "\n")

    @classmethod
    def create(cls, directory: str | Path = DEFAULT_MONITOR_DIR, prefix: str = "logger") -> "MonitorWriter":
        return cls(make_monitor_path(directory, prefix))

    def write_frames(self, frames: Iterable[TelemetryFrame]) -> None:
        lines = []
        for frame in frames:
            values = _frame_row_values(frame)
            lines.append("\t".join(_format_value(value) for value in values))
        if not lines:
            return
        self._handle.write("\n".join(lines) + "\n")

    def close(self) -> None:
        self._handle.close()
