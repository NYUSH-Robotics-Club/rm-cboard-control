#!/usr/bin/env python3
"""Decode versioned RTT frames, validate CRC, and preserve unavailable data as NaN.
Yaw diagnostics describe one callback; timestamps and source counters identify its input.
"""
import dataclasses
import struct
from typing import Dict, List

MAGIC = 0x4452
MAGIC_ALT = 0x5244
CURRENT_VERSION = 10
SUPPORTED_VERSIONS = (3, 4, 5, 6, 7, 8, 9, 10)

HEADER_STRUCT = struct.Struct("<HBBI")
PAYLOAD_STRUCT_V3 = struct.Struct("<Iff4f4f3f3f11f10B8h2BH8BII7fBHBB15fH")
PAYLOAD_STRUCT_V4 = struct.Struct("<Iff4f4f3f3f11fBHBBHHHH6B8hBH3BII7fBB15fH")
PAYLOAD_STRUCT_V5 = struct.Struct("<Iff4f4f3f3f11fBHBBHHHHH6B8hBH3B5HII7fBB15fH")
PAYLOAD_STRUCT_V6 = struct.Struct("<IffHfffB4f4f3f3f11fBHBBHHHHH6B8hBHBBB5HfII7fBB15fH")
PAYLOAD_STRUCT_V7 = struct.Struct("<IffHfffB4f4f3f3f11fHHBHBBHHHHH6B8hBHBBB5HfII7fBB15fH")
# v8 appends source availability, axis flags and motor IDs to the v7 prefix.
PAYLOAD_STRUCT_V8 = struct.Struct(PAYLOAD_STRUCT_V7.format + "IBB4BBB")
V8_METADATA = struct.Struct("<IBB4BBB")
# v9 appends one callback snapshot; the v8 prefix stays byte-for-byte compatible.
V9_DIAGNOSTICS = struct.Struct("<9Iif2I2fI3i9f")
PAYLOAD_STRUCT_V9 = struct.Struct(PAYLOAD_STRUCT_V8.format + V9_DIAGNOSTICS.format[1:])
# v10 appends four speed-PID outputs and four current feedback values.
PAYLOAD_STRUCT_V10 = struct.Struct(PAYLOAD_STRUCT_V9.format + "8f")
YAW_DIAGNOSTIC_FIELDS = (
    "yaw_diag_valid",
    "yaw_sample_ms",
    "yaw_callback_count",
    "yaw_callback_dt_ms",
    "yaw_trace_flags",
    "yaw_rc_sequence",
    "yaw_rc_dispatch_ms",
    "yaw_route_sequence",
    "yaw_route_ms",
    "yaw_rc_ch0",
    "yaw_route_rate",
    "yaw_mode",
    "yaw_feedback_ms",
    "yaw_speed_raw_rpm",
    "yaw_pid_dt_s",
    "yaw_command_unit",
    "yaw_command_status",
    "yaw_command_raw",
    "yaw_current_actual_raw",
    "yaw_pid_pout",
    "yaw_pid_iout",
    "yaw_pid_dout",
    "yaw_pid_output",
    "yaw_pid_kp",
    "yaw_pid_ki",
    "yaw_pid_kd",
    "yaw_pid_output_max",
    "yaw_pid_integral_max",
)
CRC_STRUCT = struct.Struct("<H")
PAYLOAD_STRUCTS: Dict[int, struct.Struct] = {
    3: PAYLOAD_STRUCT_V3,
    4: PAYLOAD_STRUCT_V4,
    5: PAYLOAD_STRUCT_V5,
    6: PAYLOAD_STRUCT_V6,
    7: PAYLOAD_STRUCT_V7,
    8: PAYLOAD_STRUCT_V8,
    9: PAYLOAD_STRUCT_V9,
    10: PAYLOAD_STRUCT_V10,
}
PAYLOAD_SIZES = {version: payload_struct.size for version, payload_struct in PAYLOAD_STRUCTS.items()}
PAYLOAD_SIZE = PAYLOAD_SIZES[CURRENT_VERSION]
MIN_FRAME_SIZE = HEADER_STRUCT.size + CRC_STRUCT.size


def crc16_firmware(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def _decode_remote_mode_switch(mode_code: int) -> int:
    return 0xFF if mode_code == 3 else mode_code


def _decode_bullet_speed(code: int) -> int:
    mapping = {
        0: 0,
        1: 10,
        2: 15,
        3: 16,
        4: 18,
        5: 30,
    }
    return mapping.get(code, 0)


@dataclasses.dataclass
class TelemetryFrame:
    version: int
    payload_size: int
    timestamp_ms: int
    chassis_power: float
    chassis_volt: float
    motor_target_rpm: List[float]
    motor_rpm: List[float]
    imu_angle_deg: List[float]
    imu_gyro_deg_s: List[float]
    gimbal_yaw_target_deg: float
    gimbal_yaw_actual_deg: float
    gimbal_yaw_target_deg_s: float
    gimbal_yaw_actual_deg_s: float
    gimbal_pitch_target_deg: float
    gimbal_pitch_actual_deg: float
    gimbal_pitch_target_deg_s: float
    gimbal_pitch_actual_deg_s: float
    gimbal_cmd_yaw_deg: float
    gimbal_cmd_pitch_deg: float
    gimbal_cmd_chassis_rotate_wz: float
    gimbal_yaw_encoder_raw: int
    gimbal_pitch_encoder_raw: int
    referee_game_state_raw: int
    referee_game_type: int
    referee_status: int
    referee_stage_remain_time: int
    referee_robot_id: int
    referee_robot_level: int
    referee_current_hp: int
    referee_maximum_hp: int
    referee_shooter_barrel_cooling_value: int
    referee_shooter_barrel_heat_limit: int
    referee_chassis_power_limit: int
    referee_power_management_flags: int
    referee_chassis_current: int
    referee_buffer_energy: int
    referee_shooter_17mm_1_barrel_heat: int
    referee_shooter_17mm_2_barrel_heat: int
    referee_shooter_42mm_barrel_heat: int
    referee_shoot_bullet_speed_mps: float
    rc_switches: int
    motor_online_bitmap: int
    gimbal_motor_online_bitmap: int
    shoot_motor_online_bitmap: int
    status_flags: int
    remote_protocol: int
    remote_mode_switch: int
    remote_button_bits: int
    remote_control_flags: int
    rc_rocker_l_x: int
    rc_rocker_l_y: int
    rc_rocker_r_x: int
    rc_rocker_r_y: int
    rc_dial: int
    mouse_x: int
    mouse_y: int
    mouse_z: int
    mouse_left: int
    mouse_right: int
    key_pressed_bits: int
    chassis_mode: int
    gimbal_mode: int
    shoot_mode: int
    shoot_load_mode: int
    shoot_lid_mode: int
    shoot_friction_mode: int
    shoot_bullet_speed: int
    shoot_rest_heat: int
    telemetry_drop_count: int
    free_heap_bytes: int
    chassis_cmd_vx: float
    chassis_cmd_vy: float
    chassis_cmd_wz: float
    shoot_rate: float
    shoot_loader_speed_aps: float
    shoot_friction_l_speed_aps: float
    shoot_friction_r_speed_aps: float
    can_link_bitmap: int
    vision_meta_flags: int
    vision_recv_sp_mode: int
    vision_send_sp_mode: int
    vision_recv_yaw_raw_rad: float
    vision_recv_yaw_vel_raw_rad_s: float
    vision_recv_yaw_acc_raw_rad_s2: float
    vision_recv_pitch_raw_rad: float
    vision_recv_pitch_vel_raw_rad_s: float
    vision_recv_pitch_acc_raw_rad_s2: float
    vision_send_q: List[float]
    vision_send_yaw_raw_rad: float
    vision_send_yaw_vel_raw_rad_s: float
    vision_send_pitch_raw_rad: float
    vision_send_pitch_vel_raw_rad_s: float
    vision_send_bullet_speed_mps: float
    vision_send_bullet_count: int
    seq: int
    host_rx_ms: int
    chassis_power_buffer_energy: int = 0
    chassis_power_budget_w: float = 0.0
    chassis_power_estimated_w: float = 0.0
    chassis_power_scale: float = 1.0
    chassis_power_mode_flags: int = 0
    capabilities: int = 0
    yaw_flags: int = 0
    pitch_flags: int = 0
    chassis_motor_ids: List[int] = dataclasses.field(default_factory=lambda: [0] * 4)
    chassis_pid_output: List[float] = dataclasses.field(default_factory=lambda: [float("nan")] * 4)
    chassis_current_actual_raw: List[float] = dataclasses.field(default_factory=lambda: [float("nan")] * 4)
    gimbal_enabled: int = 0
    gimbal_startup_ready: int = 0
    # Absent legacy diagnostics remain NaN, never plausible zero measurements.
    yaw_diag_valid: int | float = 0
    yaw_sample_ms: int | float = float("nan")
    yaw_callback_count: int | float = float("nan")
    yaw_callback_dt_ms: int | float = float("nan")
    yaw_trace_flags: int | float = 0
    yaw_rc_sequence: int | float = float("nan")
    yaw_rc_dispatch_ms: int | float = float("nan")
    yaw_route_sequence: int | float = float("nan")
    yaw_route_ms: int | float = float("nan")
    yaw_rc_ch0: int | float = float("nan")
    yaw_route_rate: int | float = float("nan")
    yaw_mode: int | float = float("nan")
    yaw_feedback_ms: int | float = float("nan")
    yaw_speed_raw_rpm: int | float = float("nan")
    yaw_pid_dt_s: int | float = float("nan")
    yaw_command_unit: int | float = float("nan")
    yaw_command_status: int | float = float("nan")
    yaw_command_raw: int | float = float("nan")
    yaw_current_actual_raw: int | float = float("nan")
    yaw_pid_pout: int | float = float("nan")
    yaw_pid_iout: int | float = float("nan")
    yaw_pid_dout: int | float = float("nan")
    yaw_pid_output: int | float = float("nan")
    yaw_pid_kp: int | float = float("nan")
    yaw_pid_ki: int | float = float("nan")
    yaw_pid_kd: int | float = float("nan")
    yaw_pid_output_max: int | float = float("nan")
    yaw_pid_integral_max: int | float = float("nan")



def _validate_yaw_diagnostics(frame: TelemetryFrame) -> None:
    """Apply snapshot/source validity before exposing measurements to any consumer."""
    def invalidate(names):
        for name in names:
            setattr(frame, name, float("nan"))

    if not frame.yaw_diag_valid:
        invalidate(name for name in YAW_DIAGNOSTIC_FIELDS if name not in ("yaw_diag_valid", "yaw_trace_flags"))
        frame.yaw_trace_flags = 0
        return
    if not (frame.yaw_trace_flags & 1):
        invalidate(("yaw_route_sequence", "yaw_route_ms"))
    if (frame.yaw_trace_flags & 3) != 3:
        invalidate(("yaw_rc_ch0", "yaw_rc_sequence", "yaw_rc_dispatch_ms"))
    if not (frame.yaw_flags & 4):
        invalidate(("yaw_speed_raw_rpm", "yaw_current_actual_raw"))
    if not (frame.yaw_flags & 2):
        invalidate(("yaw_feedback_ms",))
    if not (frame.yaw_flags & 1):
        invalidate(("yaw_command_unit", "yaw_command_status", "yaw_command_raw",
                    "yaw_pid_kp", "yaw_pid_ki", "yaw_pid_kd", "yaw_pid_output_max", "yaw_pid_integral_max"))
    if not (frame.yaw_flags & 8):
        invalidate(("yaw_mode", "yaw_pid_dt_s", "yaw_pid_pout", "yaw_pid_iout", "yaw_pid_dout", "yaw_pid_output"))


def chassis_mode_name(mode: int) -> str:
    names = {
        0: "ZERO_FORCE",
        1: "ROTATE",
        2: "NO_FOLLOW",
        3: "FOLLOW_GIMBAL_YAW",
    }
    return names.get(mode, f"UNKNOWN({mode})")


def gimbal_mode_name(mode: int) -> str:
    names = {
        0: "ZERO_FORCE",
        1: "FREE_MODE",
        2: "GYRO_MODE",
    }
    return names.get(mode, f"UNKNOWN({mode})")


def remote_protocol_name(protocol: int) -> str:
    names = {
        0: "UNKNOWN",
        1: "DBUS",
        2: "VT03/VT13 UART",
    }
    return names.get(protocol, f"UNKNOWN({protocol})")


def remote_mode_switch_name(raw_mode: int) -> str:
    names = {
        0: "C",
        1: "N",
        2: "S",
        0xFF: "N/A",
    }
    return names.get(raw_mode, f"UNKNOWN({raw_mode})")


def bullet_speed_name(speed: int) -> str:
    names = {
        0: "NONE",
        10: "BIG_AMU_10",
        15: "SMALL_AMU_15",
        16: "BIG_AMU_16",
        18: "SMALL_AMU_18",
        30: "SMALL_AMU_30",
    }
    return names.get(speed, f"UNKNOWN({speed})")


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_error_count = 0
        self.sync_drop_count = 0
        self.magic_hit_count = 0
        self.header_mismatch_count = 0
        self.last_bad_header = ""
        self.last_bad_reason = ""
        self.stream_locked = False

    def _parse_v3(self, raw: bytes, version: int, seq: int, host_rx_ms: int) -> TelemetryFrame:
        (
            timestamp_ms,
            chassis_power,
            chassis_volt,
            target_rpm0,
            target_rpm1,
            target_rpm2,
            target_rpm3,
            rpm0,
            rpm1,
            rpm2,
            rpm3,
            imu_pitch_deg,
            imu_yaw_deg,
            imu_roll_deg,
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz,
            referee_status,
            rc_switches,
            motor_online_bitmap,
            gimbal_motor_online_bitmap,
            shoot_motor_online_bitmap,
            status_flags,
            remote_protocol,
            remote_mode_switch,
            remote_button_bits,
            remote_control_flags,
            rc_rocker_l_x,
            rc_rocker_l_y,
            rc_rocker_r_x,
            rc_rocker_r_y,
            rc_dial,
            mouse_x,
            mouse_y,
            mouse_z,
            mouse_left,
            mouse_right,
            key_pressed_bits,
            chassis_mode,
            gimbal_mode,
            shoot_mode,
            shoot_load_mode,
            shoot_lid_mode,
            shoot_friction_mode,
            shoot_bullet_speed,
            shoot_rest_heat,
            telemetry_drop_count,
            free_heap_bytes,
            chassis_cmd_vx,
            chassis_cmd_vy,
            chassis_cmd_wz,
            shoot_rate,
            shoot_loader_speed_aps,
            shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps,
            can_link_bitmap,
            vision_meta_flags,
            vision_recv_sp_mode,
            vision_send_sp_mode,
            vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q0,
            vision_send_q1,
            vision_send_q2,
            vision_send_q3,
            vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps,
            vision_send_bullet_count,
        ) = PAYLOAD_STRUCT_V3.unpack_from(raw, HEADER_STRUCT.size)

        return TelemetryFrame(
            version=version,
            payload_size=PAYLOAD_SIZES[version],
            timestamp_ms=timestamp_ms,
            chassis_power=chassis_power,
            chassis_volt=chassis_volt,
            motor_target_rpm=[target_rpm0, target_rpm1, target_rpm2, target_rpm3],
            motor_rpm=[rpm0, rpm1, rpm2, rpm3],
            imu_angle_deg=[imu_pitch_deg, imu_yaw_deg, imu_roll_deg],
            imu_gyro_deg_s=[imu_gyro_x, imu_gyro_y, imu_gyro_z],
            gimbal_yaw_target_deg=gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg=gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s=gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s=gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg=gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg=gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s=gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s=gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg=gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg=gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz=gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw=0,
            gimbal_pitch_encoder_raw=0,
            referee_game_state_raw=(referee_status & 0x0F) << 4,
            referee_game_type=0,
            referee_status=referee_status,
            referee_stage_remain_time=0,
            referee_robot_id=0,
            referee_robot_level=0,
            referee_current_hp=0,
            referee_maximum_hp=0,
            referee_shooter_barrel_cooling_value=0,
            referee_shooter_barrel_heat_limit=0,
            referee_chassis_power_limit=0,
            referee_power_management_flags=0,
            referee_chassis_current=0,
            referee_buffer_energy=0,
            referee_shooter_17mm_1_barrel_heat=0,
            referee_shooter_17mm_2_barrel_heat=0,
            referee_shooter_42mm_barrel_heat=0,
            referee_shoot_bullet_speed_mps=0.0,
            rc_switches=rc_switches,
            motor_online_bitmap=motor_online_bitmap,
            gimbal_motor_online_bitmap=gimbal_motor_online_bitmap,
            shoot_motor_online_bitmap=shoot_motor_online_bitmap,
            status_flags=status_flags,
            remote_protocol=remote_protocol,
            remote_mode_switch=remote_mode_switch,
            remote_button_bits=remote_button_bits,
            remote_control_flags=remote_control_flags,
            rc_rocker_l_x=rc_rocker_l_x,
            rc_rocker_l_y=rc_rocker_l_y,
            rc_rocker_r_x=rc_rocker_r_x,
            rc_rocker_r_y=rc_rocker_r_y,
            rc_dial=rc_dial,
            mouse_x=mouse_x,
            mouse_y=mouse_y,
            mouse_z=mouse_z,
            mouse_left=mouse_left,
            mouse_right=mouse_right,
            key_pressed_bits=key_pressed_bits,
            chassis_mode=chassis_mode,
            gimbal_mode=gimbal_mode,
            shoot_mode=shoot_mode,
            shoot_load_mode=shoot_load_mode,
            shoot_lid_mode=shoot_lid_mode,
            shoot_friction_mode=shoot_friction_mode,
            shoot_bullet_speed=shoot_bullet_speed,
            shoot_rest_heat=shoot_rest_heat,
            telemetry_drop_count=telemetry_drop_count,
            free_heap_bytes=free_heap_bytes,
            chassis_cmd_vx=chassis_cmd_vx,
            chassis_cmd_vy=chassis_cmd_vy,
            chassis_cmd_wz=chassis_cmd_wz,
            shoot_rate=shoot_rate,
            shoot_loader_speed_aps=shoot_loader_speed_aps,
            shoot_friction_l_speed_aps=shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps=shoot_friction_r_speed_aps,
            can_link_bitmap=can_link_bitmap,
            vision_meta_flags=vision_meta_flags,
            vision_recv_sp_mode=vision_recv_sp_mode,
            vision_send_sp_mode=vision_send_sp_mode,
            vision_recv_yaw_raw_rad=vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s=vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2=vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad=vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s=vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2=vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q=[vision_send_q0, vision_send_q1, vision_send_q2, vision_send_q3],
            vision_send_yaw_raw_rad=vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s=vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad=vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s=vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps=vision_send_bullet_speed_mps,
            vision_send_bullet_count=vision_send_bullet_count,
            seq=seq,
            host_rx_ms=host_rx_ms,
        )

    def _parse_v4(self, raw: bytes, version: int, seq: int, host_rx_ms: int) -> TelemetryFrame:
        (
            timestamp_ms,
            chassis_power,
            chassis_volt,
            target_rpm0,
            target_rpm1,
            target_rpm2,
            target_rpm3,
            rpm0,
            rpm1,
            rpm2,
            rpm3,
            imu_pitch_deg,
            imu_yaw_deg,
            imu_roll_deg,
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz,
            referee_game_state_raw,
            referee_stage_remain_time,
            referee_robot_id,
            referee_robot_level,
            referee_current_hp,
            referee_maximum_hp,
            referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit,
            referee_power_management_flags,
            remote_packed,
            motor_online_bitmap,
            link_bitmap_packed,
            status_flags,
            remote_state_flags,
            rc_rocker_l_x,
            rc_rocker_l_y,
            rc_rocker_r_x,
            rc_rocker_r_y,
            rc_dial,
            mouse_x,
            mouse_y,
            mouse_z,
            mouse_buttons,
            key_pressed_bits,
            mode_packed_low,
            mode_packed_high,
            shoot_rest_heat,
            telemetry_drop_count,
            free_heap_bytes,
            chassis_cmd_vx,
            chassis_cmd_vy,
            chassis_cmd_wz,
            shoot_rate,
            shoot_loader_speed_aps,
            shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps,
            vision_meta_flags,
            vision_sp_mode_packed,
            vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q0,
            vision_send_q1,
            vision_send_q2,
            vision_send_q3,
            vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps,
            vision_send_bullet_count,
        ) = PAYLOAD_STRUCT_V4.unpack_from(raw, HEADER_STRUCT.size)

        return TelemetryFrame(
            version=version,
            payload_size=PAYLOAD_SIZES[version],
            timestamp_ms=timestamp_ms,
            chassis_power=chassis_power,
            chassis_volt=chassis_volt,
            motor_target_rpm=[target_rpm0, target_rpm1, target_rpm2, target_rpm3],
            motor_rpm=[rpm0, rpm1, rpm2, rpm3],
            imu_angle_deg=[imu_pitch_deg, imu_yaw_deg, imu_roll_deg],
            imu_gyro_deg_s=[imu_gyro_x, imu_gyro_y, imu_gyro_z],
            gimbal_yaw_target_deg=gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg=gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s=gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s=gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg=gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg=gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s=gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s=gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg=gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg=gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz=gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw=0,
            gimbal_pitch_encoder_raw=0,
            referee_game_state_raw=referee_game_state_raw,
            referee_game_type=referee_game_state_raw & 0x0F,
            referee_status=(referee_game_state_raw >> 4) & 0x0F,
            referee_stage_remain_time=referee_stage_remain_time,
            referee_robot_id=referee_robot_id,
            referee_robot_level=referee_robot_level,
            referee_current_hp=referee_current_hp,
            referee_maximum_hp=referee_maximum_hp,
            referee_shooter_barrel_cooling_value=0,
            referee_shooter_barrel_heat_limit=referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit=referee_chassis_power_limit,
            referee_power_management_flags=referee_power_management_flags,
            referee_chassis_current=0,
            referee_buffer_energy=0,
            referee_shooter_17mm_1_barrel_heat=0,
            referee_shooter_17mm_2_barrel_heat=0,
            referee_shooter_42mm_barrel_heat=0,
            referee_shoot_bullet_speed_mps=0.0,
            rc_switches=remote_packed & 0x0F,
            motor_online_bitmap=motor_online_bitmap,
            gimbal_motor_online_bitmap=link_bitmap_packed & 0x03,
            shoot_motor_online_bitmap=(link_bitmap_packed >> 2) & 0x07,
            status_flags=status_flags,
            remote_protocol=(remote_packed >> 4) & 0x03,
            remote_mode_switch=_decode_remote_mode_switch((remote_packed >> 6) & 0x03),
            remote_button_bits=remote_state_flags & 0x1F,
            remote_control_flags=(remote_state_flags >> 5) & 0x07,
            rc_rocker_l_x=rc_rocker_l_x,
            rc_rocker_l_y=rc_rocker_l_y,
            rc_rocker_r_x=rc_rocker_r_x,
            rc_rocker_r_y=rc_rocker_r_y,
            rc_dial=rc_dial,
            mouse_x=mouse_x,
            mouse_y=mouse_y,
            mouse_z=mouse_z,
            mouse_left=mouse_buttons & 0x01,
            mouse_right=(mouse_buttons >> 1) & 0x01,
            key_pressed_bits=key_pressed_bits,
            chassis_mode=mode_packed_low & 0x03,
            gimbal_mode=(mode_packed_low >> 2) & 0x03,
            shoot_mode=(mode_packed_low >> 4) & 0x01,
            shoot_load_mode=(mode_packed_low >> 5) & 0x07,
            shoot_lid_mode=mode_packed_high & 0x01,
            shoot_friction_mode=(mode_packed_high >> 1) & 0x01,
            shoot_bullet_speed=_decode_bullet_speed((mode_packed_high >> 2) & 0x07),
            shoot_rest_heat=shoot_rest_heat,
            telemetry_drop_count=telemetry_drop_count,
            free_heap_bytes=free_heap_bytes,
            chassis_cmd_vx=chassis_cmd_vx,
            chassis_cmd_vy=chassis_cmd_vy,
            chassis_cmd_wz=chassis_cmd_wz,
            shoot_rate=shoot_rate,
            shoot_loader_speed_aps=shoot_loader_speed_aps,
            shoot_friction_l_speed_aps=shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps=shoot_friction_r_speed_aps,
            can_link_bitmap=(link_bitmap_packed >> 5) & 0x03,
            vision_meta_flags=vision_meta_flags,
            vision_recv_sp_mode=vision_sp_mode_packed & 0x0F,
            vision_send_sp_mode=(vision_sp_mode_packed >> 4) & 0x0F,
            vision_recv_yaw_raw_rad=vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s=vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2=vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad=vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s=vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2=vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q=[vision_send_q0, vision_send_q1, vision_send_q2, vision_send_q3],
            vision_send_yaw_raw_rad=vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s=vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad=vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s=vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps=vision_send_bullet_speed_mps,
            vision_send_bullet_count=vision_send_bullet_count,
            seq=seq,
            host_rx_ms=host_rx_ms,
        )

    def _parse_v5(self, raw: bytes, version: int, seq: int, host_rx_ms: int) -> TelemetryFrame:
        (
            timestamp_ms,
            chassis_power,
            chassis_volt,
            target_rpm0,
            target_rpm1,
            target_rpm2,
            target_rpm3,
            rpm0,
            rpm1,
            rpm2,
            rpm3,
            imu_pitch_deg,
            imu_yaw_deg,
            imu_roll_deg,
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz,
            referee_game_state_raw,
            referee_stage_remain_time,
            referee_robot_id,
            referee_robot_level,
            referee_current_hp,
            referee_maximum_hp,
            referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit,
            referee_power_management_flags,
            remote_packed,
            motor_online_bitmap,
            link_bitmap_packed,
            status_flags,
            remote_state_flags,
            rc_rocker_l_x,
            rc_rocker_l_y,
            rc_rocker_r_x,
            rc_rocker_r_y,
            rc_dial,
            mouse_x,
            mouse_y,
            mouse_z,
            mouse_buttons,
            key_pressed_bits,
            mode_packed_low,
            mode_packed_high,
            shoot_rest_heat,
            referee_chassis_current,
            referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat,
            telemetry_drop_count,
            free_heap_bytes,
            chassis_cmd_vx,
            chassis_cmd_vy,
            chassis_cmd_wz,
            shoot_rate,
            shoot_loader_speed_aps,
            shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps,
            vision_meta_flags,
            vision_sp_mode_packed,
            vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q0,
            vision_send_q1,
            vision_send_q2,
            vision_send_q3,
            vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps,
            vision_send_bullet_count,
        ) = PAYLOAD_STRUCT_V5.unpack_from(raw, HEADER_STRUCT.size)

        return TelemetryFrame(
            version=version,
            payload_size=PAYLOAD_SIZES[version],
            timestamp_ms=timestamp_ms,
            chassis_power=chassis_power,
            chassis_volt=chassis_volt,
            motor_target_rpm=[target_rpm0, target_rpm1, target_rpm2, target_rpm3],
            motor_rpm=[rpm0, rpm1, rpm2, rpm3],
            imu_angle_deg=[imu_pitch_deg, imu_yaw_deg, imu_roll_deg],
            imu_gyro_deg_s=[imu_gyro_x, imu_gyro_y, imu_gyro_z],
            gimbal_yaw_target_deg=gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg=gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s=gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s=gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg=gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg=gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s=gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s=gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg=gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg=gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz=gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw=0,
            gimbal_pitch_encoder_raw=0,
            referee_game_state_raw=referee_game_state_raw,
            referee_game_type=referee_game_state_raw & 0x0F,
            referee_status=(referee_game_state_raw >> 4) & 0x0F,
            referee_stage_remain_time=referee_stage_remain_time,
            referee_robot_id=referee_robot_id,
            referee_robot_level=referee_robot_level,
            referee_current_hp=referee_current_hp,
            referee_maximum_hp=referee_maximum_hp,
            referee_shooter_barrel_cooling_value=referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit=referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit=referee_chassis_power_limit,
            referee_power_management_flags=referee_power_management_flags,
            referee_chassis_current=referee_chassis_current,
            referee_buffer_energy=referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat=referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat=referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat=referee_shooter_42mm_barrel_heat,
            referee_shoot_bullet_speed_mps=0.0,
            rc_switches=remote_packed & 0x0F,
            motor_online_bitmap=motor_online_bitmap,
            gimbal_motor_online_bitmap=link_bitmap_packed & 0x03,
            shoot_motor_online_bitmap=(link_bitmap_packed >> 2) & 0x07,
            status_flags=status_flags,
            remote_protocol=(remote_packed >> 4) & 0x03,
            remote_mode_switch=_decode_remote_mode_switch((remote_packed >> 6) & 0x03),
            remote_button_bits=remote_state_flags & 0x1F,
            remote_control_flags=(remote_state_flags >> 5) & 0x07,
            rc_rocker_l_x=rc_rocker_l_x,
            rc_rocker_l_y=rc_rocker_l_y,
            rc_rocker_r_x=rc_rocker_r_x,
            rc_rocker_r_y=rc_rocker_r_y,
            rc_dial=rc_dial,
            mouse_x=mouse_x,
            mouse_y=mouse_y,
            mouse_z=mouse_z,
            mouse_left=mouse_buttons & 0x01,
            mouse_right=(mouse_buttons >> 1) & 0x01,
            key_pressed_bits=key_pressed_bits,
            chassis_mode=mode_packed_low & 0x03,
            gimbal_mode=(mode_packed_low >> 2) & 0x03,
            shoot_mode=(mode_packed_low >> 4) & 0x01,
            shoot_load_mode=(mode_packed_low >> 5) & 0x07,
            shoot_lid_mode=mode_packed_high & 0x01,
            shoot_friction_mode=(mode_packed_high >> 1) & 0x01,
            shoot_bullet_speed=_decode_bullet_speed((mode_packed_high >> 2) & 0x07),
            shoot_rest_heat=shoot_rest_heat,
            telemetry_drop_count=telemetry_drop_count,
            free_heap_bytes=free_heap_bytes,
            chassis_cmd_vx=chassis_cmd_vx,
            chassis_cmd_vy=chassis_cmd_vy,
            chassis_cmd_wz=chassis_cmd_wz,
            shoot_rate=shoot_rate,
            shoot_loader_speed_aps=shoot_loader_speed_aps,
            shoot_friction_l_speed_aps=shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps=shoot_friction_r_speed_aps,
            can_link_bitmap=(link_bitmap_packed >> 5) & 0x03,
            vision_meta_flags=vision_meta_flags,
            vision_recv_sp_mode=vision_sp_mode_packed & 0x0F,
            vision_send_sp_mode=(vision_sp_mode_packed >> 4) & 0x0F,
            vision_recv_yaw_raw_rad=vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s=vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2=vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad=vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s=vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2=vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q=[vision_send_q0, vision_send_q1, vision_send_q2, vision_send_q3],
            vision_send_yaw_raw_rad=vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s=vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad=vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s=vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps=vision_send_bullet_speed_mps,
            vision_send_bullet_count=vision_send_bullet_count,
            seq=seq,
            host_rx_ms=host_rx_ms,
        )

    def _parse_v6(self, raw: bytes, version: int, seq: int, host_rx_ms: int) -> TelemetryFrame:
        (
            timestamp_ms,
            chassis_power,
            chassis_volt,
            chassis_power_buffer_energy,
            chassis_power_budget_w,
            chassis_power_estimated_w,
            chassis_power_scale,
            chassis_power_mode_flags,
            target_rpm0,
            target_rpm1,
            target_rpm2,
            target_rpm3,
            rpm0,
            rpm1,
            rpm2,
            rpm3,
            imu_pitch_deg,
            imu_yaw_deg,
            imu_roll_deg,
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz,
            referee_game_state_raw,
            referee_stage_remain_time,
            referee_robot_id,
            referee_robot_level,
            referee_current_hp,
            referee_maximum_hp,
            referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit,
            referee_power_management_flags,
            remote_packed,
            motor_online_bitmap,
            link_bitmap_packed,
            status_flags,
            remote_state_flags,
            rc_rocker_l_x,
            rc_rocker_l_y,
            rc_rocker_r_x,
            rc_rocker_r_y,
            rc_dial,
            mouse_x,
            mouse_y,
            mouse_z,
            mouse_buttons,
            key_pressed_bits,
            mode_packed_low,
            mode_packed_high,
            shoot_rest_heat,
            referee_chassis_current,
            referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat,
            referee_shoot_bullet_speed_mps,
            telemetry_drop_count,
            free_heap_bytes,
            chassis_cmd_vx,
            chassis_cmd_vy,
            chassis_cmd_wz,
            shoot_rate,
            shoot_loader_speed_aps,
            shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps,
            vision_meta_flags,
            vision_sp_mode_packed,
            vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q0,
            vision_send_q1,
            vision_send_q2,
            vision_send_q3,
            vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps,
            vision_send_bullet_count,
        ) = PAYLOAD_STRUCT_V6.unpack_from(raw, HEADER_STRUCT.size)

        return TelemetryFrame(
            version=version,
            payload_size=PAYLOAD_SIZES[version],
            timestamp_ms=timestamp_ms,
            chassis_power=chassis_power,
            chassis_volt=chassis_volt,
            motor_target_rpm=[target_rpm0, target_rpm1, target_rpm2, target_rpm3],
            motor_rpm=[rpm0, rpm1, rpm2, rpm3],
            imu_angle_deg=[imu_pitch_deg, imu_yaw_deg, imu_roll_deg],
            imu_gyro_deg_s=[imu_gyro_x, imu_gyro_y, imu_gyro_z],
            gimbal_yaw_target_deg=gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg=gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s=gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s=gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg=gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg=gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s=gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s=gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg=gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg=gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz=gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw=0,
            gimbal_pitch_encoder_raw=0,
            referee_game_state_raw=referee_game_state_raw,
            referee_game_type=referee_game_state_raw & 0x0F,
            referee_status=(referee_game_state_raw >> 4) & 0x0F,
            referee_stage_remain_time=referee_stage_remain_time,
            referee_robot_id=referee_robot_id,
            referee_robot_level=referee_robot_level,
            referee_current_hp=referee_current_hp,
            referee_maximum_hp=referee_maximum_hp,
            referee_shooter_barrel_cooling_value=referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit=referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit=referee_chassis_power_limit,
            referee_power_management_flags=referee_power_management_flags,
            referee_chassis_current=referee_chassis_current,
            referee_buffer_energy=referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat=referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat=referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat=referee_shooter_42mm_barrel_heat,
            referee_shoot_bullet_speed_mps=referee_shoot_bullet_speed_mps,
            rc_switches=remote_packed & 0x0F,
            motor_online_bitmap=motor_online_bitmap,
            gimbal_motor_online_bitmap=link_bitmap_packed & 0x03,
            shoot_motor_online_bitmap=(link_bitmap_packed >> 2) & 0x07,
            status_flags=status_flags,
            remote_protocol=(remote_packed >> 4) & 0x03,
            remote_mode_switch=_decode_remote_mode_switch((remote_packed >> 6) & 0x03),
            remote_button_bits=remote_state_flags & 0x1F,
            remote_control_flags=(remote_state_flags >> 5) & 0x07,
            rc_rocker_l_x=rc_rocker_l_x,
            rc_rocker_l_y=rc_rocker_l_y,
            rc_rocker_r_x=rc_rocker_r_x,
            rc_rocker_r_y=rc_rocker_r_y,
            rc_dial=rc_dial,
            mouse_x=mouse_x,
            mouse_y=mouse_y,
            mouse_z=mouse_z,
            mouse_left=mouse_buttons & 0x01,
            mouse_right=(mouse_buttons >> 1) & 0x01,
            key_pressed_bits=key_pressed_bits,
            chassis_mode=mode_packed_low & 0x03,
            gimbal_mode=(mode_packed_low >> 2) & 0x03,
            shoot_mode=(mode_packed_low >> 4) & 0x01,
            shoot_load_mode=(mode_packed_low >> 5) & 0x07,
            shoot_lid_mode=mode_packed_high & 0x01,
            shoot_friction_mode=(mode_packed_high >> 1) & 0x01,
            shoot_bullet_speed=_decode_bullet_speed((mode_packed_high >> 2) & 0x07),
            shoot_rest_heat=shoot_rest_heat,
            telemetry_drop_count=telemetry_drop_count,
            free_heap_bytes=free_heap_bytes,
            chassis_cmd_vx=chassis_cmd_vx,
            chassis_cmd_vy=chassis_cmd_vy,
            chassis_cmd_wz=chassis_cmd_wz,
            shoot_rate=shoot_rate,
            shoot_loader_speed_aps=shoot_loader_speed_aps,
            shoot_friction_l_speed_aps=shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps=shoot_friction_r_speed_aps,
            can_link_bitmap=(link_bitmap_packed >> 5) & 0x03,
            vision_meta_flags=vision_meta_flags,
            vision_recv_sp_mode=vision_sp_mode_packed & 0x0F,
            vision_send_sp_mode=(vision_sp_mode_packed >> 4) & 0x0F,
            vision_recv_yaw_raw_rad=vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s=vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2=vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad=vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s=vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2=vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q=[vision_send_q0, vision_send_q1, vision_send_q2, vision_send_q3],
            vision_send_yaw_raw_rad=vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s=vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad=vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s=vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps=vision_send_bullet_speed_mps,
            vision_send_bullet_count=vision_send_bullet_count,
            seq=seq,
            host_rx_ms=host_rx_ms,
        )

    def _parse_v7(self, raw: bytes, version: int, seq: int, host_rx_ms: int) -> TelemetryFrame:
        (
            timestamp_ms,
            chassis_power,
            chassis_volt,
            chassis_power_buffer_energy,
            chassis_power_budget_w,
            chassis_power_estimated_w,
            chassis_power_scale,
            chassis_power_mode_flags,
            target_rpm0,
            target_rpm1,
            target_rpm2,
            target_rpm3,
            rpm0,
            rpm1,
            rpm2,
            rpm3,
            imu_pitch_deg,
            imu_yaw_deg,
            imu_roll_deg,
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw,
            gimbal_pitch_encoder_raw,
            referee_game_state_raw,
            referee_stage_remain_time,
            referee_robot_id,
            referee_robot_level,
            referee_current_hp,
            referee_maximum_hp,
            referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit,
            referee_power_management_flags,
            remote_packed,
            motor_online_bitmap,
            link_bitmap_packed,
            status_flags,
            remote_state_flags,
            rc_rocker_l_x,
            rc_rocker_l_y,
            rc_rocker_r_x,
            rc_rocker_r_y,
            rc_dial,
            mouse_x,
            mouse_y,
            mouse_z,
            mouse_buttons,
            key_pressed_bits,
            mode_packed_low,
            mode_packed_high,
            shoot_rest_heat,
            referee_chassis_current,
            referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat,
            referee_shoot_bullet_speed_mps,
            telemetry_drop_count,
            free_heap_bytes,
            chassis_cmd_vx,
            chassis_cmd_vy,
            chassis_cmd_wz,
            shoot_rate,
            shoot_loader_speed_aps,
            shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps,
            vision_meta_flags,
            vision_sp_mode_packed,
            vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q0,
            vision_send_q1,
            vision_send_q2,
            vision_send_q3,
            vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps,
            vision_send_bullet_count,
        ) = PAYLOAD_STRUCT_V7.unpack_from(raw, HEADER_STRUCT.size)

        return TelemetryFrame(
            version=version,
            payload_size=PAYLOAD_SIZES[version],
            timestamp_ms=timestamp_ms,
            chassis_power=chassis_power,
            chassis_volt=chassis_volt,
            motor_target_rpm=[target_rpm0, target_rpm1, target_rpm2, target_rpm3],
            motor_rpm=[rpm0, rpm1, rpm2, rpm3],
            imu_angle_deg=[imu_pitch_deg, imu_yaw_deg, imu_roll_deg],
            imu_gyro_deg_s=[imu_gyro_x, imu_gyro_y, imu_gyro_z],
            gimbal_yaw_target_deg=gimbal_yaw_target_deg,
            gimbal_yaw_actual_deg=gimbal_yaw_actual_deg,
            gimbal_yaw_target_deg_s=gimbal_yaw_target_deg_s,
            gimbal_yaw_actual_deg_s=gimbal_yaw_actual_deg_s,
            gimbal_pitch_target_deg=gimbal_pitch_target_deg,
            gimbal_pitch_actual_deg=gimbal_pitch_actual_deg,
            gimbal_pitch_target_deg_s=gimbal_pitch_target_deg_s,
            gimbal_pitch_actual_deg_s=gimbal_pitch_actual_deg_s,
            gimbal_cmd_yaw_deg=gimbal_cmd_yaw_deg,
            gimbal_cmd_pitch_deg=gimbal_cmd_pitch_deg,
            gimbal_cmd_chassis_rotate_wz=gimbal_cmd_chassis_rotate_wz,
            gimbal_yaw_encoder_raw=gimbal_yaw_encoder_raw,
            gimbal_pitch_encoder_raw=gimbal_pitch_encoder_raw,
            referee_game_state_raw=referee_game_state_raw,
            referee_game_type=referee_game_state_raw & 0x0F,
            referee_status=(referee_game_state_raw >> 4) & 0x0F,
            referee_stage_remain_time=referee_stage_remain_time,
            referee_robot_id=referee_robot_id,
            referee_robot_level=referee_robot_level,
            referee_current_hp=referee_current_hp,
            referee_maximum_hp=referee_maximum_hp,
            referee_shooter_barrel_cooling_value=referee_shooter_barrel_cooling_value,
            referee_shooter_barrel_heat_limit=referee_shooter_barrel_heat_limit,
            referee_chassis_power_limit=referee_chassis_power_limit,
            referee_power_management_flags=referee_power_management_flags,
            referee_chassis_current=referee_chassis_current,
            referee_buffer_energy=referee_buffer_energy,
            referee_shooter_17mm_1_barrel_heat=referee_shooter_17mm_1_barrel_heat,
            referee_shooter_17mm_2_barrel_heat=referee_shooter_17mm_2_barrel_heat,
            referee_shooter_42mm_barrel_heat=referee_shooter_42mm_barrel_heat,
            referee_shoot_bullet_speed_mps=referee_shoot_bullet_speed_mps,
            rc_switches=remote_packed & 0x0F,
            motor_online_bitmap=motor_online_bitmap,
            gimbal_motor_online_bitmap=link_bitmap_packed & 0x03,
            shoot_motor_online_bitmap=(link_bitmap_packed >> 2) & 0x07,
            status_flags=status_flags,
            remote_protocol=(remote_packed >> 4) & 0x03,
            remote_mode_switch=_decode_remote_mode_switch((remote_packed >> 6) & 0x03),
            remote_button_bits=remote_state_flags & 0x1F,
            remote_control_flags=(remote_state_flags >> 5) & 0x07,
            rc_rocker_l_x=rc_rocker_l_x,
            rc_rocker_l_y=rc_rocker_l_y,
            rc_rocker_r_x=rc_rocker_r_x,
            rc_rocker_r_y=rc_rocker_r_y,
            rc_dial=rc_dial,
            mouse_x=mouse_x,
            mouse_y=mouse_y,
            mouse_z=mouse_z,
            mouse_left=mouse_buttons & 0x01,
            mouse_right=(mouse_buttons >> 1) & 0x01,
            key_pressed_bits=key_pressed_bits,
            chassis_mode=mode_packed_low & 0x03,
            gimbal_mode=(mode_packed_low >> 2) & 0x03,
            shoot_mode=(mode_packed_low >> 4) & 0x01,
            shoot_load_mode=(mode_packed_low >> 5) & 0x07,
            shoot_lid_mode=mode_packed_high & 0x01,
            shoot_friction_mode=(mode_packed_high >> 1) & 0x01,
            shoot_bullet_speed=_decode_bullet_speed((mode_packed_high >> 2) & 0x07),
            shoot_rest_heat=shoot_rest_heat,
            telemetry_drop_count=telemetry_drop_count,
            free_heap_bytes=free_heap_bytes,
            chassis_cmd_vx=chassis_cmd_vx,
            chassis_cmd_vy=chassis_cmd_vy,
            chassis_cmd_wz=chassis_cmd_wz,
            shoot_rate=shoot_rate,
            shoot_loader_speed_aps=shoot_loader_speed_aps,
            shoot_friction_l_speed_aps=shoot_friction_l_speed_aps,
            shoot_friction_r_speed_aps=shoot_friction_r_speed_aps,
            can_link_bitmap=(link_bitmap_packed >> 5) & 0x03,
            vision_meta_flags=vision_meta_flags,
            vision_recv_sp_mode=vision_sp_mode_packed & 0x0F,
            vision_send_sp_mode=(vision_sp_mode_packed >> 4) & 0x0F,
            vision_recv_yaw_raw_rad=vision_recv_yaw_raw_rad,
            vision_recv_yaw_vel_raw_rad_s=vision_recv_yaw_vel_raw_rad_s,
            vision_recv_yaw_acc_raw_rad_s2=vision_recv_yaw_acc_raw_rad_s2,
            vision_recv_pitch_raw_rad=vision_recv_pitch_raw_rad,
            vision_recv_pitch_vel_raw_rad_s=vision_recv_pitch_vel_raw_rad_s,
            vision_recv_pitch_acc_raw_rad_s2=vision_recv_pitch_acc_raw_rad_s2,
            vision_send_q=[vision_send_q0, vision_send_q1, vision_send_q2, vision_send_q3],
            vision_send_yaw_raw_rad=vision_send_yaw_raw_rad,
            vision_send_yaw_vel_raw_rad_s=vision_send_yaw_vel_raw_rad_s,
            vision_send_pitch_raw_rad=vision_send_pitch_raw_rad,
            vision_send_pitch_vel_raw_rad_s=vision_send_pitch_vel_raw_rad_s,
            vision_send_bullet_speed_mps=vision_send_bullet_speed_mps,
            vision_send_bullet_count=vision_send_bullet_count,
            seq=seq,
            host_rx_ms=host_rx_ms,
            chassis_power_buffer_energy=chassis_power_buffer_energy,
            chassis_power_budget_w=chassis_power_budget_w,
            chassis_power_estimated_w=chassis_power_estimated_w,
            chassis_power_scale=chassis_power_scale,
            chassis_power_mode_flags=chassis_power_mode_flags,
        )

    def feed(self, data: bytes, host_rx_ms: int) -> List[TelemetryFrame]:
        self._buffer.extend(data)
        frames: List[TelemetryFrame] = []
        magic_candidates = (struct.pack("<H", MAGIC), struct.pack(">H", MAGIC))

        while len(self._buffer) >= HEADER_STRUCT.size:
            if self._buffer[0:2] not in magic_candidates:
                idx_little = self._buffer.find(magic_candidates[0], 1)
                idx_big = self._buffer.find(magic_candidates[1], 1)
                idx_candidates = [idx for idx in (idx_little, idx_big) if idx >= 0]
                idx = min(idx_candidates) if idx_candidates else -1
                if idx < 0:
                    keep = len(magic_candidates[0]) - 1
                    if len(self._buffer) > keep:
                        drop = len(self._buffer) - keep
                        self.sync_drop_count += drop
                        del self._buffer[:drop]
                    break
                self.sync_drop_count += idx
                del self._buffer[:idx]
                continue

            if len(self._buffer) < HEADER_STRUCT.size:
                break

            magic, version, payload_len_low8, seq = HEADER_STRUCT.unpack_from(self._buffer)
            self.magic_hit_count += 1

            if magic not in (MAGIC, MAGIC_ALT) or version not in SUPPORTED_VERSIONS:
                self.header_mismatch_count += 1
                self.last_bad_header = (
                    f"magic=0x{magic:04x} ver={version} len_low8={payload_len_low8} seq={seq}"
                )
                self.last_bad_reason = (
                    f"magic_bad={int(magic not in (MAGIC, MAGIC_ALT))} "
                    f"ver_bad={int(version not in SUPPORTED_VERSIONS)} "
                    f"expect_magic=0x{MAGIC:04x}/0x{MAGIC_ALT:04x} "
                    f"expect_ver={'/'.join(str(v) for v in SUPPORTED_VERSIONS)}"
                )
                self.sync_drop_count += 1
                del self._buffer[:1]
                continue

            payload_size = PAYLOAD_SIZES[version]
            expected_low8 = payload_size & 0xFF
            frame_size = HEADER_STRUCT.size + payload_size + CRC_STRUCT.size
            if payload_len_low8 != expected_low8:
                self.header_mismatch_count += 1
                self.last_bad_header = (
                    f"magic=0x{magic:04x} ver={version} len_low8={payload_len_low8} seq={seq}"
                )
                self.last_bad_reason = (
                    f"len_bad=1 expect_len_low8={expected_low8} payload_size={payload_size}"
                )
                self.sync_drop_count += 1
                del self._buffer[:1]
                continue

            if len(self._buffer) < frame_size:
                break

            raw = bytes(self._buffer[:frame_size])
            expected_crc = CRC_STRUCT.unpack_from(raw, frame_size - CRC_STRUCT.size)[0]
            actual_crc = crc16_firmware(raw[:-CRC_STRUCT.size])
            if expected_crc != actual_crc:
                self.crc_error_count += 1
                self.last_bad_reason = (
                    f"crc_bad=1 crc_rx=0x{expected_crc:04x} crc_calc=0x{actual_crc:04x}"
                )
                del self._buffer[:1]
                continue

            if not self.stream_locked:
                self.stream_locked = True
                self.crc_error_count = 0
                self.sync_drop_count = 0
                self.header_mismatch_count = 0
                self.last_bad_header = ""
                self.last_bad_reason = ""

            if version >= 8:
                frame = self._parse_v7(raw, version, seq, host_rx_ms)
                metadata = V8_METADATA.unpack_from(raw, HEADER_STRUCT.size + PAYLOAD_STRUCT_V7.size)
                frame.capabilities, frame.yaw_flags, frame.pitch_flags = metadata[:3]
                frame.chassis_motor_ids = list(metadata[3:7])
                frame.gimbal_enabled, frame.gimbal_startup_ready = metadata[7:]
                if version >= 9:
                    values = V9_DIAGNOSTICS.unpack_from(raw, HEADER_STRUCT.size + PAYLOAD_STRUCT_V8.size)
                    for name, value in zip(YAW_DIAGNOSTIC_FIELDS, values):
                        setattr(frame, name, value)
                    _validate_yaw_diagnostics(frame)
                if version >= 10:
                    values = struct.unpack_from("<8f", raw, HEADER_STRUCT.size + PAYLOAD_STRUCT_V9.size)
                    frame.chassis_pid_output = list(values[:4])
                    frame.chassis_current_actual_raw = list(values[4:])
                frames.append(frame)
            elif version == 7:
                frames.append(self._parse_v7(raw, version, seq, host_rx_ms))
            elif version == 6:
                frames.append(self._parse_v6(raw, version, seq, host_rx_ms))
            elif version == 5:
                frames.append(self._parse_v5(raw, version, seq, host_rx_ms))
            elif version == 4:
                frames.append(self._parse_v4(raw, version, seq, host_rx_ms))
            else:
                frames.append(self._parse_v3(raw, version, seq, host_rx_ms))
            del self._buffer[:frame_size]

        return frames
