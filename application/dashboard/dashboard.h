/* 将应用诊断打包为RTT遥测。版本8保留版本7前缀，并附带数据能力、轴状态和电机编号。
 * 浮点NaN表示未接入、过期或未启用的测量/目标，不能画成零值。只观察，不改变控制。
 */
#ifndef RM_DASHBOARD_H
#define RM_DASHBOARD_H

#include <stdint.h>

#define DASHBOARD_RTT_CHANNEL 1U
#define DASHBOARD_RTT_BUFFER_SIZE 2048U
#define DASHBOARD_FRAME_MAGIC 0x4452U
#define DASHBOARD_FRAME_VERSION 8U

enum {
  DASHBOARD_CAN = 1U,      /* CAN位表示该总线有配置电机在100ms内反馈，不代表总线无错误。 */
  DASHBOARD_REMOTE = 2U,
  DASHBOARD_CHASSIS = 4U,
  DASHBOARD_IMU = 8U,
  DASHBOARD_GIMBAL = 16U
};

#pragma pack(push, 1)
typedef struct {
  uint32_t timestamp_ms;
  float chassis_power, chassis_volt;
  uint16_t chassis_power_buffer_energy;
  float chassis_power_budget_w, chassis_estimated_power_w, chassis_power_scale;
  uint8_t chassis_power_mode_flags;
  float motor_target_rpm[4], motor_rpm[4], imu_angle_deg[3], imu_gyro_deg_s[3];
  float gimbal_yaw_target_deg, gimbal_yaw_actual_deg, gimbal_yaw_target_deg_s, gimbal_yaw_actual_deg_s;
  float gimbal_pitch_target_deg, gimbal_pitch_actual_deg, gimbal_pitch_target_deg_s, gimbal_pitch_actual_deg_s;
  float gimbal_cmd_yaw_deg, gimbal_cmd_pitch_deg, gimbal_cmd_chassis_rotate_wz;
  uint16_t gimbal_yaw_encoder_raw, gimbal_pitch_encoder_raw;
  uint8_t referee_game_state; uint16_t referee_stage_remain_time; uint8_t referee_robot_id, referee_robot_level;
  uint16_t referee_current_hp, referee_maximum_hp, referee_shooter_barrel_cooling_value, referee_shooter_barrel_heat_limit, referee_chassis_power_limit;
  uint8_t referee_power_management_flags, remote_packed, motor_online_bitmap, link_bitmap_packed, status_flags, remote_state_flags;
  int16_t rc_rocker_l_x, rc_rocker_l_y, rc_rocker_r_x, rc_rocker_r_y, rc_dial, mouse_x, mouse_y, mouse_z;
  uint8_t mouse_buttons; uint16_t key_pressed_bits; uint8_t mode_packed_low, mode_packed_high, shoot_rest_heat;
  uint16_t referee_chassis_current, referee_buffer_energy, referee_shooter_17mm_1_barrel_heat, referee_shooter_17mm_2_barrel_heat, referee_shooter_42mm_barrel_heat;
  float referee_shoot_bullet_speed_mps; uint32_t telemetry_drop_count, free_heap_bytes;
  float chassis_cmd_vx, chassis_cmd_vy, chassis_cmd_wz, shoot_rate, shoot_loader_speed_aps, shoot_friction_l_speed_aps, shoot_friction_r_speed_aps;
  uint8_t vision_meta_flags, vision_sp_mode_packed;
  float vision_recv_yaw_raw_rad, vision_recv_yaw_vel_raw_rad_s, vision_recv_yaw_acc_raw_rad_s2, vision_recv_pitch_raw_rad, vision_recv_pitch_vel_raw_rad_s, vision_recv_pitch_acc_raw_rad_s2;
  float vision_send_q[4], vision_send_yaw_raw_rad, vision_send_yaw_vel_raw_rad_s, vision_send_pitch_raw_rad, vision_send_pitch_vel_raw_rad_s, vision_send_bullet_speed_mps;
  uint16_t vision_send_bullet_count;
  uint32_t capabilities; /* 表示生产端已接入哪些数据，在线/有效性由状态位和NaN区分。 */
  uint8_t yaw_flags, pitch_flags; /* GimbalMonitorAxis.flags；位置旁路时不发布位置目标。 */
  uint8_t chassis_motor_ids[4]; /* 与四组速度数组同序；0表示该槽没有配置电机。 */
  uint8_t gimbal_enabled, gimbal_startup_ready;
} DashboardPayload;
typedef struct { uint16_t magic; uint8_t version, payload_len; uint32_t seq; DashboardPayload payload; uint16_t crc16; } DashboardFrame;
#pragma pack(pop)

/* 主循环中初始化订阅和非阻塞RTT通道；失败不发布伪造的有效数据。可重复调用。 */
void Dashboard_Init(void);
/* 主循环读取最新应用状态，写一帧；缓冲区满则丢弃本帧并计数，不阻塞控制。 */
void Dashboard_Step(void);
#endif
