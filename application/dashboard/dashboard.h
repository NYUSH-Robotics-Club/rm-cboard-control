#ifndef RM_DASHBOARD_H
#define RM_DASHBOARD_H

#include <stdint.h>

#define DASHBOARD_RTT_CHANNEL 1U
#define DASHBOARD_RTT_BUFFER_SIZE 2048U
#define DASHBOARD_FRAME_MAGIC 0x4452U
#define DASHBOARD_FRAME_VERSION 7U

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
} DashboardPayload;
typedef struct { uint16_t magic; uint8_t version, payload_len; uint32_t seq; DashboardPayload payload; uint16_t crc16; } DashboardFrame;
#pragma pack(pop)

void Dashboard_Init(void);
void Dashboard_Step(void);
#endif
