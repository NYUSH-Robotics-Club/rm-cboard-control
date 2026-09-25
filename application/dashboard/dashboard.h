/* 将应用诊断打包为RTT遥测。版本10保留版本9前缀，并附带底盘速度环诊断。
 * 浮点NaN表示未接入、过期或未启用的测量/目标，不能画成零值。只观察，不改变控制。
 */
#ifndef RM_DASHBOARD_H
#define RM_DASHBOARD_H

#include <stdint.h>

#define DASHBOARD_RTT_CHANNEL 1U
#define DASHBOARD_RTT_BUFFER_SIZE 2048U
#define DASHBOARD_FRAME_MAGIC 0x4452U
#define DASHBOARD_FRAME_VERSION 10U

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
  /* v8 yaw为当前有效回调的连续角度参考（度），仅速度环时也有值；
   * 是否用于位置闭环由yaw_flags的POSITION_ACTIVE标明。其余两字段未接入。 */
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
  /* v9：同一云台回调的快照；diag_valid=0时整个尾部无效。
   * trace_flags使用GIMBAL_TRACE_*；无来源时不能把ch0=0当作真实回中。
   * ms来自MCU，序号自然回绕；callback_count不是遥测seq或UART帧数。
   */
  uint32_t yaw_diag_valid, yaw_sample_ms, yaw_callback_count, yaw_callback_dt_ms;
  uint32_t yaw_trace_flags, yaw_rc_sequence, yaw_rc_dispatch_ms, yaw_route_sequence, yaw_route_ms;
  int32_t yaw_rc_ch0;
  float yaw_route_rate;
  uint32_t yaw_mode, yaw_feedback_ms;
  float yaw_speed_raw_rpm, yaw_pid_dt_s;
  /* command_raw是应用限幅后的服务请求，不是CAN发送确认；电流均为原始刻度，非安培。 */
  uint32_t yaw_command_unit;
  int32_t yaw_command_status, yaw_command_raw, yaw_current_actual_raw;
  float yaw_pid_pout, yaw_pid_iout, yaw_pid_dout, yaw_pid_output;
  float yaw_pid_kp, yaw_pid_ki, yaw_pid_kd, yaw_pid_output_max, yaw_pid_integral_max;
  /* 四轮顺序与chassis_motor_ids、motor_target_rpm和motor_rpm一致。
   * pid_output是速度PID最终输出的原始电流刻度；current_actual_raw是电调反馈原始刻度。
   * 无新鲜反馈或控制未运行时发布NaN，避免把停机零值当成有效调节数据。 */
  float chassis_pid_output[4], chassis_current_actual_raw[4];
} DashboardPayload;
typedef struct { uint16_t magic; uint8_t version, payload_len; uint32_t seq; DashboardPayload payload; uint16_t crc16; } DashboardFrame;
#pragma pack(pop)

/* 主循环中初始化订阅和非阻塞RTT通道；失败不发布伪造的有效数据。可重复调用。 */
void Dashboard_Init(void);
/* 主循环读取最新应用状态，写一帧；缓冲区满则丢弃本帧并计数，不阻塞控制。 */
void Dashboard_Step(void);
#endif
