/*
 * 只读应用快照并发布RTT诊断帧，浮点单位在协议字段名中标出。
 * 不访问HAL、不猜测未接入数据；无效值用NaN，RTT满时丢帧而不阻塞控制。
 */
#include "dashboard.h"
#include "gimbal_monitor.h"
#include "chassis_controller.h"
#include "message_center.h"
#include "remote_messages.h"
#include "control_messages.h"
#include "sensor_messages.h"
#include "motor_service.h"
#include "robot_config.h"
#include "bsp_time.h"
#include "SEGGER_RTT.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(DashboardPayload) == 300U, "dashboard v8 payload ABI");
_Static_assert(sizeof(DashboardFrame) == 310U, "dashboard v8 frame ABI");

static uint8_t s_initialized;
static uint32_t s_sequence, s_drop_count;
static uint8_t s_buffer[DASHBOARD_RTT_BUFFER_SIZE];
/* 消息派发和Dashboard_Step均在主循环执行；各源独立记录到达时间。 */
static RemoteControlMessage s_remote;
static SensorData s_sensor;
static ChassisCmd s_chassis_cmd;
static uint32_t s_remote_ms, s_sensor_ms, s_chassis_ms;
static bool s_remote_seen, s_sensor_seen, s_chassis_seen;
static bool s_remote_subscribed, s_sensor_subscribed, s_chassis_subscribed;

static void on_message(const MsgEvent *ev, void *user) {
  (void)user;
  if (!ev) return;
  if (ev->topic == TOPIC_RC_UPDATE && ev->size == sizeof(s_remote)) {
    memcpy(&s_remote, ev->data, sizeof(s_remote));
    s_remote_ms = BspTime_NowMs(); s_remote_seen = true;
  } else if (ev->topic == TOPIC_IMU_UPDATE && ev->size == sizeof(s_sensor)) {
    memcpy(&s_sensor, ev->data, sizeof(s_sensor));
    s_sensor_ms = BspTime_NowMs(); s_sensor_seen = true;
  } else if (ev->topic == TOPIC_CHASSIS_CMD && ev->size == sizeof(s_chassis_cmd)) {
    memcpy(&s_chassis_cmd, ev->data, sizeof(s_chassis_cmd));
    s_chassis_ms = BspTime_NowMs(); s_chassis_seen = true;
  }
}

static uint16_t dashboard_crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
  }
  return crc;
}

/* 读取失败跳过本帧，不在控制回调中无限等待写者。 */
static bool copy_monitor(GimbalMonitorSnapshot *out) {
  uint32_t first = g_gimbal_monitor.sequence;
  if (first & 1U) return false;
  atomic_signal_fence(memory_order_seq_cst);
  memcpy(out, (const void *)&g_gimbal_monitor, sizeof(*out));
  atomic_signal_fence(memory_order_seq_cst);
  return first == g_gimbal_monitor.sequence && !(first & 1U);
}

void Dashboard_Init(void) {
  if (s_initialized) return;
  SEGGER_RTT_Init();
  if (SEGGER_RTT_ConfigUpBuffer(DASHBOARD_RTT_CHANNEL, "Dashboard", s_buffer,
                                sizeof(s_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0) return;
  s_remote_subscribed = MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_message, NULL) == 0;
  s_sensor_subscribed = MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_message, NULL) == 0;
  s_chassis_subscribed = MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_message, NULL) == 0;
  s_initialized = 1U;
}

static bool fresh_motor(uint8_t id, uint32_t now, MotorSnapshot *snapshot) {
  return MotorService_GetSnapshot(id, snapshot) == ROBOT_STATUS_OK &&
      snapshot->initialized && snapshot->feedback_valid &&
      (uint32_t)(now - snapshot->feedback_timestamp_ms) <= 100U;
}

static void fill_sources(DashboardPayload *p, uint32_t now) {
  const RobotConfig_t *robot = RobotConfig_Get();
  if (robot) {
    p->capabilities |= DASHBOARD_CAN;
    /* CAN灯仅说明总线上有新鲜电机反馈；不将其冒充总线错误/解锁状态。 */
    for (uint8_t i = 0; i < robot->total_motor_count; ++i) {
      const MotorConfig_t *config = &robot->motor_configs[i];
      MotorSnapshot snapshot;
      if (!fresh_motor(config->motor_id, now, &snapshot)) continue;
      if (config->can_channel == CAN_CHANNEL_1) p->link_bitmap_packed |= 1U << 5;
      if (config->can_channel == CAN_CHANNEL_2) p->link_bitmap_packed |= 1U << 6;
    }
  }
  if (s_remote_subscribed) p->capabilities |= DASHBOARD_REMOTE;
  if (s_remote_seen && (uint32_t)(now - s_remote_ms) <= 200U) {
    p->status_flags |= 0x02U;
    p->remote_packed = (uint8_t)((s_remote.rc.s[0] & 3) | ((s_remote.rc.s[1] & 3) << 2) | (1U << 4) | (3U << 6));
    p->rc_rocker_l_x = s_remote.rc.ch[2]; p->rc_rocker_l_y = s_remote.rc.ch[3];
    p->rc_rocker_r_x = s_remote.rc.ch[0]; p->rc_rocker_r_y = s_remote.rc.ch[1];
    p->rc_dial = s_remote.rc.ch[4];
    p->mouse_x = s_remote.mouse.x; p->mouse_y = s_remote.mouse.y; p->mouse_z = s_remote.mouse.z;
    p->mouse_buttons = (uint8_t)((s_remote.mouse.press_l != 0) | ((s_remote.mouse.press_r != 0) << 1));
    p->key_pressed_bits = s_remote.key.v;
  }
  if (s_sensor_subscribed) p->capabilities |= DASHBOARD_IMU;
  if (s_sensor_seen && (uint32_t)(now - s_sensor_ms) <= 100U) {
    p->status_flags |= 0x04U;
    p->imu_angle_deg[0] = s_sensor.pitch; p->imu_angle_deg[1] = s_sensor.yaw_total_angle;
    p->imu_angle_deg[2] = s_sensor.roll;
    const float rad_to_deg = 57.295779513f;
    p->imu_gyro_deg_s[0] = s_sensor.g_gx * rad_to_deg;
    p->imu_gyro_deg_s[1] = s_sensor.g_gy * rad_to_deg;
    p->imu_gyro_deg_s[2] = s_sensor.g_gz * rad_to_deg;
  }
  const ChassisController *chassis = ChassisApp_GetController();
  uint8_t count = ChassisApp_GetMotorIds(p->chassis_motor_ids);
  if (chassis && count > 0U && count <= 4U) {
    p->capabilities |= DASHBOARD_CHASSIS;
    for (uint8_t i = 0; i < count; ++i) {
      MotorSnapshot snapshot;
      const MotorConfig_t *config = MotorService_GetConfig(p->chassis_motor_ids[i]);
      /* 仅DJI适配器明确提供RPM；未知厂商的速度不能冒用此单位。 */
      if (!config || config->vendor != MOTOR_VENDOR_DJI) continue;
      if (fresh_motor(p->chassis_motor_ids[i], now, &snapshot)) {
        p->motor_rpm[i] = snapshot.speed;
        p->motor_online_bitmap |= (uint8_t)(1U << i);
      }
      /* 控制器目标为转子RPM，保持其真实数组顺序并随帧携带软件ID。 */
      if (s_chassis_subscribed && s_chassis_seen && (uint32_t)(now - s_chassis_ms) <= 100U)
        p->motor_target_rpm[i] = chassis->target_speeds[i];
    }
    if (p->motor_online_bitmap) p->status_flags |= 0x10U;
  }
  if (s_chassis_subscribed && s_chassis_seen && (uint32_t)(now - s_chassis_ms) <= 100U) {
    p->chassis_cmd_vx = s_chassis_cmd.vx; p->chassis_cmd_vy = s_chassis_cmd.vy;
    p->chassis_cmd_wz = s_chassis_cmd.wz;
    if (s_chassis_cmd.enabled) p->status_flags |= 0x40U;
  }
}

void Dashboard_Step(void) {
  DashboardFrame frame = {0};
  GimbalMonitorSnapshot monitor = {0};
  if (!s_initialized) Dashboard_Init();
  if (!s_initialized || !copy_monitor(&monitor)) return;
  frame.magic = DASHBOARD_FRAME_MAGIC;
  frame.version = DASHBOARD_FRAME_VERSION;
  frame.payload_len = (uint8_t)sizeof(DashboardPayload); /* 协议保留长度低8位，按版本取完整长度。 */
  frame.seq = s_sequence++;
  frame.payload = (DashboardPayload){
    .chassis_power = NAN,
    .chassis_volt = NAN,
    .chassis_power_budget_w = NAN,
    .chassis_estimated_power_w = NAN,
    .chassis_power_scale = NAN,
    .motor_target_rpm = {NAN, NAN, NAN, NAN},
    .motor_rpm = {NAN, NAN, NAN, NAN},
    .imu_angle_deg = {NAN, NAN, NAN},
    .imu_gyro_deg_s = {NAN, NAN, NAN},
    .gimbal_yaw_target_deg = NAN,
    .gimbal_yaw_actual_deg = NAN,
    .gimbal_yaw_target_deg_s = NAN,
    .gimbal_yaw_actual_deg_s = NAN,
    .gimbal_pitch_target_deg = NAN,
    .gimbal_pitch_actual_deg = NAN,
    .gimbal_pitch_target_deg_s = NAN,
    .gimbal_pitch_actual_deg_s = NAN,
    .gimbal_cmd_yaw_deg = NAN,
    .gimbal_cmd_pitch_deg = NAN,
    .gimbal_cmd_chassis_rotate_wz = NAN,
    .referee_shoot_bullet_speed_mps = NAN,
    .chassis_cmd_vx = NAN,
    .chassis_cmd_vy = NAN,
    .chassis_cmd_wz = NAN,
    .shoot_rate = NAN,
    .shoot_loader_speed_aps = NAN,
    .shoot_friction_l_speed_aps = NAN,
    .shoot_friction_r_speed_aps = NAN,
    .vision_recv_yaw_raw_rad = NAN,
    .vision_recv_yaw_vel_raw_rad_s = NAN,
    .vision_recv_yaw_acc_raw_rad_s2 = NAN,
    .vision_recv_pitch_raw_rad = NAN,
    .vision_recv_pitch_vel_raw_rad_s = NAN,
    .vision_recv_pitch_acc_raw_rad_s2 = NAN,
    .vision_send_q = {NAN, NAN, NAN, NAN},
    .vision_send_yaw_raw_rad = NAN,
    .vision_send_yaw_vel_raw_rad_s = NAN,
    .vision_send_pitch_raw_rad = NAN,
    .vision_send_pitch_vel_raw_rad_s = NAN,
    .vision_send_bullet_speed_mps = NAN,
  };
  DashboardPayload *p = &frame.payload;
  const uint32_t now = BspTime_NowMs();
  p->timestamp_ms = now;
  p->mode_packed_low = 3U << 2; /* 旧云台模式枚举的保留值，避免伪报零力/自由模式。 */
  p->remote_packed = 3U << 6; /* 未收到遥控时模式未知，不伪造C档。 */
  fill_sources(p, now);
  p->capabilities |= DASHBOARD_GIMBAL;
  bool fresh = monitor.magic == 0x474D4F4EU && monitor.version == 1U &&
      monitor.size_bytes == sizeof(monitor) && (uint32_t)(now - monitor.tick_ms) <= 100U;
  if (fresh) {
    p->status_flags |= 0x08U;
    p->gimbal_enabled = monitor.enabled != 0;
    p->gimbal_startup_ready = monitor.startup_ready != 0;
    p->yaw_flags = (uint8_t)monitor.yaw.flags;
    p->pitch_flags = (uint8_t)monitor.pitch.flags;
    if (monitor.enabled) p->status_flags |= 0x20U;
    /* 旧模式枚举与当前命令不一一对应；v8消费者使用enabled/轴flags。 */
    p->gimbal_yaw_encoder_raw = (uint16_t)monitor.yaw.encoder_raw;
    p->gimbal_pitch_encoder_raw = (uint16_t)monitor.pitch.encoder_raw;
    if (monitor.yaw.flags & GIMBAL_MONITOR_FEEDBACK_FRESH) {
      p->link_bitmap_packed |= 1U;
      p->gimbal_yaw_actual_deg = monitor.yaw.position_actual_ticks * (360.0f / 8192.0f);
      /* spin世界角速度目标必须与同坐标的内环反馈比较。 */
      p->gimbal_yaw_actual_deg_s = monitor.yaw.speed_loop_actual_rpm * 6.0f;
      if (!(monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE))
        p->gimbal_yaw_actual_deg_s = monitor.yaw.speed_actual_rpm * 6.0f;
    }
    if (monitor.pitch.flags & GIMBAL_MONITOR_FEEDBACK_FRESH) {
      p->link_bitmap_packed |= 2U;
      p->gimbal_pitch_actual_deg = monitor.pitch.position_actual_ticks * (360.0f / 8192.0f);
      p->gimbal_pitch_actual_deg_s = monitor.pitch.speed_actual_rpm * 6.0f;
    }
    if (monitor.yaw.flags & GIMBAL_MONITOR_POSITION_ACTIVE)
      p->gimbal_yaw_target_deg = monitor.yaw.position_target_ticks * (360.0f / 8192.0f);
    if (monitor.pitch.flags & GIMBAL_MONITOR_POSITION_ACTIVE)
      p->gimbal_pitch_target_deg = monitor.pitch.position_target_ticks * (360.0f / 8192.0f);
    if (monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE)
      p->gimbal_yaw_target_deg_s = monitor.yaw.speed_target_rpm * 6.0f;
    if (monitor.pitch.flags & GIMBAL_MONITOR_CONTROL_ACTIVE)
      p->gimbal_pitch_target_deg_s = monitor.pitch.speed_target_rpm * 6.0f;
  }
  p->telemetry_drop_count = s_drop_count;
  frame.crc16 = dashboard_crc16((const uint8_t *)&frame, (uint16_t)(sizeof(frame) - sizeof(frame.crc16)));
  if (SEGGER_RTT_Write(DASHBOARD_RTT_CHANNEL, &frame, sizeof(frame)) != sizeof(frame)) s_drop_count++;
}
