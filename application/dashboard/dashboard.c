#include "dashboard.h"
#include "gimbal_monitor.h"
#include "bsp_time.h"
#include "SEGGER_RTT.h"
#include <string.h>

_Static_assert(sizeof(DashboardPayload) == 288U, "dashboard payload ABI");
_Static_assert(sizeof(DashboardFrame) == 298U, "dashboard frame ABI");

static uint8_t s_initialized;
static uint32_t s_sequence;
static uint32_t s_drop_count;
static uint8_t s_buffer[DASHBOARD_RTT_BUFFER_SIZE];

static uint16_t dashboard_crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit) crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
  }
  return crc;
}

static float ticks_to_deg(float ticks) { return ticks * (360.0f / 8192.0f); }

static void copy_monitor(GimbalMonitorSnapshot *out) {
  uint32_t first;
  do {
    first = g_gimbal_monitor.sequence;
    if (first & 1U) continue;
    memcpy(out, (const void *)&g_gimbal_monitor, sizeof(*out));
  } while (first != g_gimbal_monitor.sequence || (first & 1U));
}

void Dashboard_Init(void) {
  if (s_initialized) return;
  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(DASHBOARD_RTT_CHANNEL, "Dashboard", s_buffer,
                            sizeof(s_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  s_initialized = 1U;
}

void Dashboard_Step(void) {
  DashboardFrame frame = {0};
  GimbalMonitorSnapshot monitor = {0};
  if (!s_initialized) Dashboard_Init();
  copy_monitor(&monitor);
  frame.magic = DASHBOARD_FRAME_MAGIC;
  frame.version = DASHBOARD_FRAME_VERSION;
  frame.payload_len = (uint8_t)sizeof(DashboardPayload);
  frame.seq = s_sequence++;
  frame.payload.timestamp_ms = BspTime_NowMs();
  frame.payload.gimbal_yaw_target_deg = ticks_to_deg(monitor.yaw.position_target_ticks);
  frame.payload.gimbal_yaw_actual_deg = ticks_to_deg(monitor.yaw.position_actual_ticks);
  frame.payload.gimbal_yaw_target_deg_s = monitor.yaw.speed_target_rpm * 6.0f;
  frame.payload.gimbal_yaw_actual_deg_s = monitor.yaw.speed_actual_rpm * 6.0f;
  frame.payload.gimbal_pitch_target_deg = ticks_to_deg(monitor.pitch.position_target_ticks);
  frame.payload.gimbal_pitch_actual_deg = ticks_to_deg(monitor.pitch.position_actual_ticks);
  frame.payload.gimbal_pitch_target_deg_s = monitor.pitch.speed_target_rpm * 6.0f;
  frame.payload.gimbal_pitch_actual_deg_s = monitor.pitch.speed_actual_rpm * 6.0f;
  frame.payload.gimbal_yaw_encoder_raw = (uint16_t)monitor.yaw.encoder_raw;
  frame.payload.gimbal_pitch_encoder_raw = (uint16_t)monitor.pitch.encoder_raw;
  frame.payload.link_bitmap_packed = (uint8_t)(((monitor.yaw.flags & GIMBAL_MONITOR_FEEDBACK_FRESH) ? 1U : 0U) |
                                               ((monitor.pitch.flags & GIMBAL_MONITOR_FEEDBACK_FRESH) ? 2U : 0U));
  frame.payload.status_flags = (uint8_t)(((monitor.yaw.flags | monitor.pitch.flags) & GIMBAL_MONITOR_FEEDBACK_SEEN) ? 0x0CU : 0U);
  frame.payload.telemetry_drop_count = s_drop_count;
  frame.crc16 = dashboard_crc16((const uint8_t *)&frame, (uint16_t)(sizeof(frame) - sizeof(frame.crc16)));
  if (SEGGER_RTT_Write(DASHBOARD_RTT_CHANNEL, &frame, sizeof(frame)) != sizeof(frame)) s_drop_count++;
}
