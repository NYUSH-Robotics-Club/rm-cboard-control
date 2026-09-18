/* Exercise the real dashboard producer with deterministic application data.
 * RTT writes are captured as binary frames; no hardware or control output is used.
 */
#include "dashboard.h"
#include "gimbal_monitor.h"
#include "chassis_controller.h"
#include "message_center.h"
#include "remote_messages.h"
#include "control_messages.h"
#include "motor_service.h"
#include "robot_config.h"
#include "SEGGER_RTT.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile GimbalMonitorSnapshot g_gimbal_monitor;
static uint32_t now = 1000, feedback_ms = 1000;
static unsigned writes;
static bool full;
static DashboardFrame captured;
static ChassisController chassis = {.target_speeds = {100, -200, 300, -400}};
static const MotorConfig_t motors[] = {
    {.motor_id = 2, .can_channel = CAN_CHANNEL_1},
    {.motor_id = 1, .can_channel = CAN_CHANNEL_1},
    {.motor_id = 4, .can_channel = CAN_CHANNEL_2},
    {.motor_id = 3, .can_channel = CAN_CHANNEL_2},
};
static const RobotConfig_t robot = {.motor_configs = motors, .total_motor_count = 4};
uint32_t BspTime_NowMs(void) { return now; }
const RobotConfig_t *RobotConfig_Get(void) { return &robot; }
const MotorConfig_t *MotorService_GetConfig(uint8_t id) {
    for (unsigned i = 0; i < 4; ++i) if (motors[i].motor_id == id) return &motors[i];
    return NULL;
}
RobotStatus MotorService_GetSnapshot(uint8_t id, MotorSnapshot *out) {
    *out = (MotorSnapshot){.initialized = true, .feedback_valid = true,
        .feedback_timestamp_ms = feedback_ms, .speed = id * 10.0f};
    return ROBOT_STATUS_OK;
}
const ChassisController *ChassisApp_GetController(void) { return &chassis; }
uint8_t ChassisApp_GetMotorIds(uint8_t ids[4]) {
    const uint8_t order[] = {2, 1, 4, 3}; memcpy(ids, order, sizeof(order)); return 4;
}
void SEGGER_RTT_Init(void) {}
int SEGGER_RTT_ConfigUpBuffer(unsigned index, const char *name, void *buffer, unsigned size, unsigned flags) {
    (void)name; (void)buffer; (void)flags;
    assert(index == 1 && size >= sizeof(DashboardFrame)); return 0;
}
unsigned SEGGER_RTT_Write(unsigned index, const void *data, unsigned size) {
    assert(index == 1 && size == sizeof(captured));
    memcpy(&captured, data, size); ++writes;
    return full ? 0 : size;
}
static void emit(void) { assert(fwrite(&captured, sizeof(captured), 1, stdout) == 1); }
int main(void) {
    MsgEvent events[16]; MsgCenter_Init(events, 16); Dashboard_Init();
    RemoteControlMessage rc = {.rc = {.ch = {111, -222, 333, -444, 555}, .s = {2, 3}}};
    SensorData imu = {.yaw_total_angle = 720, .pitch = 10, .roll = -20, .g_gz = 1};
    ChassisCmd cmd = {.vx = 0.25f, .vy = -0.5f, .wz = 0.75f, .enabled = true};
    assert(MsgCenter_Publish(TOPIC_RC_UPDATE, &rc, sizeof(rc)) == 0);
    assert(MsgCenter_Publish(TOPIC_IMU_UPDATE, &imu, sizeof(imu)) == 0);
    assert(MsgCenter_Publish(TOPIC_CHASSIS_CMD, &cmd, sizeof(cmd)) == 0);
    MsgCenter_Dispatch();
    g_gimbal_monitor = (GimbalMonitorSnapshot){.sequence = 2, .magic = 0x474D4F4E,
        .version = GIMBAL_MONITOR_VERSION, .size_bytes = sizeof(GimbalMonitorSnapshot), .tick_ms = now,
        .callback_count = 123, .callback_dt_ms = 4,
        .command_trace = {.flags = 15, .rc_sequence = 17, .rc_dispatch_ms = 990,
            .route_sequence = 122, .route_ms = 998, .rc_ch0 = -660},
        .yaw_route_rate = 1, .yaw_mode = 0,
        .yaw_pid_pout = 240, .yaw_pid_iout = -2, .yaw_pid_dout = 3, .yaw_pid_output = 230,
        .yaw_pid_kp = 120, .yaw_pid_ki = 1, .yaw_pid_kd = 0.5f,
        .yaw_pid_output_max = 6000, .yaw_pid_integral_max = 200,
        .enabled = 1, .startup_ready = 1,
        .yaw = {.flags = 15, .position_actual_ticks = 16384, .position_target_ticks = 8192,
            .speed_actual_rpm = 3, .speed_loop_actual_rpm = 2, .speed_target_rpm = 4,
            .encoder_raw = 1234, .feedback_ms = 996, .speed_loop_dt_s = 0.004f,
            .command_raw = 321, .command_status = 0, .command_unit = 2, .current_actual_raw = -1234},
        .pitch = {.flags = 31, .position_actual_ticks = 2048, .position_target_ticks = 1024,
            .speed_actual_rpm = -2, .speed_target_rpm = -3, .encoder_raw = 2048}};
    Dashboard_Step();
    assert(captured.version == 9 && captured.payload_len == (412 & 255));
    assert(captured.payload.yaw_diag_valid == 1 && captured.payload.yaw_rc_ch0 == -660);
    assert(captured.payload.yaw_rc_sequence == 17 && captured.payload.yaw_route_sequence == 122);
    assert(captured.payload.yaw_command_raw == 321 && captured.payload.yaw_current_actual_raw == -1234);
    assert(captured.payload.yaw_pid_output == 230 && captured.payload.yaw_pid_pout == 240);
    assert(captured.payload.capabilities == 31);
    assert((captured.payload.link_bitmap_packed & 0x63) == 0x63);
    assert(captured.payload.rc_rocker_r_x == 111 && captured.payload.rc_rocker_l_x == 333);
    assert(isnan(captured.payload.gimbal_yaw_target_deg));
    assert(captured.payload.gimbal_cmd_yaw_deg == 360);
    assert(captured.payload.gimbal_yaw_target_deg_s == 24);
    assert(captured.payload.gimbal_yaw_actual_deg == 720);
    assert(captured.payload.gimbal_yaw_actual_deg_s == 12);
    assert(captured.payload.gimbal_pitch_target_deg == 45);
    assert(fabsf(captured.payload.imu_gyro_deg_s[2] - 57.29578f) < 0.001f);
    assert(captured.payload.motor_rpm[0] == 20 && captured.payload.motor_target_rpm[1] == -200);
    assert(isnan(captured.payload.chassis_power)); emit();
    now += 201; Dashboard_Step();
    assert(captured.payload.link_bitmap_packed == 0 && captured.payload.status_flags == 0);
    assert(isnan(captured.payload.gimbal_yaw_actual_deg) && isnan(captured.payload.motor_rpm[0]));
    assert(isnan(captured.payload.gimbal_cmd_yaw_deg));
    assert(isnan(captured.payload.gimbal_yaw_target_deg_s));
    assert(captured.payload.yaw_diag_valid == 0 && isnan(captured.payload.yaw_pid_output));
    assert(isnan(captured.payload.chassis_cmd_vx) && isnan(captured.payload.imu_angle_deg[0])); emit();
    g_gimbal_monitor.sequence = 3; unsigned before = writes; Dashboard_Step(); assert(writes == before);
    g_gimbal_monitor.sequence = 4; g_gimbal_monitor.tick_ms = now;
    g_gimbal_monitor.yaw.flags = 31; Dashboard_Step();
    assert(captured.payload.gimbal_yaw_target_deg == 360); emit();
    full = true; Dashboard_Step(); full = false; Dashboard_Step();
    assert(captured.payload.telemetry_drop_count == 1); emit();
    /* Fresh feedback alone must not publish stale PID/reference targets. */
    g_gimbal_monitor.enabled = 0; g_gimbal_monitor.startup_ready = 0;
    g_gimbal_monitor.yaw.flags = 7; Dashboard_Step();
    assert(captured.payload.yaw_diag_valid == 1 && isnan(captured.payload.yaw_pid_output));
    assert(isnan(captured.payload.gimbal_cmd_yaw_deg));
    assert(isnan(captured.payload.gimbal_yaw_target_deg));
    assert(isnan(captured.payload.gimbal_yaw_target_deg_s)); emit();
    /* A valid negative continuous reference must keep its sign and turns. */
    g_gimbal_monitor.enabled = 1; g_gimbal_monitor.startup_ready = 1;
    g_gimbal_monitor.yaw.flags = 15;
    g_gimbal_monitor.yaw.position_target_ticks = -12288;
    g_gimbal_monitor.yaw.speed_target_rpm = -4; Dashboard_Step();
    assert(captured.payload.gimbal_cmd_yaw_deg == -540);
    assert(captured.payload.gimbal_yaw_target_deg_s == -24);
    assert(isnan(captured.payload.gimbal_yaw_target_deg)); emit();
    return 0;
}
