/*
 * 保存现有哨兵舵轮机器人的电机和控制参数。
 * 当前是两转向电机的旧实现，不代表通用四模块舵轮。
 *
 * 云台前馈入口为yaw电机的.feedforward；本车型没有pitch，不给底盘转向电机套用此入口。
 * ff = clamp(velocity_gain * speed_target_rpm + bias, -output_max, +output_max)，
 * 叠加到速度PID之后，最终仍受协议命令限幅。速度目标取yaw模式限速后的内环目标。
 * velocity_gain填原始命令刻度/RPM，通常为正；bias填有符号原始刻度，零速时仍会输出。
 * output_max填前馈绝对限幅，0关闭整项并忽略其余字段；默认全0，保持原行为。
 * 例如10刻度/RPM乘5RPM为50刻度，仅说明单位，不是推荐参数；按实测匀速出力填写。
 * 本车yaw为电压协议，不能把步兵电流协议的系数直接抄来，也不额外乘direction。
 * 先测定系数并设置小幅值上限，再逐步验证；PID增益为0时前馈仍可产生输出。
 * 禁用/反馈失联时不输出；启用后非有限参数、负限幅或计算溢出会停止云台并重新对齐。
 * offline_alarm_id填写电调硬件ID，红灯按该数报码；不是软件motor_id或CAN帧ID。
 * 蜂鸣次数取can_channel（CAN1一次、CAN2两次），每组红灯之后用蓝灯分隔。
 */
#include "sentry_swerve.h"

/* 哨兵保留原模式限速；旧70刻度按200Hz参考换算为度/秒，不代表已实测周期。 */
static const YawControlConfig s_yaw_control = {
    .manual_rate_deg_s = 615.234375f,
    .manual_speed_rpm = 300.0f,
    .vision_speed_rpm = 500.0f,
    .spin_speed_rpm = 500.0f,
    .speed_loop_only = false, /* 哨兵继续执行位置/速度串级。 */
    .near_target_lead_deg = 180.0f,
    .near_error_deg = 10.0f,
    .near_speed_rpm = 10.0f,
    .brake_speed_rpm = 10.0f,
    .near_damping_gain = 0.35f,
    .approach_target_lead_deg = 180.0f,
    .approach_error_deg = 30.0f,
    .approach_speed_rpm = 30.0f,
    .approach_damping_gain = 0.35f,
    .far_target_lead_deg = 180.0f,
    .far_damping_gain = 0.35f
};

/**
 * Sentry / Swerve Chassis Config
 * - 4x M3508: wheel drive motors
 * - 2x GM6020: wheel rotator / steering motors
 *
 * NOTE: Shooter/gimbal counts are set to 0 in RobotConfig_t.
 */

// ========== MOTOR LIST ==========
static const MotorConfig_t g_motor_configs_sentry_swerve[] = {

    // --------------------------
    // DRIVE (M3508) — CAN1
    // --------------------------
    // Wheel 0 drive (right-omniwheel)
    {
        .motor_id = 0, .offline_alarm_id = 1,
        .vendor = MOTOR_VENDOR_DJI,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .control_mode = MOTOR_CONTROL_APPLICATION,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = 1,
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f},
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 1 drive (front normal wheel)
    {.motor_id = 1, .offline_alarm_id = 2, .vendor = MOTOR_VENDOR_DJI, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .control_mode = MOTOR_CONTROL_APPLICATION, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x202, .can_tx_id = 0x200, .tx_slot = 1, .direction = -1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {8.0f, 0.0f, 0.1f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 2 drive (left-omniwheel)
    {.motor_id = 2, .offline_alarm_id = 3, .vendor = MOTOR_VENDOR_DJI, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .control_mode = MOTOR_CONTROL_APPLICATION, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x203, .can_tx_id = 0x200, .tx_slot = 2, .direction = 1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {10.0f, 0.0f, 0.1f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // Wheel 3 drive (back normal wheel)
    {.motor_id = 3, .offline_alarm_id = 4, .vendor = MOTOR_VENDOR_DJI, .type = MOTOR_TYPE_M3508, .role = MOTOR_ROLE_CHASSIS_DRIVE, .control_mode = MOTOR_CONTROL_APPLICATION, .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x204, .can_tx_id = 0x200, .tx_slot = 3, .direction = -1, .limits.m3508 = {.speed_limit = 10000.0f}, .pid_outer = {8.0f, 0.0f, 0.005f, 15000.0f, 7500.0f}, .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // --------------------------
    // STEERING (GM6020) — CAN1
    // --------------------------
    //  (GM6020 ID 5 => RX 0x209)
    {
        .motor_id = 5, .offline_alarm_id = 5, .vendor = MOTOR_VENDOR_DJI, .type = MOTOR_TYPE_GM6020, .role = MOTOR_ROLE_CHASSIS_STEER, .control_mode = MOTOR_CONTROL_APPLICATION, .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x209, // 0x204 + 5
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 0,       // For 0x2FF, slot = (motor_id - 5)
        .direction = 1,
        .limits.gm6020 = {.angle_min = 0.0f, .angle_max = 8192.0f, .gravity_compensation = 0.0f, .initial_angle = 1084.0f},
        .pid_outer = {0.7f, 0.045f, 0.018f, 300.0f, 300.0f}, // Yaw angle PID
        .pid_inner = {22.0f, 0.01f, 3.0f, 30000.0f, 4000.0f} // Yaw speed PID
    },

    // Steer motor B (GM6020 ID 6 => RX 0x20A)
    {
        .motor_id = 6, .offline_alarm_id = 6, .vendor = MOTOR_VENDOR_DJI, .type = MOTOR_TYPE_GM6020, .role = MOTOR_ROLE_CHASSIS_STEER, .control_mode = MOTOR_CONTROL_APPLICATION, .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x20A, // 0x204 + 6
        .can_tx_id = 0x2FF,
        .tx_slot = 1,
        .direction = 1,
        .limits.gm6020 = {.angle_min = 0.0f, .angle_max = 8192.0f, .gravity_compensation = 0.0f, .initial_angle = 2434.0f},
        .pid_outer = {0.7f, 0.045f, 0.018f, 300.0f, 300.0f},
        .pid_inner = {22.0f, 0.01f, 3.0f, 30000.0f, 4000.0f}},

    // --------------------------
    // GIMBAL YAW (GM6020) — CAN1
    // --------------------------
    // Yaw gimbal motor (GM6020 ID 7 => RX 0x20B)
    // DUAL-LOOP CONTROL: angle → speed → current (same as infantry)
    {
        .motor_id = 7, .offline_alarm_id = 7,
        .vendor = MOTOR_VENDOR_DJI,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .feedforward = {.velocity_gain = 0.0f, .bias = 0.0f, .output_max = 0.0f},
        .yaw_control = &s_yaw_control,
        .control_mode = MOTOR_CONTROL_APPLICATION,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x20B, // 0x204 + 7
        .can_tx_id = 0x2FF, // GM6020 ID 5-7 use 0x2FF
        .tx_slot = 2,       // motor_id - 5 = 7 - 5 = 2
        .direction = 1,
        .limits.gm6020 = {
            .angle_min = 0.0f,
            .angle_max = 8192.0f,
            .gravity_compensation = 0.0f,
            .initial_angle = -1.0f // Auto-init: use current angle on first feedback
        },
        .pid_outer = {0.7f, 0.045f, 0.018f, 300.0f, 300.0f},
        .pid_inner = {22.0f, 0.01f, 3.0f, 30000.0f, 4000.0f} // Speed → Current
    }};

// ========== ROBOT CONFIG ==========
const RobotConfig_t g_robot_config_sentry_swerve = {
    .name = "Sentry Swerve Standard",
    .chassis_type = CHASSIS_TYPE_SWERVE,
    .chassis_motor_count = 6, // 4 drive + 2 steer
    .gimbal_motor_count = 1,  // 1 yaw motor
    .shooter_motor_count = 0,
    .motor_configs = g_motor_configs_sentry_swerve,
    .total_motor_count = 7,   // 6 chassis + 1 gimbal
    .enable_imu_calibration = 0 // Sentry does not need IMU calibration
};
