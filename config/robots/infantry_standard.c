/*
 * 保存步兵全向轮机器人的几何、电机、方向、限幅和控制参数；修改后须重新编译验证。
 * yaw/pitch前馈在各轴.feedforward填写，默认全0关闭：
 *   ff = clamp(velocity_gain * speed_target_rpm + bias, ±output_max)。
 * velocity_gain单位为原始命令刻度/RPM；bias为有符号原始刻度，零速时也会出力。
 * output_max填正数才启用，0关闭整项；速度目标取内环限速后的RPM，不额外乘direction。
 * yaw使用电流协议、pitch使用电压协议，系数不能互抄；前馈独立于PID，未知系数先填0。
 * pitch原重力补偿单独叠加，勿用bias重复补偿；失联或参数非法时双轴停止。
 * pitch限位填写绝对编码：angle_min=1566（最高），angle_max=2205（最低）。
 * initial_angle=1971为启动/重新对齐目标；必须在限位内，负数改为锁存当前位置。
 * gravity_zero_angle=1971为重力正弦项零点（绝对编码），独立于启动目标：
 *   gravity_ff = direction * gravity_compensation * sin((raw - gravity_zero_angle) * 2π / 8192)。
 * 在该零点重力项为0；gravity_compensation填原始命令幅值，0关闭重力项。
 * pitch的enable_yaw_pitch_compensation=false关闭yaw转动对pitch目标的改写。
 * true恢复旧耦合算法（机械模型未验证）；该开关独立于重力补偿，自瞄仍禁用耦合。
 * offline_alarm_id填写电调硬件ID：红灯按该数闪烁，每组之后蓝灯分隔；
 * 蜂鸣次数取CAN总线号（CAN1一次、CAN2两次），不是0x201等报文ID。
 */
#include "infantry_standard.h"

static const ChassisFollowConfig s_chassis_follow = {
    .yaw_forward_ticks = 4555U, /* 用户确认的头朝前绝对编码。 */
    .yaw_ccw_sign = -1,         /* 暂按增大为左转；安装方向尚待实车确认。 */
};

/* 手动输入按度/秒积分；普通推杆与回中共用限速。 */
static const YawControlConfig s_yaw_control = {
    .manual_rate_deg_s = 100.0f,
    .target_lead_deg = 30.0f,
    .manual_speed_rpm = 30.0f,
    .vision_speed_rpm = 10.0f,
    .spin_speed_rpm = 10.0f,
    .speed_loop_only = true /* 步兵暂时旁路位置环；哨兵保留串级。 */
};

/**
 * @brief Standard Infantry Robot Configuration
 *
 * - 4x M3508 chassis motors (omni wheels)
 * - 2x GM6020 gimbal motors (pitch + yaw)
 * - 3x M3508 shooter motors (turntable + 2x friction wheels)
 */

/*
 * motor_id 是全局唯一的软件编号；重复编号会使电机服务拒绝初始化。
 * 硬件 ID 由电机/电调设置，下面的 CAN 地址和槽位与它对应，不会修改硬件 ID。
 * CAN1：底盘硬件 ID 1~4、Yaw ID 5；CAN2：左右摩擦轮 ID 1/2、拨弹 ID 3、Pitch ID 4。
 * tx_slot 从 0 开始，每槽占控制报文的两个字节，高字节在前。
 */
static const MotorConfig_t g_motor_configs_infantry_standard[] = {
    // ========== CHASSIS MOTORS (4x M3508) ==========
    // 右后轮：CAN1 硬件 ID 1，软件编号 1。
    {
        .motor_id = 1, .offline_alarm_id = 1,
        .vendor = MOTOR_VENDOR_DJI,
        .type = MOTOR_TYPE_M3508,
        .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .control_mode = MOTOR_CONTROL_APPLICATION,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x201,
        .can_tx_id = 0x200,
        .tx_slot = 0,
        .direction = -1, // 旧安装方向值；全向轮启用前须核对正转方向。
        .limits.m3508 = {.speed_limit = 10000.0f},
        .pid_outer = {15.0f, 0.0f, 0.0f, 12000.0f, 3000.0f}, // Reference infantry speed PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}          // Not used
    },

    // 右前轮：CAN1 硬件 ID 2，软件编号 2。
    {.motor_id = 2, .offline_alarm_id = 2,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_CHASSIS_DRIVE,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_1,
     .can_rx_id = 0x202,
     .can_tx_id = 0x200,
     .tx_slot = 1,
     .direction = +1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer = {15.0f, 0.0f, 0.0f, 12000.0f, 3000.0f},
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // 左前轮：CAN1 硬件 ID 3，软件编号 3。
    {.motor_id = 3, .offline_alarm_id = 3,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_CHASSIS_DRIVE,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_1,
     .can_rx_id = 0x203,
     .can_tx_id = 0x200,
     .tx_slot = 2,
     .direction = 1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer = {15.0f, 0.0f, 0.0f, 12000.0f, 3000.0f},
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // 左后轮：CAN1 硬件 ID 4，软件编号 4。
    {.motor_id = 4, .offline_alarm_id = 4,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_CHASSIS_DRIVE,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_1,
     .can_rx_id = 0x204,
     .can_tx_id = 0x200,
     .tx_slot = 3,
     .direction = -1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer = {15.0f, 0.0f, 0.0f, 12000.0f, 3000.0f},
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // ========== SHOOTER MOTORS (3x M3508) ==========
    // 拨弹：CAN2 硬件 ID 3，软件编号 9。
    {.motor_id = 9, .offline_alarm_id = 3,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_SHOOTER_FEED,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_2,
     .can_rx_id = 0x203,
     .can_tx_id = 0x200,
     .tx_slot = 2,
     .direction = +1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer =
         {10.0f, 1.0f, 0.0f, 5000.0f,
          5000.0f}, // Reference infantry loader speed PID
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // 左摩擦轮：CAN2 硬件 ID 1，软件编号 6。
    {.motor_id = 6, .offline_alarm_id = 1,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_SHOOTER_FRICTION,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_2,
     .can_rx_id = 0x201,
     .can_tx_id = 0x200,
     .tx_slot = 0,
     .direction = +1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer = {7.0f, 0.8f, 0.08f, 12000.0f, 20000.0f},
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // 右摩擦轮：CAN2 硬件 ID 2，软件编号 7。
    {.motor_id = 7, .offline_alarm_id = 2,
     .vendor = MOTOR_VENDOR_DJI,
     .type = MOTOR_TYPE_M3508,
     .role = MOTOR_ROLE_SHOOTER_FRICTION,
     .control_mode = MOTOR_CONTROL_APPLICATION,
     .can_channel = CAN_CHANNEL_2,
     .can_rx_id = 0x202,
     .can_tx_id = 0x200,
     .tx_slot = 1,
     .direction = +1,
     .limits.m3508 = {.speed_limit = 10000.0f},
     .pid_outer = {7.0f, 0.8f, 0.08f, 12000.0f, 20000.0f},
     .pid_inner = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},

    // ========== GIMBAL MOTORS (2x GM6020) ==========
    // Yaw：CAN1 硬件 ID 5，软件编号 5。
    {
        .motor_id = 5, .offline_alarm_id = 5,
        .vendor = MOTOR_VENDOR_DJI,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_YAW,
        .feedforward = {.velocity_gain = 0.0f, .bias = 0.0f, .output_max = 0.0f},
        .yaw_control = &s_yaw_control,
        .control_mode = MOTOR_CONTROL_APPLICATION,
        .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x209, // GM6020 反馈地址 = 0x204 + 硬件 ID 5。
        .can_tx_id = 0x2FE, // GM6020 ID5 电流指令，匹配已开启的电调电流环。
        .tx_slot = 0,
        .direction = +1,
        .limits.gm6020 =
            {
                .angle_min = 0.0f,
                .angle_max = 8192.0f,
                .gravity_compensation = 0.0f,
                .initial_angle = -1.0f // Auto-initialize from current position (no startup vibration)
            },
        .protocol.dji = {GM6020_COMMAND_CURRENT, 12000}, // 电流原始刻度，5460 约为 1 A。
        // 位置环参数保留；speed_loop_only=true时不执行，恢复后输出仍受模式限速。
        .pid_outer = {28.0f, 0.0f, 1.3f, 2200.0f, 100.0f},
        // 速度误差为RPM、输出为电流原始刻度；保留当前用户PID，反馈失联仍归零。
        .pid_inner = {300.0f, 135.0f, 0.0f, 26000.0f, 4000.0f}
    },

    // Pitch：CAN2 硬件 ID 4，软件编号 8。
    {
        .motor_id = 8, .offline_alarm_id = 4,
        .vendor = MOTOR_VENDOR_DJI,
        .type = MOTOR_TYPE_GM6020,
        .role = MOTOR_ROLE_GIMBAL_PITCH,
        .feedforward = {.velocity_gain = 0.0f, .bias = 0.0f, .output_max = 0.0f},
        .control_mode = MOTOR_CONTROL_APPLICATION,
        .can_channel = CAN_CHANNEL_2,
        .can_rx_id = 0x208, // GM6020 反馈地址 = 0x204 + 硬件 ID 4。
        .can_tx_id = 0x1FF, // GM6020 硬件 ID 1~4 的控制报文。
        .tx_slot = 3,
        .direction = -1, // Pitch direction correction
        .limits.gm6020 =
            {
                .angle_min = 1566.0f, // 实测最高位置；抬头时编码减小。
                .angle_max = 2205.0f, // 实测最低位置。
                .gravity_compensation =
                    0.0f,             // Gravity compensation for pitch
                .initial_angle = 1971.0f, // 用户确认的初始位置，等待真实反馈后再闭环。
                .gravity_zero_angle = 1971.0f, // 用户标定的重力机械零点，单位为编码器刻度。
                .enable_yaw_pitch_compensation = false // 关闭yaw对pitch目标的耦合改写。
            },
        .pid_outer = {22.0f, 0.0f, 1.1f, 950.0f, 100.0f}, // Reference infantry pitch angle PID
        .pid_inner = {0.0f, 0.0f, 0.0f, 26000.0f, 1800.0f}
    }};

/* 用户确认：X形±45°，前后/左右轮中心距均0.54m，轮半径0.07m，M3508 P19。
 * 正驱动向量统一取向前分量为正；电机正反向另由MotorConfig.direction修正。
 * P19手册减速比3591/187；速度是初调上限，首次架空核对四轮方向后再落地。
 */
#define OMNI_HALF_TRACK_M 0.27f
#define OMNI_HALF_WHEELBASE_M 0.27f
#define OMNI_DIAGONAL_UNIT 0.70710678118655f
#define OMNI_RADIUS_M 0.07f
#define M3508_P19_REDUCTION (3591.0f / 187.0f)

static const OmniChassisConfig g_omni_infantry = {
    .configured = 1,
    .max_translation_mps = 0.90f,
    .max_rotation_radps = 1.20f,
    .wheels = {
        {2,  OMNI_HALF_WHEELBASE_M,  OMNI_HALF_TRACK_M,
             OMNI_DIAGONAL_UNIT, -OMNI_DIAGONAL_UNIT, OMNI_RADIUS_M, M3508_P19_REDUCTION},
        {1,  OMNI_HALF_WHEELBASE_M, -OMNI_HALF_TRACK_M,
             OMNI_DIAGONAL_UNIT,  OMNI_DIAGONAL_UNIT, OMNI_RADIUS_M, M3508_P19_REDUCTION},
        {4, -OMNI_HALF_WHEELBASE_M, -OMNI_HALF_TRACK_M,
             OMNI_DIAGONAL_UNIT, -OMNI_DIAGONAL_UNIT, OMNI_RADIUS_M, M3508_P19_REDUCTION},
        {3, -OMNI_HALF_WHEELBASE_M,  OMNI_HALF_TRACK_M,
             OMNI_DIAGONAL_UNIT,  OMNI_DIAGONAL_UNIT, OMNI_RADIUS_M, M3508_P19_REDUCTION},
    }
};

// Robot configuration structure
const RobotConfig_t g_robot_config_infantry_standard = {
    .name = "Infantry Standard",
    .chassis_type = CHASSIS_TYPE_OMNI,
    .omni = &g_omni_infantry,
    .chassis_follow = &s_chassis_follow,
    .chassis_motor_count = 4,
    .gimbal_motor_count = 2,
    .shooter_motor_count = 3,
    .motor_configs = g_motor_configs_infantry_standard,
    .total_motor_count = 9,
    .enable_imu_calibration = 1 // Infantry needs IMU calibration
};
