/*
 * 定义车型、电机、控制模式和机器人配置的公共数据结构。
 * 这里只描述数据，不包含硬件读写或控制流程。
 */
#ifndef CONFIG_TYPES_H
#define CONFIG_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/** Chassis kinematics family selected by a robot configuration. */
typedef enum {
    CHASSIS_TYPE_MECANUM = 0,
    CHASSIS_TYPE_SWERVE,
    CHASSIS_TYPE_OMNI
} ChassisType_e;

#define OMNI_WHEEL_COUNT 4U

/* Wheel position in metres: +x forward, +y left, +yaw counterclockwise.
 * drive_x/y is the unit vector of positive wheel travel, not the roller axis.
 * MotorConfig.direction converts that wheel direction to signed rotor RPM.
 */
typedef struct {
    uint8_t motor_id;
    float x_m;
    float y_m;
    float drive_x;
    float drive_y;
    float radius_m;
    float reduction_ratio; /* Rotor revolutions per wheel revolution, positive. */
} OmniWheelConfig;

typedef struct {
    uint8_t configured; /* Geometry supplied; motor signs still need a wheel test. */
    float max_translation_mps;
    float max_rotation_radps;
    OmniWheelConfig wheels[OMNI_WHEEL_COUNT];
} OmniChassisConfig;

/* 云台相对底盘的安装标定；仅用于中档平移坐标换算，不是云台位置目标。 */
typedef struct {
    uint16_t yaw_forward_ticks; /* GM6020单圈0..8191，云台指向底盘正前方。 */
    int8_t yaw_ccw_sign;        /* +1：编码增大为俯视左转；-1：编码减小为左转。 */
} ChassisFollowConfig;

/**
 * @brief CAN channel enumeration
 */
typedef enum {
    CAN_CHANNEL_1 = 0,
    CAN_CHANNEL_2 = 1,
    CAN_CHANNEL_COUNT
} CAN_Channel_t;

/**
 * @brief Motor type enumeration
 */
typedef enum {
    MOTOR_TYPE_M3508,    // M3508 brushless motor with C620 ESC
    MOTOR_TYPE_GM6020,   // GM6020 brushless gimbal motor
    MOTOR_TYPE_M2006,    // M2006 motor (for future use)
    MOTOR_TYPE_VENDOR_DEFINED = 255
} MotorType_e;

/** Motor protocol/vendor. Zero intentionally preserves all existing configs. */
typedef enum {
    MOTOR_VENDOR_DJI = 0,
    MOTOR_VENDOR_DM,
    MOTOR_VENDOR_BENMO,
    MOTOR_VENDOR_LK
} MotorVendor_e;

/* GM6020 driver mode must match its Assistant current-loop switch. */
typedef enum {
    GM6020_COMMAND_VOLTAGE = 0, /* Preserve existing zero-initialized configs. */
    GM6020_COMMAND_CURRENT
} GM6020CommandMode_e;

/* 本末 BM1505B 的两种运行指令格式。 */
typedef enum {
    BENMO_CONTROL_RAW_CURRENT = 0x01,
    BENMO_CONTROL_SPEED_RPM_X10 = 0x02
} BenmoControlMode_e;

/**
 * Per-motor control switch. APPLICATION is the compatibility mode used by the
 * existing controllers. Other modes are executed by MotorService when the
 * application supplies a configured setpoint.
 */
typedef enum {
    MOTOR_CONTROL_APPLICATION = 0,
    MOTOR_CONTROL_DISABLED,
    MOTOR_CONTROL_OPEN_LOOP_CURRENT,
    MOTOR_CONTROL_SPEED,
    MOTOR_CONTROL_POSITION,
    MOTOR_CONTROL_POSITION_SPEED_CASCADE
} MotorControlMode_e;

/**
 * @brief Motor role enumeration
 * Defines the functional role of each motor in the robot system
 */
typedef enum {
    MOTOR_ROLE_CHASSIS_DRIVE,      // Chassis drive motor (mecanum or swerve)
    
    MOTOR_ROLE_CHASSIS_STEER,      // Swerve drive steering motor
    MOTOR_ROLE_GIMBAL_YAW,         // Gimbal yaw axis
    MOTOR_ROLE_GIMBAL_PITCH,       // Gimbal pitch axis
    MOTOR_ROLE_SHOOTER_FEED,       // Shooter feed/turntable motor
    MOTOR_ROLE_SHOOTER_FRICTION    // Shooter friction wheel motor
} MotorRole_e;

/**
 * @brief PID parameters structure
 */
typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float output_max;      // Maximum output value
    float integral_max;    // Maximum integral value (anti-windup)
} PIDParams_t;

/* yaw 应用参数；速度调试开关只绕过位置环，不改变电调模式或 PID 参数。 */
typedef struct {
    float manual_rate_deg_s;       /* 满杆目标角速度；速度调试时除以6得到RPM。 */
    float target_lead_deg;         /* 推杆时允许的新目标领先量；回中不改目标。 */
    float manual_speed_rpm;        /* 普通推杆和回中共用的速度上限。 */
    float vision_speed_rpm;        /* 明确进入视觉模式时的速度上限。 */
    float spin_speed_rpm;          /* 明确进入spin模式时的速度上限。 */
    bool speed_loop_only;          /* true：暂时旁路位置环，只调电机RPM速度环。 */
} YawControlConfig;

/* 仅由云台应用在速度PID之后叠加，不改变PID算法或电调模式。 */
typedef struct {
    float velocity_gain; /* 原始命令刻度/RPM，乘速度内环目标；不再乘motor direction。 */
    float bias;          /* 有符号原始命令刻度，零速时也存在；不是重力幅值。 */
    float output_max;    /* 前馈合计绝对限幅；0关闭，启用时须有限且>0。 */
} GimbalFeedforwardConfig;

/**
 * @brief Motor configuration structure
 * Contains all parameters needed to configure a single motor
 */
typedef struct {
    // Basic identification
    uint8_t motor_id;              // Logical motor ID (0-15)
    uint8_t offline_alarm_id;      /* 红灯次数1..16，填写已确认硬件ID；0表示尚未配置报码。 */
    MotorVendor_e vendor;          // CAN protocol/vendor adapter
    MotorType_e type;              // Motor type
    MotorRole_e role;              // Functional role
    MotorControlMode_e control_mode; // Runtime command interpretation

    // CAN communication parameters
    CAN_Channel_t can_channel;     // CAN bus channel (CAN_CHANNEL_1 or CAN_CHANNEL_2)
    uint16_t can_rx_id;            // CAN ID for receiving feedback
    uint16_t can_tx_id;            // CAN command group selected by the motor protocol
    uint8_t tx_slot;               // Slot position in TX frame (0-3)

    // Motor-specific parameters
    int8_t direction;              // Motor direction: +1 or -1

    // Type-specific limits (union to save memory)
    union {
        // GM6020-specific parameters
        struct {
            float angle_min;            // Minimum angle limit (encoder units)
            float angle_max;            // Maximum angle limit (encoder units)
            float gravity_compensation; // Gravity compensation torque (for pitch axis)
            float initial_angle;        // 编码器刻度；pitch启动/重新对齐目标，负数则锁存当前位置。
            float gravity_zero_angle;   // 云台应用重力正弦项的零点，绝对编码刻度；默认0保留原生编码基准。
            bool enable_yaw_pitch_compensation; // 默认false；true允许旧yaw耦合算法改写pitch目标，与重力项独立。
        } gm6020;

        // M3508-specific parameters
        struct {
            float speed_limit;          // Maximum speed (RPM)
        } m3508;
    } limits;

    /*
     * 厂商协议参数。新增型号时只改所选车型配置，不把型号常数写进驱动。
     * DJI 全零配置沿用 GM6020 电压控制；电流模式必须显式指定。
     */
    union {
        struct {
            GM6020CommandMode_e gm6020_mode;
            int16_t command_limit; /* Native counts; 0 uses the protocol maximum. */
        } dji;
        struct {
            float position_min;
            float position_max;
            float velocity_min;
            float velocity_max;
            float torque_min;
            float torque_max;
        } dm;
        struct {
            int16_t command_limit;
            uint32_t encoder_counts_per_rev;
        } lk;
        struct {
            uint8_t motor_address;
            BenmoControlMode_e command_mode;
            uint8_t feedback_mode;
            int16_t command_limit;
        } benmo;
    } protocol;

    /* 云台通常为位置/速度环。普通底盘M3508驱动轮在APPLICATION模式下为速度/电流环，
     * 电流环output_max=0表示不启用，保留原速度单环；速度用RPM，电流用原始刻度。
     */
    PIDParams_t pid_outer;
    PIDParams_t pid_inner;
    const YawControlConfig *yaw_control; /* 仅yaw应用读取；缺配置时禁止yaw出力。 */
    GimbalFeedforwardConfig feedforward; /* yaw/pitch前馈；省略时全零关闭。 */
} MotorConfig_t;

/**
 * @brief Robot configuration structure
 * Top-level configuration for a complete robot
 */
typedef struct {
    const char *name;                    // Robot configuration name
    ChassisType_e chassis_type;          // Selected chassis strategy
    uint8_t chassis_motor_count;         // Number of chassis motors
    uint8_t gimbal_motor_count;          // Number of gimbal motors
    uint8_t shooter_motor_count;         // Number of shooter motors
    const MotorConfig_t *motor_configs;  // Pointer to motor configuration array
    uint8_t total_motor_count;           // Total number of motors
    uint8_t enable_imu_calibration;      // Enable IMU calibration at startup (1 = enabled, 0 = disabled)
    const OmniChassisConfig *omni;       // NULL for other chassis families.
    const ChassisFollowConfig *chassis_follow; /* NULL保留该车型原有模式换算。 */
} RobotConfig_t;

#endif // CONFIG_TYPES_H
