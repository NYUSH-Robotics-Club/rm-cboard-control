/*
 * 声明发射机构控制器。
 * 目标速度使用转子 RPM，反馈时间使用毫秒，输出为电流命令；不处理电机协议。
 */
#ifndef SHOOTER_CONTROLLER_H
#define SHOOTER_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "motor_messages.h"
#include "sensor_messages.h"

// Shooter system motor count
#define SHOOTER_MOTOR_COUNT 4

// Shooter system parameters
#define MOTOR5_CONST_SPEED 1000      // Turntable speed
#define SHOOTER_CONST_SPEED 7500     // Shooter wheel speed
#define SHOOTER_RAMP_STEP 500.0f      // Acceleration step

/* 拨弹堵转初调值，尚未实车标定；电流用反馈原始刻度，不是安培。
 * 当前拨弹1000 RPM、Kp=1，静止比例输出约1000，故先取800作检测门槛。
 * 仅按持续高电流判断，不要求低转速；一次拨弹最多反转一次，重堵等待松开。
 */
/* 应用共享可调变量，定义在shooter_controller.c；范围1..32767原始刻度。
 * 仅在取消拨弹后由主循环修改；不在中断中写入。非正值使拨盘锁定停机，
 * 修正后取消拨弹才能解锁。修改初始值须重新构建/烧录，运行时赋值不持久保存。
 */
extern int16_t g_feed_stall_current_threshold;
#define FEED_STALL_DURATION_MS 500U
#define FEED_REVERSE_SPEED_RPM 500.0f /* 正值幅度，实际目标与正常拨弹方向相反。 */
#define FEED_REVERSE_DURATION_MS 200U

typedef enum {
    FEED_RECOVERY_MONITORING = 0,
    FEED_RECOVERY_REVERSING,
    FEED_RECOVERY_BLOCKED /* 再次持续高电流，拨盘零电流直到取消拨弹。 */
} FeedRecoveryState;

/* 由各控制器独占；所有时间为可回绕的uint32毫秒，计时有效位独立于零时刻。 */
typedef struct {
    FeedRecoveryState state;
    bool reverse_attempted; /* 停机/失联取消动作但不补发次数；取消拨弹后清除。 */
    bool high_current_active;
    uint32_t high_current_since_ms;
    uint32_t last_check_ms;
    uint32_t reverse_since_ms;
} FeedRecovery;

// Shooter controller structure
typedef struct {
    // Turntable target speed
    float turntable_target;
    float ramped_turntable;
    
    // Shooter wheel target speeds
    float shooter1_target;
    float shooter2_target;
    float ramped_shooter1;
    float ramped_shooter2;
    float ramped_yaw;
    
    // Running state
    bool enabled;
    bool feedback_seen[3]; /* 拨盘、左摩擦轮、右摩擦轮；收到过反馈后置true。 */
    bool feedback_fault;   /* 缺配置/首帧或任一反馈超过100ms，整个发射机构禁止出力。 */
    FeedRecovery feed_recovery;
    
    // PID controllers (turntable and shooter wheels)
    PID_Controller turntable_pid;
    PID_Controller shooter1_pid;
    PID_Controller shooter2_pid;
    PID_Controller yaw_pid;
    
    // Motor feedbacks (turntable and shooter wheels)
    Motor_Feedback turntable_feedback;
    Motor_Feedback shooter1_feedback;
    Motor_Feedback shooter2_feedback;
    
    // Output currents
    int16_t output_currents[SHOOTER_MOTOR_COUNT];
} ShooterController;

/**
 * @brief Initialize shooter controller
 * @param controller Shooter controller pointer
 */
void ShooterController_Init(ShooterController *controller);

/**
 * @brief Update shooter control logic
 * @param controller Shooter controller pointer
 * @param sensor_data Sensor data pointer
 */
void ShooterController_Update(ShooterController *controller, SensorData *sensor_data);

/**
 * @brief Compute shooter system motor currents
 * @param controller Shooter controller pointer
 * @param current_tick Current timestamp
 */
void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick);

/**
 * @brief Set turntable speed
 * @param controller Shooter controller pointer
 * @param speed Target speed
 */
void ShooterController_SetTurntableSpeed(ShooterController *controller, float speed);

/**
 * @brief Set shooter wheel speeds
 * @param controller Shooter controller pointer
 * @param shooter1_speed Shooter wheel 1 speed
 * @param shooter2_speed Shooter wheel 2 speed
 */
void ShooterController_SetShooterSpeeds(ShooterController *controller, float shooter1_speed, float shooter2_speed);

/**
 * @brief 三台电机零命令，清目标/斜坡、堵转计时和全部PID历史；不清有效反馈。
 * 已用反转次数和重堵锁定保留，取消拨弹才解锁，避免失联恢复后反复尝试。
 * @param controller Shooter controller pointer
 */
void ShooterController_Stop(ShooterController *controller);

/**
 * @brief Get output currents
 * @param controller Shooter controller pointer
 * @return Output currents array pointer
 */
const int16_t* ShooterController_GetOutputCurrents(const ShooterController *controller);

/**
 * @brief Check if shooter system is running
 * @param controller Shooter controller pointer
 * @return true if shooter is running
 */
bool ShooterController_IsRunning(const ShooterController *controller);

/**
 * @brief Update motor feedback
 * @param controller Shooter controller pointer
 * @param motor_id 车型配置中的软件电机ID
 * @param angle Angle
 * @param speed Speed
 * @param current Current
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

/* 主循环初始化订阅；只读诊断指针由应用持有，外部不可修改。 */
void ShooterApp_Init(void);
const ShooterController *ShooterApp_GetController(void);

#endif // SHOOTER_CONTROLLER_H
