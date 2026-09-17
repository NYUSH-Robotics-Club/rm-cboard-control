/*
 * 声明普通底盘控制器的数据和操作。
 * 速度使用 RPM，角度使用电机编码器刻度，时间使用毫秒。
 */
#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "motor_messages.h"
#include "sensor_messages.h"

// Chassis motor count
#define CHASSIS_MOTOR_COUNT 4

#define CHASSIS_STEER_COUNT 2

// Chassis control parameters
#define CHASSIS_DEMO_TARGET_SPEED 8050  // Increased by 15% from 7000


// Chassis controller structure
typedef struct {
    // Drive motor target speeds
    float target_speeds[CHASSIS_MOTOR_COUNT];

    // Running state
    bool running;

    // Drive PID controllers
    PID_Controller speed_pids[CHASSIS_MOTOR_COUNT];
    /* 控制器拥有每轮的可选电流PID；输入/输出均为DJI原始电流刻度。
     * 配置pid_inner.output_max>0启用；停机或反馈过期时与速度环一起清零。
     */
    PID_Controller current_pids[CHASSIS_MOTOR_COUNT];
    bool current_loop_enabled[CHASSIS_MOTOR_COUNT];

    // Drive motor feedbacks
    Motor_Feedback motor_feedbacks[CHASSIS_MOTOR_COUNT];
    bool feedback_seen[CHASSIS_MOTOR_COUNT]; /* Timestamp zero can be a valid event. */

    // Drive output currents
    int16_t output_currents[CHASSIS_MOTOR_COUNT];

    // -----------------------------
    // Swerve steering (optional)
    // -----------------------------
    float steer_target_angles[CHASSIS_STEER_COUNT];          // encoder ticks
    PID_Controller steer_pids[CHASSIS_STEER_COUNT];          // angle PID
    Motor_Feedback steer_feedbacks[CHASSIS_STEER_COUNT];     // steer feedback
    int16_t steer_output_currents[CHASSIS_STEER_COUNT];      // steer currents
} ChassisController;


/**
 * @brief Initialize chassis controller
 * @param controller Chassis controller pointer
 */
void ChassisController_Init(ChassisController *controller);

/**
 * @brief Update chassis control logic
 * @param controller Chassis controller pointer
 * @param sensor_data Sensor data pointer
 */
void ChassisController_Update(ChassisController *controller, SensorData* sensor_data);

/**
 * @brief Compute chassis motor currents
 * @param controller Chassis controller pointer
 * @param current_tick Current timestamp
 */
void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick);

/**
 * @brief Set chassis target speeds
 * @param controller Chassis controller pointer
 * @param speeds Target speeds array
 */
void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT]);

/**
 * @brief Stop chassis
 * @param controller Chassis controller pointer
 */
void ChassisController_Stop(ChassisController *controller);

/**
 * @brief Get output currents
 * @param controller Chassis controller pointer
 * @return Output currents array pointer
 */
const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller);

/**
 * @brief Check if any motor is running
 * @param controller Chassis controller pointer
 * @return true if any motor is running
 */
bool ChassisController_IsRunning(const ChassisController *controller);

/**
 * @brief Update motor feedback (to be called from CAN receive path)
 * @param controller Chassis controller pointer
 * @param motor_id Motor ID in range 0..3
 * @param angle Encoder angle
 * @param speed Speed (RPM)
 * @param current 电调反馈的有符号原始电流刻度，不是安培
 * @param temp Temperature
 * @param current_tick Current timestamp
 */
void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick);

/**
 * @brief Initialize chassis application (subscriptions, etc.)
 */
void ChassisApp_Init(void);

/* 主循环只读诊断；指针归应用所有，速度为RPM。
 * ids容量须为4，编号与速度数组同序；空指针/未配置返回0，否则返回写入数。 */
const ChassisController *ChassisApp_GetController(void);
uint8_t ChassisApp_GetMotorIds(uint8_t ids[CHASSIS_MOTOR_COUNT]);

/**
 * @brief Wait for swerve steer motors to align to initial position
 * @note Only applicable for sentry_swerve configuration
 */
void Sentry_WaitForSteerAlignment(void);

#endif // CHASSIS_CONTROLLER_H
