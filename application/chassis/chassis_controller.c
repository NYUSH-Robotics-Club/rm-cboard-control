/*
 * 接收底盘命令和电机反馈，并计算驱动电流。
 * 车型换算由底盘策略完成，CAN 发送由电机服务完成。
 */
#include "chassis_controller.h"
#include "motor_service.h"
#include <string.h>
#include <stdint.h>
#include "message_center.h"
#include "motor_messages.h"
#include "sensor_messages.h"
#include "logger.h"
#include "control_messages.h"
#include "chassis_strategy.h"
#include "bsp_time.h"
#include "robot_config.h"
#include <math.h>

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static ChassisController s_ctrl;

// Chassis motor configuration (dynamically assigned during init)
static uint8_t s_chassis_motor_ids[CHASSIS_MOTOR_COUNT];
static int8_t s_motor_directions[CHASSIS_MOTOR_COUNT];
static uint8_t s_chassis_motor_count = 0;
static const ChassisStrategy *s_chassis_strategy = NULL;
static const OmniChassisConfig *s_omni = NULL;
static float s_max_drive_rpm;
static bool s_config_valid;

static void ResetPidIntegrals(ChassisController *controller)
{
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        PID_Reset(&controller->speed_pids[i]);
        PID_Reset(&controller->current_pids[i]);
    }
}

/* 调用者已检查使能和反馈时效；速度环产生电流目标，可选内环使用同一帧电流反馈。
 * 零配置仍直接使用速度环输出，不增加新的电机协议或改变其他车型控制方式。
 */
static int16_t ComputeSingleMotorCurrent(ChassisController *controller, uint8_t index)
{
    Motor_Feedback *feedback = &controller->motor_feedbacks[index];
    float command = PID_Calculate(&controller->speed_pids[index],
                                  controller->target_speeds[index], feedback->speed);
    if (controller->current_loop_enabled[index]) {
        command = PID_Calculate(&controller->current_pids[index], command, feedback->current);
    }
    return (int16_t)command;
}

void ChassisController_Init(ChassisController *controller)
{
    if (controller == NULL) return;
    
    memset(controller, 0, sizeof(ChassisController));
    s_chassis_strategy = ChassisStrategy_GetActive();
    const RobotConfig_t *robot = RobotConfig_Get();
    const bool omni = robot && robot->chassis_type == CHASSIS_TYPE_OMNI;
    s_omni = omni ? robot->omni : NULL;
    s_max_drive_rpm = (float)CHASSIS_DEMO_TARGET_SPEED;
    s_config_valid = true;

    // Find chassis motors by role (module layer handles config)
    s_chassis_motor_count = MotorService_FindByRole(MOTOR_ROLE_CHASSIS_DRIVE,
                                                    s_chassis_motor_ids,
                                                    CHASSIS_MOTOR_COUNT);
    if (omni) {
        s_config_valid = s_omni && s_omni->configured &&
                         s_chassis_motor_count == OMNI_WHEEL_COUNT;
        if (s_omni) {
            /* Geometry owns wheel order; CAN slot and array index are unrelated. */
            uint8_t discovered[CHASSIS_MOTOR_COUNT];
            memcpy(discovered, s_chassis_motor_ids, sizeof(discovered));
            bool mapping_valid = true;
            for (uint8_t i = 0; i < s_chassis_motor_count; ++i) {
                uint8_t id = s_omni->wheels[i].motor_id;
                bool found = false;
                for (uint8_t j = 0; j < s_chassis_motor_count; ++j)
                    if (discovered[j] == id) found = true;
                for (uint8_t j = 0; j < i; ++j)
                    if (s_omni->wheels[j].motor_id == id) found = false;
                if (!found) mapping_valid = false;
            }
            if (mapping_valid) {
                for (uint8_t i = 0; i < s_chassis_motor_count; ++i)
                    s_chassis_motor_ids[i] = s_omni->wheels[i].motor_id;
            } else {
                /* Never send a stop command to a different role on a bad map. */
                s_config_valid = false;
            }
        }
    }

    // Get motor directions and initialize PIDs from configuration service
    for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
        const MotorConfig_t *config =
            MotorService_GetConfig(s_chassis_motor_ids[i]);
        if (config) {
            s_motor_directions[i] = config->direction;
            if (omni) {
                if (config->type != MOTOR_TYPE_M3508 ||
                    config->role != MOTOR_ROLE_CHASSIS_DRIVE ||
                    (config->direction != 1 && config->direction != -1) ||
                    !isfinite(config->limits.m3508.speed_limit) ||
                    config->limits.m3508.speed_limit <= 0.0f) {
                    s_config_valid = false;
                } else {
                    s_max_drive_rpm = fminf(s_max_drive_rpm, config->limits.m3508.speed_limit);
                }
            }

            // Initialize PID from config
            PID_Init(&controller->speed_pids[i],
                     config->pid_outer.kp,
                     config->pid_outer.ki,
                     config->pid_outer.kd,
                     config->pid_outer.output_max,
                     config->pid_outer.integral_max);

            /* 应用控制使用反馈事件中的原始电流；非法内环配置使底盘保持停机。
             * 此处只限制int16_t接口容量，厂商协议限幅仍由电机服务/适配器执行。
             */
            const PIDParams_t *current_pid = &config->pid_inner;
            if (!isfinite(current_pid->output_max) || current_pid->output_max < 0.0f) {
                s_config_valid = false;
            } else if (current_pid->output_max > 0.0f) {
                if (config->control_mode != MOTOR_CONTROL_APPLICATION ||
                    !isfinite(current_pid->kp) || !isfinite(current_pid->ki) ||
                    !isfinite(current_pid->kd) || !isfinite(current_pid->integral_max) ||
                    current_pid->integral_max < 0.0f || current_pid->output_max > INT16_MAX) {
                    s_config_valid = false;
                } else {
                    controller->current_loop_enabled[i] = true;
                    PID_Init(&controller->current_pids[i], current_pid->kp, current_pid->ki,
                             current_pid->kd, current_pid->output_max, current_pid->integral_max);
                }
            }

            controller->target_speeds[i] = 0.0f;
        } else {
            s_config_valid = false;
        }
    }
}

void ChassisController_Update(ChassisController *controller, SensorData* sensor_data)
{
    if (controller == NULL) return;
    (void)sensor_data;

    ChassisKinematicsInput input = {
        .vx = s_last_cmd.vx,
        .vy = s_last_cmd.vy,
        .wz = s_last_cmd.wz,
        .max_drive_speed = s_max_drive_rpm,
        .omni = s_omni
    };
    ChassisKinematicsOutput output = {0};
    RobotStatus status = ROBOT_STATUS_NOT_READY;
    if (s_last_cmd.enabled && s_config_valid && s_chassis_strategy && s_chassis_strategy->compute) {
        status = s_chassis_strategy->compute(&input, &output);
    }
    memset(controller->target_speeds, 0, sizeof(controller->target_speeds));
    if (status == ROBOT_STATUS_OK && output.drive_count == s_chassis_motor_count) {
        uint8_t count = output.drive_count < s_chassis_motor_count
                            ? output.drive_count
                            : s_chassis_motor_count;
        for (uint8_t i = 0; i < count; ++i) {
            controller->target_speeds[i] =
                s_motor_directions[i] * output.drive_speed[i];
        }
    }

    controller->running = s_last_cmd.enabled && status == ROBOT_STATUS_OK &&
                          output.drive_count == s_chassis_motor_count;
}

void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;
    
    // Chassis status logging (5Hz rate limited in main.c)
    LOG_INFO(LOG_TAG_CHA, "enabled=%d", (int)s_last_cmd.enabled);
    for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
        uint32_t dt = BspTime_NowMs() - controller->motor_feedbacks[i].last_update_time;
        LOG_DEBUG(LOG_TAG_CHA, "motor[%d] id=%d tgt=%.1f fb=%.1f dt=%lu out=%d",
                  i,
                  s_chassis_motor_ids[i],
                  controller->target_speeds[i],
                  controller->motor_feedbacks[i].speed,
                  (unsigned long)dt,
                  controller->output_currents[i]);
    }


    /* Losing an omni wheel invalidates the requested body motion. Wait for all
     * four real feedback events, including events received at tick zero.
     */
    bool allow_output = controller->running && s_config_valid;
    if (s_chassis_strategy && s_chassis_strategy->type == CHASSIS_TYPE_OMNI) {
        for (uint8_t i = 0; i < s_chassis_motor_count; ++i) {
            if (!controller->feedback_seen[i] ||
                current_tick - controller->motor_feedbacks[i].last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS)
                allow_output = false;
        }
    }

    // Compute current for each chassis motor
    for (int i = 0; i < s_chassis_motor_count; i++) {
        if (!allow_output || !controller->feedback_seen[i] ||
            current_tick - controller->motor_feedbacks[i].last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) {
            PID_Reset(&controller->speed_pids[i]);
            PID_Reset(&controller->current_pids[i]);
            controller->output_currents[i] = 0;
            (void)MotorService_CommandCurrent(s_chassis_motor_ids[i], 0);
            continue;
        }
        int16_t motor_current = ComputeSingleMotorCurrent(controller, (uint8_t)i);
        controller->output_currents[i] = motor_current;

        // Send motor current (buffered, will be flushed in main loop)
        (void)MotorService_CommandConfigured(s_chassis_motor_ids[i],
                                             controller->target_speeds[i],
                                             motor_current);
    }
}

void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
{
    if (controller == NULL || speeds == NULL) return;
    controller->running = true;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = speeds[i]; }
}

void ChassisController_Stop(ChassisController *controller)
{
    if (controller == NULL) return;
    controller->running = false;
    ResetPidIntegrals(controller);
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = 0.0f; }
}

const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
{
    if (controller == NULL) return NULL;
    return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller)
{
    if (controller == NULL) return false;
    for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { if (controller->target_speeds[i] != 0) { return true; } }
    return false;
}

void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{

    if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
    controller->motor_feedbacks[motor_id].angle = angle;
    controller->motor_feedbacks[motor_id].speed = speed;
    controller->motor_feedbacks[motor_id].current = current;
    controller->motor_feedbacks[motor_id].temp = temp;
    controller->motor_feedbacks[motor_id].last_update_time = current_tick;
    controller->feedback_seen[motor_id] = true;
}

// Subscription callbacks
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ChassisCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
        // Update controller and compute currents when command arrives
        ChassisController_Update(&s_ctrl, &s_last_sensor);
        ChassisController_ComputeCurrents(&s_ctrl, BspTime_NowMs());
    }
}

static void on_imu_update(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(SensorData)) {
        memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
    }
}

static void on_motor_feedback(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;

        // Check if this motor is a chassis motor
        for (uint8_t i = 0; i < s_chassis_motor_count; i++) {
            if (m->id == s_chassis_motor_ids[i]) {
                // Found matching chassis motor, update feedback at index i
                ChassisController_UpdateMotorFeedback(&s_ctrl, i, m->angle, m->speed, m->current, m->temp, m->tick_ms);
                break;
            }
        }
    }
}

void ChassisApp_Init(void) {
    
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    // Initialize chassis controller (module layer handles config)
    ChassisController_Init(&s_ctrl);

    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

const ChassisController* ChassisApp_GetController(void) {
    return &s_ctrl;
}

uint8_t ChassisApp_GetMotorIds(uint8_t ids[CHASSIS_MOTOR_COUNT]) {
    if (!ids || !s_config_valid) return 0U;
    memcpy(ids, s_chassis_motor_ids, s_chassis_motor_count);
    return s_chassis_motor_count;
}
