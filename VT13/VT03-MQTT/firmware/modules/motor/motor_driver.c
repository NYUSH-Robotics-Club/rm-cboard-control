/*
 * 保存现有 DJI 电机状态、计算控制电流并缓存发送帧。
 * 新应用应通过 MotorService 使用它，避免直接依赖内部状态。
 */
#include "motor_driver.h"
#include "dji_motor_protocol.h"
#include "message_center.h"
#include "can_comm.h"
#include "can_manager.h"
#include "robot_config.h"
#include "printing.h"
#include "logger.h"
#include "bsp_time.h"
#include <math.h>
#include <string.h>

// External CAN managers
extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

// Global motor contexts (one per motor)
static MotorContext_t g_motor_contexts[MOTOR_DRIVER_MAX_MOTORS];

// Robot configuration reference (set during init)
static const RobotConfig_t *g_robot_config = NULL;

// Subscription flags
static bool g_module_initialized = false;

// Forward declarations for callbacks
static void on_gm6020_feedback(const MsgEvent *ev, void *user);
static void on_motor_feedback(const MsgEvent *ev, void *user);

/**
 * @brief Initialize motor driver module
 */
void MotorDriver_ModuleInit(void)
{
    if (g_module_initialized) {
        return;  // Already initialized
    }

    // Clear all motor contexts
    memset(g_motor_contexts, 0, sizeof(g_motor_contexts));

    // Get robot configuration
    g_robot_config = RobotConfig_Get();

    // Initialize all motors from configuration
    if (g_robot_config != NULL) {
        LOG_INFO(LOG_TAG_MOT, "Initializing %d motors from config '%s'",
                 g_robot_config->total_motor_count, g_robot_config->name);

        for (uint8_t i = 0; i < g_robot_config->total_motor_count; i++) {
            const MotorConfig_t *motor_cfg = &g_robot_config->motor_configs[i];
            // This legacy state store only owns DJI feedback and PID state.
            if (motor_cfg->vendor != MOTOR_VENDOR_DJI) {
                continue;
            }
            MotorDriver_Init(motor_cfg->motor_id, motor_cfg);

            LOG_DEBUG(LOG_TAG_MOT, "Motor %d: type=%d role=%d CAN=%d init=%d",
                      motor_cfg->motor_id,
                      motor_cfg->type,
                      motor_cfg->role,
                      motor_cfg->can_channel,
                      g_motor_contexts[motor_cfg->motor_id].initialized);
        }
        LOG_INFO(LOG_TAG_MOT, "All motors initialized");
    } else {
        LOG_ERROR(LOG_TAG_MOT, "ERROR: g_robot_config is NULL!");
    }

    // Subscribe to CAN feedback topics
    (void)MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, on_gm6020_feedback, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);

    g_module_initialized = true;
}

/**
 * @brief Initialize a motor from configuration
 */
bool MotorDriver_Init(uint8_t motor_id, const MotorConfig_t *config)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS || config == NULL) {
        return false;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];

    // Clear context
    memset(ctx, 0, sizeof(MotorContext_t));

    // Store motor identification
    ctx->motor_id = motor_id;
    ctx->type = config->type;
    ctx->role = config->role;
    ctx->config = config;

    // Initialize target angle from configuration
    if (config->type == MOTOR_TYPE_GM6020) {
        ctx->angle_target = config->limits.gm6020.initial_angle;
        // If initial angle < 0, mark as not initialized (will auto-init on first feedback)
        ctx->angle_initialized = (config->limits.gm6020.initial_angle >= 0.0f);
    } else {
        ctx->angle_target = 0.0f;
        ctx->angle_initialized = false;
    }

    // Initialize PID controllers from configuration
    PID_Init(&ctx->pid_outer,
             config->pid_outer.kp,
             config->pid_outer.ki,
             config->pid_outer.kd,
             config->pid_outer.output_max,
             config->pid_outer.integral_max);

    // Inner loop PID (if configured)
    if (config->pid_inner.kp != 0.0f || config->pid_inner.output_max != 0.0f) {
        PID_Init(&ctx->pid_inner,
                 config->pid_inner.kp,
                 config->pid_inner.ki,
                 config->pid_inner.kd,
                 config->pid_inner.output_max,
                 config->pid_inner.integral_max);
    }

    ctx->initialized = true;
    return true;
}

/**
 * @brief Update motor feedback
 */
void MotorDriver_UpdateFeedback(uint8_t motor_id,
                                uint16_t angle_raw,
                                int16_t speed_rpm,
                                int16_t current,
                                uint32_t timestamp)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (!ctx->initialized) {
        return;  // Motor not initialized yet
    }

    // Update feedback data
    ctx->angle_raw = angle_raw;
    ctx->speed_rpm = speed_rpm;
    ctx->feedback_current = current;
    ctx->last_feedback_time = timestamp;

    // Compute angle in radians (for GM6020 gimbal control)
    if (ctx->type == MOTOR_TYPE_GM6020) {
        // 一圈8192刻度，不把机械上限当作编码器分辨率。
        const float max_encoder = 8192.0f;
        ctx->target_angle_rad = (float)angle_raw / max_encoder * 2.0f * M_PI;
    }

    // Auto-initialize angle target on first feedback
    if (!ctx->angle_initialized) {
        ctx->angle_target = (float)angle_raw;
        ctx->angle_initialized = true;
    }
}

/**
 * @brief Compute motor current command
 */
int16_t MotorDriver_ComputeCurrent(uint8_t motor_id,
                                   float target,
                                   bool use_angle_control)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return 0;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (!ctx->initialized || !ctx->angle_initialized) {
        return 0;  // Not ready
    }

    int16_t output_current = 0;

    if (use_angle_control) {
        // Outer loop: Angle control
        ctx->angle_target = target;

        // Apply angle limits (for GM6020 gimbal motors)
        if (ctx->type == MOTOR_TYPE_GM6020) {
            if (ctx->angle_target < ctx->config->limits.gm6020.angle_min) {
                ctx->angle_target = ctx->config->limits.gm6020.angle_min;
            }
            if (ctx->angle_target > ctx->config->limits.gm6020.angle_max) {
                ctx->angle_target = ctx->config->limits.gm6020.angle_max;
            }
        }

        // PID: angle error -> speed target
        float speed_target = PID_Calculate(&ctx->pid_outer, ctx->angle_target, (float)ctx->angle_raw);

        // Inner loop: Speed control (if configured)
        if (ctx->config->pid_inner.kp != 0.0f) {
            output_current = (int16_t)PID_Calculate(&ctx->pid_inner, speed_target, (float)ctx->speed_rpm);
        } else {
            // Direct output from angle PID
            output_current = (int16_t)speed_target;
        }

        // Add gravity compensation (for pitch motors)
        if (ctx->type == MOTOR_TYPE_GM6020 && ctx->role == MOTOR_ROLE_GIMBAL_PITCH) {
            float gravity_comp = ctx->config->limits.gm6020.gravity_compensation;
            float direction = (float)ctx->config->direction;
            output_current += (int16_t)(gravity_comp * direction);
        }

    } else {
        // Direct speed control
        output_current = (int16_t)PID_Calculate(&ctx->pid_outer, target, (float)ctx->speed_rpm);
    }

    // Apply direction correction
    output_current *= ctx->config->direction;

    // Clamp current based on motor type
    int16_t max_current = DjiMotor_CommandLimit(ctx->config);

    if (output_current > max_current) output_current = max_current;
    if (output_current < -max_current) output_current = -max_current;

    return output_current;
}

int16_t MotorDriver_GetCommandLimit(uint8_t motor_id)
{
    MotorContext_t *ctx = MotorDriver_GetContext(motor_id);
    return ctx && ctx->initialized ? DjiMotor_CommandLimit(ctx->config) : 0;
}

MotorContext_t* MotorDriver_GetContext(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return NULL;
    }
    return &g_motor_contexts[motor_id];
}

/**
 * @brief Check if motor is initialized
 */
bool MotorDriver_IsInitialized(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return false;
    }
    return g_motor_contexts[motor_id].initialized &&
           g_motor_contexts[motor_id].angle_initialized;
}

/**
 * @brief Set motor angle target
 */
void MotorDriver_SetAngleTarget(uint8_t motor_id, float target_angle)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (ctx->initialized) {
        ctx->angle_target = target_angle;
    }
}

/**
 * @brief Reset motor PID integrals
 */
void MotorDriver_ResetPID(uint8_t motor_id)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (ctx->initialized) {
        PID_Reset(&ctx->pid_outer);
        PID_Reset(&ctx->pid_inner);
    }
}

// ========== Message Center Callbacks ==========

/**
 * @brief GM6020 feedback callback
 */
static void on_gm6020_feedback(const MsgEvent *ev, void *user)
{
    (void)user;

    if (ev->size == sizeof(GM6020FeedbackEvent)) {
        const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;
        MotorDriver_UpdateFeedback(m->id, m->angle, m->speed, m->current, m->tick_ms);
    }
}

/**
 * @brief M3508/M2006 feedback callback
 */
static void on_motor_feedback(const MsgEvent *ev, void *user)
{
    (void)user;

    if (ev->size == sizeof(MotorFeedbackEvent)) {
        const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
        MotorDriver_UpdateFeedback(m->id, m->angle, m->speed, m->current, m->tick_ms);
    }
}

// ========== Application Layer Interface ==========

/**
 * @brief Find motors by role
 */
uint8_t MotorDriver_FindByRole(MotorRole_e role, uint8_t *motor_ids, uint8_t max_count)
{
    if (motor_ids == NULL || max_count == 0 || g_robot_config == NULL) {
        return 0;
    }

    uint8_t count = 0;
    for (uint8_t i = 0; i < g_robot_config->total_motor_count && count < max_count; i++) {
        const MotorConfig_t *motor_cfg = &g_robot_config->motor_configs[i];
        if (motor_cfg->role == role) {
            motor_ids[count] = motor_cfg->motor_id;
            count++;
        }
    }

    return count;
}

/**
 * @brief Send current command to a motor
 */
void MotorDriver_SendCurrent(uint8_t motor_id, int16_t current)
{
    if (motor_id >= MOTOR_DRIVER_MAX_MOTORS) {
        return;
    }

    MotorContext_t *ctx = &g_motor_contexts[motor_id];
    if (!ctx->initialized || ctx->config == NULL) {
        // Warning: Motor not initialized (rate limited)
        static uint32_t last_warn = 0;
        if (BspTime_NowMs() - last_warn > 1000) {
            LOG_ERROR(LOG_TAG_MOT, "Motor %d not initialized (init=%d, cfg=%p)",
                      motor_id, ctx->initialized, (void*)ctx->config);
            last_warn = BspTime_NowMs();
        }
        return;
    }

    // Determine which CAN manager to use
    CAN_Manager_t *can_mgr = (ctx->config->can_channel == CAN_CHANNEL_1) ?
                              &can1_manager : &can2_manager;

    // Send current via CAN manager
    CAN_Manager_SendMotorCurrent(can_mgr, motor_id, current);
}

/**
 * @brief Flush all pending motor current commands
 */
void MotorDriver_FlushAll(void)
{
    // Flush both CAN channels
    CAN_Manager_FlushTx(&can1_manager);
    CAN_Manager_FlushTx(&can2_manager);
}
