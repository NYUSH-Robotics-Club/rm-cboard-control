/*
 * 接收发射命令并控制拨盘和摩擦轮。
 * 电机选择来自角色配置；拨弹持续高电流时短时反转一次，重堵停拨盘。
 * 任一发射电机失联时三台归零并清除控制历史，不在这里处理协议。
 */
#include "shooter_controller.h"
#include "motor_service.h"
#include <string.h>
#include "message_center.h"
#include "motor_messages.h"
#include "sensor_messages.h"
#include "control_messages.h"
#include "bsp_time.h"
#include "bsp_can.h"

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

/* 自定义堵转电流阈值：反馈绝对值的原始刻度，默认800；填写正整数。 */
int16_t g_feed_stall_current_threshold = 800;

// Static variables for app wrapper
static ShootCmd s_last_cmd;
static SensorData s_last_sensor;
static ShooterController s_ctrl;

// Shooter motor configuration (dynamically assigned during init)
static uint8_t s_feed_motor_id = 0xFF;      // Turntable/feed motor
static uint8_t s_friction1_motor_id = 0xFF; // Friction wheel 1
static uint8_t s_friction2_motor_id = 0xFF; // Friction wheel 2

/* 首帧标志独立于时间戳，避免开机100ms内把全零缓存误判为在线。 */
static bool ShooterFeedbackHealthy(const ShooterController *controller, uint32_t now_ms)
{
    return s_feed_motor_id != 0xFF && s_friction1_motor_id != 0xFF &&
           s_friction2_motor_id != 0xFF && controller->feedback_seen[0] &&
           controller->feedback_seen[1] && controller->feedback_seen[2] &&
           now_ms - controller->turntable_feedback.last_update_time <= MOTOR_FEEDBACK_TIMEOUT_MS &&
           now_ms - controller->shooter1_feedback.last_update_time <= MOTOR_FEEDBACK_TIMEOUT_MS &&
           now_ms - controller->shooter2_feedback.last_update_time <= MOTOR_FEEDBACK_TIMEOUT_MS;
}

static float RampTowards(float current, float target, float step)
{
    if (current < target) { current += step; if (current > target) current = target; }
    else if (current > target) { current -= step; if (current < target) current = target; }
    return current;
}

/* 换向时清掉原方向积分/输出滤波，并用当前RPM初始化微分，避免旧出力抵消反转。 */
static void ResetFeedPid(ShooterController *controller)
{
    PID_Reset(&controller->turntable_pid);
    controller->turntable_pid.last_measure = controller->turntable_feedback.speed;
}

static void CancelFeedRecovery(ShooterController *controller)
{
    memset(&controller->feed_recovery, 0, sizeof(controller->feed_recovery));
    controller->turntable_target = 0.0f;
    controller->ramped_turntable = 0.0f;
    ResetFeedPid(controller);
}

/* 在发流前运行；反馈/总线联锁必须先通过。时间来自毫秒时钟，不按回调次数累计。 */
static void UpdateFeedRecovery(ShooterController *controller, uint32_t now_ms)
{
    FeedRecovery *recovery = &controller->feed_recovery;
    if (g_feed_stall_current_threshold <= 0) {
        recovery->state = FEED_RECOVERY_BLOCKED;
        recovery->high_current_active = false;
        controller->turntable_target = 0.0f;
        controller->ramped_turntable = 0.0f;
        ResetFeedPid(controller);
        return;
    }
    if (recovery->state == FEED_RECOVERY_REVERSING) {
        if (now_ms - recovery->reverse_since_ms < FEED_REVERSE_DURATION_MS) {
            controller->turntable_target = -FEED_REVERSE_SPEED_RPM;
            controller->ramped_turntable = controller->turntable_target;
            return;
        }
        /* 到时结束反转，本轮零电流，下轮从零按原斜坡恢复正向拨弹。 */
        recovery->state = FEED_RECOVERY_MONITORING;
        recovery->high_current_active = false;
        controller->turntable_target = 0.0f;
        controller->ramped_turntable = 0.0f;
        ResetFeedPid(controller);
        return;
    }
    if (recovery->state == FEED_RECOVERY_BLOCKED) {
        controller->turntable_target = 0.0f;
        controller->ramped_turntable = 0.0f;
        return;
    }

    /* 先扩展到32位，保证-32768的绝对值也能正确判断。 */
    int32_t current = controller->turntable_feedback.current;
    if (current < 0) current = -current;
    if (current < g_feed_stall_current_threshold || controller->ramped_turntable <= 0.0f) {
        recovery->high_current_active = false;
        return;
    }
    /* 长时间未执行控制不算连续观测，恢复后重新计时。 */
    if (!recovery->high_current_active ||
        now_ms - recovery->last_check_ms > MOTOR_FEEDBACK_TIMEOUT_MS) {
        recovery->high_current_active = true;
        recovery->high_current_since_ms = now_ms;
    }
    recovery->last_check_ms = now_ms;
    if (now_ms - recovery->high_current_since_ms < FEED_STALL_DURATION_MS) return;

    recovery->high_current_active = false;
    ResetFeedPid(controller);
    if (recovery->reverse_attempted) {
        recovery->state = FEED_RECOVERY_BLOCKED;
        controller->turntable_target = 0.0f;
        controller->ramped_turntable = 0.0f;
    } else {
        recovery->state = FEED_RECOVERY_REVERSING;
        recovery->reverse_attempted = true;
        recovery->reverse_since_ms = now_ms;
        controller->turntable_target = -FEED_REVERSE_SPEED_RPM;
        controller->ramped_turntable = controller->turntable_target;
    }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
{
    if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) {
        PID_Reset(pid);
        return 0;
    }
    float current_speed = feedback->speed;
    return (int16_t)PID_Calculate(pid, target, current_speed);
}

void ShooterController_Init(ShooterController *controller)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(ShooterController));
    s_feed_motor_id = s_friction1_motor_id = s_friction2_motor_id = 0xFF;

    // Find shooter motors by role (module layer handles config)
    uint8_t feed_motors[1];
    uint8_t friction_motors[2];

    // Find feed motor
    if (MotorService_FindByRole(MOTOR_ROLE_SHOOTER_FEED, feed_motors, 1) > 0) {
        s_feed_motor_id = feed_motors[0];

        // Initialize turntable PID from motor configuration
        const MotorConfig_t *config = MotorService_GetConfig(s_feed_motor_id);
        if (config) {
            PID_Init(&controller->turntable_pid,
                     config->pid_outer.kp,
                     config->pid_outer.ki,
                     config->pid_outer.kd,
                     config->pid_outer.output_max,
                     config->pid_outer.integral_max);
        }
    }

    // Find friction wheels
    uint8_t friction_count = MotorService_FindByRole(MOTOR_ROLE_SHOOTER_FRICTION, friction_motors, 2);
    if (friction_count > 0) {
        s_friction1_motor_id = friction_motors[0];

        // Initialize shooter1 PID from motor configuration
        const MotorConfig_t *config = MotorService_GetConfig(s_friction1_motor_id);
        if (config) {
            PID_Init(&controller->shooter1_pid,
                     config->pid_outer.kp,
                     config->pid_outer.ki,
                     config->pid_outer.kd,
                     config->pid_outer.output_max,
                     config->pid_outer.integral_max);
        }
    }

    if (friction_count > 1) {
        s_friction2_motor_id = friction_motors[1];

        // Initialize shooter2 PID from motor configuration
        const MotorConfig_t *config = MotorService_GetConfig(s_friction2_motor_id);
        if (config) {
            PID_Init(&controller->shooter2_pid,
                     config->pid_outer.kp,
                     config->pid_outer.ki,
                     config->pid_outer.kd,
                     config->pid_outer.output_max,
                     config->pid_outer.integral_max);
        }
    }
}

void ShooterController_Update(ShooterController *controller, SensorData* sensor_data)
{
    if (controller == NULL) return;
    (void)sensor_data;  // Not needed anymore
    if (!s_last_cmd.feed_enabled) CancelFeedRecovery(controller);
    
    controller->feedback_fault = !ShooterFeedbackHealthy(controller, BspTime_NowMs());
    if (controller->feedback_fault || !BspCan_OutputsArmed()) {
        ShooterController_Stop(controller);
        return;
    }

    // Use standardized command from cmd_controller
    controller->enabled = s_last_cmd.friction_enabled;
    
    // Set turntable target (only feed when feed_enabled)
    float turntable_target = s_last_cmd.feed_enabled ? MOTOR5_CONST_SPEED : 0.0f;
    
    // Set shooter wheel targets
    bool friction_requested = BspCan_OutputsArmed() && s_last_cmd.friction_enabled;
    float shooter1_target = friction_requested ? -SHOOTER_CONST_SPEED : 0.0f;
    float shooter2_target = friction_requested ?  SHOOTER_CONST_SPEED : 0.0f;
    
    if (controller->feed_recovery.state == FEED_RECOVERY_MONITORING) {
        controller->turntable_target = turntable_target;
        controller->ramped_turntable = RampTowards(controller->ramped_turntable, turntable_target, SHOOTER_RAMP_STEP);
    }
    controller->shooter1_target = shooter1_target;
    controller->shooter2_target = shooter2_target;

    // Apply ramping
    controller->ramped_shooter1 = RampTowards(controller->ramped_shooter1, shooter1_target, SHOOTER_RAMP_STEP);
    controller->ramped_shooter2 = RampTowards(controller->ramped_shooter2, shooter2_target, SHOOTER_RAMP_STEP);
}

void ShooterController_ComputeCurrents(ShooterController *controller, uint32_t current_tick)
{
    if (controller == NULL) return;

    /* 独立检查计算入口，调用者跳过Update也不能沿用旧电流。 */
    if (!s_last_cmd.feed_enabled) CancelFeedRecovery(controller);
    controller->feedback_fault = !ShooterFeedbackHealthy(controller, current_tick);
    if (controller->feedback_fault || !BspCan_OutputsArmed()) {
        ShooterController_Stop(controller);
        return;
    }

    if (s_last_cmd.feed_enabled) UpdateFeedRecovery(controller, current_tick);

    /* 反转结束/重堵/取消时直接零电流；摩擦轮继续使用原来的斜坡和零速闭环。 */
    controller->output_currents[0] = s_last_cmd.feed_enabled && controller->ramped_turntable != 0.0f
        ? ComputeSingleMotorCurrent(&controller->turntable_pid, controller->ramped_turntable,
                                    &controller->turntable_feedback, current_tick) : 0;
    /* Keep the original zero-speed loop active after the ramp reaches zero. */
    controller->output_currents[1] = BspCan_OutputsArmed() ? ComputeSingleMotorCurrent(&controller->shooter1_pid, controller->ramped_shooter1, &controller->shooter1_feedback, current_tick) : 0;
    controller->output_currents[2] = 0;  // Not used
    controller->output_currents[3] = BspCan_OutputsArmed() ? ComputeSingleMotorCurrent(&controller->shooter2_pid, controller->ramped_shooter2, &controller->shooter2_feedback, current_tick) : 0;

    // Send motor currents (module layer handles CAN)
    if (s_feed_motor_id != 0xFF) {
        (void)MotorService_CommandConfigured(s_feed_motor_id,
                                             controller->ramped_turntable,
                                             controller->output_currents[0]);
    }
    if (s_friction1_motor_id != 0xFF) {
        (void)MotorService_CommandConfigured(s_friction1_motor_id,
                                             controller->ramped_shooter1,
                                             controller->output_currents[1]);
    }
    if (s_friction2_motor_id != 0xFF) {
        (void)MotorService_CommandConfigured(s_friction2_motor_id,
                                             controller->ramped_shooter2,
                                             controller->output_currents[3]);
    }
}

void ShooterController_SetTurntableSpeed(ShooterController *controller, float speed)
{ if (controller == NULL) return; controller->turntable_target = speed; }

void ShooterController_SetShooterSpeeds(ShooterController *controller, float shooter1_speed, float shooter2_speed)
{ if (controller == NULL) return; controller->shooter1_target = shooter1_speed; controller->shooter2_target = shooter2_speed; }

void ShooterController_Stop(ShooterController *controller)
{
    if (controller == NULL) return;
    controller->enabled = false;
    controller->turntable_target = 0.0f;
    controller->shooter1_target = 0.0f;
    controller->shooter2_target = 0.0f;
    controller->ramped_turntable = 0.0f;
    controller->ramped_shooter1 = 0.0f;
    controller->ramped_shooter2 = 0.0f;
    controller->ramped_yaw = 0.0f;
    controller->feed_recovery.high_current_active = false;
    if (controller->feed_recovery.state == FEED_RECOVERY_REVERSING) {
        controller->feed_recovery.state = FEED_RECOVERY_MONITORING;
    }
    PID_Reset(&controller->turntable_pid);
    PID_Reset(&controller->shooter1_pid);
    PID_Reset(&controller->shooter2_pid);
    PID_Reset(&controller->yaw_pid);
    memset(controller->output_currents, 0, sizeof(controller->output_currents));
    /* 直接零命令避免零速闭环仍产生制动电流；失联节点可能无法收到。 */
    if (s_feed_motor_id != 0xFF) (void)MotorService_CommandCurrent(s_feed_motor_id, 0);
    if (s_friction1_motor_id != 0xFF) (void)MotorService_CommandCurrent(s_friction1_motor_id, 0);
    if (s_friction2_motor_id != 0xFF) (void)MotorService_CommandCurrent(s_friction2_motor_id, 0);
}

const int16_t* ShooterController_GetOutputCurrents(const ShooterController *controller)
{ if (controller == NULL) return NULL; return controller->output_currents; }

bool ShooterController_IsRunning(const ShooterController *controller)
{
    if (controller == NULL) return false;
    return controller->enabled || 
           controller->ramped_turntable != 0 || 
           controller->ramped_shooter1 != 0 || 
           controller->ramped_shooter2 != 0;
}

void ShooterController_UpdateMotorFeedback(ShooterController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
{
    if (controller == NULL) return;

    Motor_Feedback *feedback = NULL;

    // Dynamically match motor_id to feedback structure
    if (motor_id == s_feed_motor_id) {
        feedback = &controller->turntable_feedback;
        controller->feedback_seen[0] = true;
        /* 即使同轮派发稍后又来高电流，已观测到的低电流也会打断连续计时。 */
        if (current > -g_feed_stall_current_threshold && current < g_feed_stall_current_threshold) {
            controller->feed_recovery.high_current_active = false;
        }
    }
    else if (motor_id == s_friction1_motor_id) {
        feedback = &controller->shooter1_feedback;
        controller->feedback_seen[1] = true;
    }
    else if (motor_id == s_friction2_motor_id) {
        feedback = &controller->shooter2_feedback;
        controller->feedback_seen[2] = true;
    }
    else {
        return;  // Not a shooter motor
    }

    if (feedback != NULL) {
        feedback->angle = angle;
        feedback->speed = speed;
        feedback->current = current;
        feedback->temp = temp;
        feedback->last_update_time = current_tick;
    }
}

// Subscription callbacks
static void on_shoot_cmd(const MsgEvent *ev, void *user) {
    (void)user;
    if (ev->size == sizeof(ShootCmd)) {
        memcpy(&s_last_cmd, ev->data, sizeof(ShootCmd));
        // Update controller and compute currents when command arrives
        ShooterController_Update(&s_ctrl, &s_last_sensor);
        ShooterController_ComputeCurrents(&s_ctrl, BspTime_NowMs());
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

        // Check if this motor is a shooter motor
        if (m->id == s_feed_motor_id || m->id == s_friction1_motor_id || m->id == s_friction2_motor_id) {
            ShooterController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
        }
    }
}

void ShooterApp_Init(void) {
    memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    memset(&s_last_sensor, 0, sizeof(s_last_sensor));

    // Initialize shooter controller (module layer handles config)
    ShooterController_Init(&s_ctrl);

    (void)MsgCenter_Subscribe(TOPIC_SHOOT_CMD, on_shoot_cmd, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
}

const ShooterController* ShooterApp_GetController(void) {
    return &s_ctrl;
}
