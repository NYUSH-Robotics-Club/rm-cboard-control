/*
 * 控制现有哨兵的四个驱动电机和两个转向电机。
 * 这是项目已有的特殊舵轮布局，不应当成通用四模块舵轮公式。
 */
// Implements the SAME exported symbols as chassis_controller.c, but for Sentry
// Swerve:
// - 4x M3508 drive motors (role: MOTOR_ROLE_CHASSIS_DRIVE)
// - 2x GM6020 steer motors (role: MOTOR_ROLE_CHASSIS_STEER)

#include "chassis_controller.h" // IMPORTANT: use the common header name
#include "control_messages.h"
#include "chassis_strategy.h"
#include "message_center.h"
#include "motor_messages.h"
#include "motor_driver.h"
#include "motor_service.h"
#include "printing.h"
#include "logger.h"
#include "sensor_messages.h"
#include "bsp_time.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_FEEDBACK_TIMEOUT_MS (100U)

// Math constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Static variables for app wrapper
static ChassisCmd s_last_cmd;
static SensorData s_last_sensor;
static ChassisController s_ctrl;
static const ChassisStrategy *s_chassis_strategy = NULL;

// Motor configuration (dynamically assigned during init)
static uint8_t s_drive_motor_ids[CHASSIS_MOTOR_COUNT];
static int8_t s_drive_motor_directions[CHASSIS_MOTOR_COUNT];
static uint8_t s_drive_motor_count = 0;

#define CHASSIS_STEER_COUNT 2
static uint8_t s_steer_motor_ids[CHASSIS_STEER_COUNT];
static int8_t s_steer_motor_directions[CHASSIS_STEER_COUNT];
static uint8_t s_steer_motor_count = 0;

static void ResetPidIntegrals(ChassisController *controller) {
  for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    controller->speed_pids[i].integral = 0.0f;
  }
  for (int i = 0; i < CHASSIS_STEER_COUNT; i++) {
    controller->steer_pids[i].integral = 0.0f;
  }
}

static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target,
                                         Motor_Feedback *feedback,
                                         uint32_t current_tick) {
  if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) {
    return 0;
  }
  float measured = feedback->speed; // drive: RPM
  return (int16_t)PID_Calculate(pid, target, measured);
}

void ChassisController_Init(ChassisController *controller) {
  if (controller == NULL)
    return;

  memset(controller, 0, sizeof(ChassisController));
  s_chassis_strategy = ChassisStrategy_GetActive();
  if (!s_chassis_strategy ||
      s_chassis_strategy->execution != CHASSIS_EXECUTION_LEGACY_CONTROLLER) {
    LOG_ERROR(LOG_TAG_SEN, "Swerve strategy/config mismatch");
  }

  // Find drive motors by role
  s_drive_motor_count = MotorService_FindByRole(
      MOTOR_ROLE_CHASSIS_DRIVE, s_drive_motor_ids, CHASSIS_MOTOR_COUNT);

  // Find steer motors by role (requires enum MOTOR_ROLE_CHASSIS_STEER)
  s_steer_motor_count = MotorService_FindByRole(
      MOTOR_ROLE_CHASSIS_STEER, s_steer_motor_ids, CHASSIS_STEER_COUNT);

  // Log found steer motors
  LOG_INFO(LOG_TAG_SEN, "Found %d steer motors", s_steer_motor_count);
  for (uint8_t i = 0; i < s_steer_motor_count; i++) {
    LOG_DEBUG(LOG_TAG_SEN, "Steer motor[%d] = %d", i, s_steer_motor_ids[i]);
  }

  // Init drive PIDs from motor contexts
  for (uint8_t i = 0; i < s_drive_motor_count; i++) {
    MotorContext_t *ctx = MotorDriver_GetContext(s_drive_motor_ids[i]);
    if (ctx && ctx->config) {
      s_drive_motor_directions[i] = ctx->config->direction;

      PID_Init(&controller->speed_pids[i], ctx->config->pid_outer.kp,
               ctx->config->pid_outer.ki, ctx->config->pid_outer.kd,
               ctx->config->pid_outer.output_max,
               ctx->config->pid_outer.integral_max);

      controller->target_speeds[i] = 0.0f;
    }
  }

  // Init steer PIDs from motor contexts (outer PID is angle/position PID per
  // your config)
  for (uint8_t i = 0; i < s_steer_motor_count; i++) {
    MotorContext_t *ctx = MotorDriver_GetContext(s_steer_motor_ids[i]);
    if (ctx && ctx->config) {
      s_steer_motor_directions[i] = ctx->config->direction;

      PID_Init(&controller->steer_pids[i], ctx->config->pid_outer.kp,
               ctx->config->pid_outer.ki, ctx->config->pid_outer.kd,
               ctx->config->pid_outer.output_max,
               ctx->config->pid_outer.integral_max);

      controller->steer_target_angles[i] =
          (float)ctx->config->limits.gm6020.initial_angle;
    }
  }
}

void ChassisController_Update(ChassisController *controller,
                              SensorData *sensor_data) {
  (void)sensor_data;
  if (controller == NULL)
    return;

  float vx_norm = s_last_cmd.vx;
  float vy_norm = s_last_cmd.vy;

  // Calculate joystick magnitude (speed) and direction (angle)
  float magnitude = sqrtf(vx_norm * vx_norm + vy_norm * vy_norm);

  // Drive speed: proportional to joystick magnitude
  float max_speed = (float)CHASSIS_DEMO_TARGET_SPEED;
  float drive_speed = magnitude * max_speed;

  float drive_direction = 1.0f;

  if (magnitude > 0.1f) {
    float target_angle_rad = atan2f(vx_norm, -vy_norm);
    float joystick_angle_ticks = target_angle_rad * (8192.0f / (2.0f * M_PI));

    // Map joystick to ±90deg range (ensures forward/backward map to same angle)
    float canonical_angle_ticks = joystick_angle_ticks;
    if (canonical_angle_ticks > 2048.0f) {
      canonical_angle_ticks -= 4096.0f;
      drive_direction = -1.0f;
    } else if (canonical_angle_ticks < -2048.0f) {
      canonical_angle_ticks += 4096.0f;
      drive_direction = -1.0f;
    }

    // Shortest path optimization
    if (s_steer_motor_count > 0) {
      MotorContext_t *ctx0 = MotorDriver_GetContext(s_steer_motor_ids[0]);
      if (ctx0 && ctx0->config && ctx0->angle_initialized) {
        float initial0 = (float)ctx0->config->limits.gm6020.initial_angle;
        float last_target0 = controller->steer_target_angles[0];

        float option1_angle = initial0 + canonical_angle_ticks;
        while (option1_angle >= 8192.0f) option1_angle -= 8192.0f;
        while (option1_angle < 0.0f) option1_angle += 8192.0f;

        float option2_angle = initial0 + canonical_angle_ticks + 4096.0f;
        while (option2_angle >= 8192.0f) option2_angle -= 8192.0f;
        while (option2_angle < 0.0f) option2_angle += 8192.0f;

        float diff1 = option1_angle - last_target0;
        if (diff1 > 4096.0f) diff1 -= 8192.0f;
        if (diff1 < -4096.0f) diff1 += 8192.0f;

        float diff2 = option2_angle - last_target0;
        if (diff2 > 4096.0f) diff2 -= 8192.0f;
        if (diff2 < -4096.0f) diff2 += 8192.0f;

        float final_angle_ticks;
        if (fabsf(diff1) <= fabsf(diff2)) {
          final_angle_ticks = canonical_angle_ticks;
        } else {
          final_angle_ticks = canonical_angle_ticks + 4096.0f;
          while (final_angle_ticks >= 4096.0f) final_angle_ticks -= 8192.0f;
          while (final_angle_ticks < -4096.0f) final_angle_ticks += 8192.0f;
          drive_direction = -drive_direction;
        }

        for (uint8_t i = 0; i < s_steer_motor_count; i++) {
          MotorContext_t *ctx = MotorDriver_GetContext(s_steer_motor_ids[i]);
          if (ctx && ctx->config && ctx->angle_initialized) {
            float initial = (float)ctx->config->limits.gm6020.initial_angle;
            float target_angle = initial + final_angle_ticks;

            while (target_angle >= 8192.0f)
              target_angle -= 8192.0f;
            while (target_angle < 0.0f)
              target_angle += 8192.0f;

            controller->steer_target_angles[i] = target_angle;
          }
        }
      }
    }

    // Apply drive speed with direction to ALL drive motors
    if (s_drive_motor_count >= 4) {
      for (uint8_t i = 0; i < s_drive_motor_count; i++) {
        controller->target_speeds[i] =
            s_drive_motor_directions[i] * drive_speed * drive_direction;
      }
    }
  } else {
    // Joystick released, stop all drive motors
    if (s_drive_motor_count >= 4) {
      for (uint8_t i = 0; i < s_drive_motor_count; i++) {
        controller->target_speeds[i] = 0.0f;
      }
    }
  }

  controller->running = s_last_cmd.enabled;
}

static int16_t SteerController_CascadeControl(uint8_t motor_id,
                                              float target_angle) {
  MotorContext_t *steer = MotorDriver_GetContext(motor_id);
  if (!steer || !steer->angle_initialized || !steer->config) {
    // Debug: Steer motor not initialized
    LOG_WARN(LOG_TAG_SEN, "Steer motor %d not initialized (ptr=%p init=%d)",
             motor_id, (void*)steer, (steer ? steer->angle_initialized : -1));
    return 0;
  }

  // Use provided target angle
  steer->angle_target = target_angle;

  // Encoder range
  float enc_max = (steer->config->limits.gm6020.angle_max > 0.0f)
                      ? steer->config->limits.gm6020.angle_max
                      : 8192.0f;

  // Wrap target (already wrapped in ChassisController_Update, but ensure here)
  if (steer->angle_target >= enc_max)
    steer->angle_target -= enc_max;
  else if (steer->angle_target < 0.0f)
    steer->angle_target += enc_max;

  // Angle error (shortest path)
  float current_angle = (float)steer->angle_raw;
  float angle_error = steer->angle_target - current_angle;

  if (fabsf(angle_error) < 1.0f)
    angle_error = 0.0f;

  if (angle_error > enc_max / 2.0f)
    angle_error -= enc_max;
  if (angle_error < -enc_max / 2.0f)
    angle_error += enc_max;

  // OUTER: angle → speed
  float cmd_angle_to_speed =
      PID_Calculate(&steer->pid_outer, 0.0f, -angle_error);

  // RPM limit (stability)
  const float rpm_limit = 300.0f;
  if (cmd_angle_to_speed > rpm_limit)
    cmd_angle_to_speed = rpm_limit;
  if (cmd_angle_to_speed < -rpm_limit)
    cmd_angle_to_speed = -rpm_limit;

  // INNER: speed → current
  float speed_feedback = (float)steer->speed_rpm;

  float cmd_speed_to_current =
      PID_Calculate(&steer->pid_inner, cmd_angle_to_speed, speed_feedback);

  // Current clamp
  const float current_limit = 25000.0f;
  if (cmd_speed_to_current > current_limit)
    cmd_speed_to_current = current_limit;
  if (cmd_speed_to_current < -current_limit)
    cmd_speed_to_current = -current_limit;

  return (int16_t)cmd_speed_to_current;
}

void ChassisController_ComputeCurrents(ChassisController *controller,
                                       uint32_t current_tick) {
  if (controller == NULL)
    return;

  static uint32_t last = 0;
  if (BspTime_NowMs() - last > 200) {
    last = BspTime_NowMs();

    // Sentry chassis status logging (5Hz rate limited in main.c)
    LOG_INFO(LOG_TAG_SEN, "enabled=%d", (int)s_last_cmd.enabled);

    // Drive motor debug
    for (uint8_t i = 0; i < s_drive_motor_count; i++) {
      uint32_t dt =
          BspTime_NowMs() - controller->motor_feedbacks[i].last_update_time;
      LOG_DEBUG(LOG_TAG_SEN, "drive[%d] id=%d tgt=%.1f fb=%.1f dt=%lu out=%d", i,
                s_drive_motor_ids[i], controller->target_speeds[i],
                controller->motor_feedbacks[i].speed, (unsigned long)dt,
                controller->output_currents[i]);
    }

    // Steer motor debug
    for (uint8_t i = 0; i < s_steer_motor_count; i++) {
      uint32_t dt =
          BspTime_NowMs() - controller->steer_feedbacks[i].last_update_time;
      LOG_DEBUG(LOG_TAG_SEN, "steer[%d] id=%d tgt=%.1f ang=%u spd=%.1f dt=%lu out=%d", i,
                s_steer_motor_ids[i], controller->steer_target_angles[i],
                controller->steer_feedbacks[i].angle,
                controller->steer_feedbacks[i].speed, (unsigned long)dt,
                controller->steer_output_currents[i]);
    }
  }

  // Drive currents
  for (int i = 0; i < s_drive_motor_count; i++) {
    int16_t motor_current = ComputeSingleMotorCurrent(
        &controller->speed_pids[i], controller->target_speeds[i],
        &controller->motor_feedbacks[i], current_tick);
    controller->output_currents[i] = motor_current;
    (void)MotorService_CommandConfigured(s_drive_motor_ids[i],
                                         controller->target_speeds[i],
                                         motor_current);
  }

  // Steer currents
  // ===== Steer currents (angle position control) =====
  for (int i = 0; i < s_steer_motor_count; i++) {
    int16_t motor_current = SteerController_CascadeControl(
        s_steer_motor_ids[i],
        controller->steer_target_angles[i] // Use target angle from Update()
    );

    controller->steer_output_currents[i] = motor_current;
    (void)MotorService_CommandConfigured(s_steer_motor_ids[i],
                                         controller->steer_target_angles[i],
                                         motor_current);
  }
}

void ChassisController_SetTargetSpeeds(
    ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT]) {
  if (controller == NULL || speeds == NULL)
    return;
  for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    controller->target_speeds[i] = speeds[i];
  }
}

void ChassisController_SetSteerTargetAngles(
    ChassisController *controller, const float angles[CHASSIS_STEER_COUNT]) {
  if (controller == NULL || angles == NULL)
    return;
  for (int i = 0; i < CHASSIS_STEER_COUNT; i++) {
    controller->steer_target_angles[i] = angles[i];
  }
}

void ChassisController_Stop(ChassisController *controller) {
  if (controller == NULL)
    return;
  controller->running = false;
  ResetPidIntegrals(controller);

  for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    controller->target_speeds[i] = 0.0f;
  }
  // Steer holds by default
}

const int16_t *
ChassisController_GetOutputCurrents(const ChassisController *controller) {
  if (controller == NULL)
    return NULL;
  return controller->output_currents;
}

bool ChassisController_IsRunning(const ChassisController *controller) {
  if (controller == NULL)
    return false;
  for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    if (controller->target_speeds[i] != 0.0f)
      return true;
  }
  return false;
}

// Drive feedback updater: motor_id is local index 0..3 (same as your existing
// chassis_controller)
void ChassisController_UpdateMotorFeedback(ChassisController *controller,
                                           uint8_t motor_id, uint16_t angle,
                                           int16_t speed, int16_t current,
                                           uint8_t temp,
                                           uint32_t current_tick) {
  if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT)
    return;
  controller->motor_feedbacks[motor_id].angle = angle;
  controller->motor_feedbacks[motor_id].speed = speed;
  controller->motor_feedbacks[motor_id].current = current;
  controller->motor_feedbacks[motor_id].temp = temp;
  controller->motor_feedbacks[motor_id].last_update_time = current_tick;
}

// Steer feedback updater: index 0..1
void ChassisController_UpdateSteerFeedback(ChassisController *controller,
                                           uint8_t index, uint16_t angle,
                                           int16_t speed, int16_t current,
                                           uint8_t temp,
                                           uint32_t current_tick) {
  if (controller == NULL || index >= CHASSIS_STEER_COUNT)
    return;
  controller->steer_feedbacks[index].angle = angle;
  controller->steer_feedbacks[index].speed = speed;
  controller->steer_feedbacks[index].current = current;
  controller->steer_feedbacks[index].temp = temp;
  controller->steer_feedbacks[index].last_update_time = current_tick;
}

// =====================
// Subscription callbacks
// =====================
static void on_chassis_cmd(const MsgEvent *ev, void *user) {
  (void)user;
  if (ev->size == sizeof(ChassisCmd)) {
    memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
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

    // Drive motor match -> update drive feedback by LOCAL INDEX 0..3
    for (uint8_t i = 0; i < s_drive_motor_count; i++) {
      if (m->id == s_drive_motor_ids[i]) {
        ChassisController_UpdateMotorFeedback(&s_ctrl, i, m->angle, m->speed,
                                              m->current, m->temp, m->tick_ms);
        return;
      }
    }
  }
}

// NEW: GM6020 feedback callback for steer motors
static void on_gm6020_feedback(const MsgEvent *ev, void *user) {
  (void)user;
  if (ev->size == sizeof(GM6020FeedbackEvent)) {
    const GM6020FeedbackEvent *m = (const GM6020FeedbackEvent *)ev->data;

    // Steer motor match -> update steer feedback by LOCAL INDEX 0..1
    for (uint8_t i = 0; i < s_steer_motor_count; i++) {
      if (m->id == s_steer_motor_ids[i]) {
        ChassisController_UpdateSteerFeedback(&s_ctrl, i, m->angle, m->speed,
                                              m->current, 0, m->tick_ms);
        return;
      }
    }
  }
}

void ChassisApp_Init(void) {
  memset(&s_last_cmd, 0, sizeof(s_last_cmd));
  memset(&s_last_sensor, 0, sizeof(s_last_sensor));

  ChassisController_Init(&s_ctrl);

  (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
  (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
  (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
  (void)MsgCenter_Subscribe(
      TOPIC_GM6020_FEEDBACK, on_gm6020_feedback,
      NULL); // Subscribe to GM6020 feedback for steer motors
}

const ChassisController *ChassisApp_GetController(void) { return &s_ctrl; }

uint8_t ChassisApp_GetMotorIds(uint8_t ids[CHASSIS_MOTOR_COUNT]) {
  if (!ids) return 0U;
  memcpy(ids, s_drive_motor_ids, s_drive_motor_count);
  return s_drive_motor_count;
}

/**
 * @brief Wait for swerve steer motors to align to initial position
 * @note Should be called after ChassisApp_Init() and CAN bus stabilization
 */
void Sentry_WaitForSteerAlignment(void) {
  const float ALIGNMENT_THRESHOLD = 100.0f; // encoder ticks tolerance
  const uint32_t TIMEOUT_MS = 5000;         // 5 seconds timeout
  const uint32_t CHECK_INTERVAL_MS = 50;

  LOG_INFO(LOG_TAG_SEN, "Waiting for steer motors to align to initial position...");

  uint32_t start_time = BspTime_NowMs();
  bool all_aligned = false;

  while (BspTime_NowMs() - start_time < TIMEOUT_MS && !all_aligned) {
    // Send zero command to hold position (motors will go to initial_angle)
    // Just dispatch messages to process feedback
    MsgCenter_Dispatch();

    // Send currents to hold steer motors at target position
    ChassisController_ComputeCurrents(&s_ctrl, BspTime_NowMs());

    BspTime_DelayMs(CHECK_INTERVAL_MS);

    // Check if all steer motors have reached target position
    all_aligned = true;
    for (uint8_t i = 0; i < s_steer_motor_count; i++) {
      MotorContext_t *steer = MotorDriver_GetContext(s_steer_motor_ids[i]);
      if (steer && steer->angle_initialized) {
        float target = steer->angle_target;
        float current = (float)steer->angle_raw;
        float error = fabsf(target - current);

        // Handle wraparound (0-8191 encoder range, 8192 ticks per revolution)
        if (error > 4096.0f) {
          error = 8192.0f - error;
        }

        LOG_DEBUG(LOG_TAG_SEN, "Steer motor %d: target=%.1f current=%.1f error=%.1f",
                  s_steer_motor_ids[i], target, current, error);

        if (error > ALIGNMENT_THRESHOLD) {
          all_aligned = false;
        }
      } else {
        all_aligned = false;
      }
    }
  }

  if (all_aligned) {
    LOG_INFO(LOG_TAG_SEN, "Steer alignment complete!");
  } else {
    LOG_WARN(LOG_TAG_SEN, "Steer alignment timeout, continuing anyway...");
  }
}
