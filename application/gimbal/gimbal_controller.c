/*
 * 执行云台 yaw/pitch 控制、速度前馈、重力补偿和耦合补偿。
 * 电机编号和参数来自机器人配置，不在这里拼接 CAN 数据。
 */
#include "gimbal_controller.h"
#include "gimbal_monitor.h"
#include "yaw_reference.h"
#include "message_center.h"
#include "motor_driver.h"
#include "motor_service.h"
#include "pid.h"
#include "printing.h"
#include "logger.h"
#include "bsp_time.h"
#include <math.h>
#include <string.h>
#include <stdatomic.h>

volatile GimbalMonitorSnapshot g_gimbal_monitor;
_Static_assert(sizeof(GimbalMonitorAxis) == 56U, "monitor axis ABI");
_Static_assert(sizeof(GimbalMonitorSnapshot) == 148U, "monitor snapshot ABI");

/* 暂存本次服务请求；每次有效命令回调先清空，停机分支也会记录零命令及状态。 */
static int16_t s_monitor_commands[2];
static RobotStatus s_monitor_status[2];

// Motor IDs (dynamically assigned during init)
static uint8_t s_pitch_motor_id = 0xFF;
static uint8_t s_yaw_motor_id = 0xFF;

/* yaw状态只由命令派发上下文修改；电机兼容字段仍保存单圈角。 */
static YawReference s_yaw_reference;
static YawControlMode s_yaw_mode;
static bool s_yaw_mode_valid;
static float s_spin_turn_offset_deg;

static void command_axis(uint8_t id, int16_t command) {
  RobotStatus status = MotorService_CommandCurrent(id, command);
  unsigned axis = id == s_yaw_motor_id ? 0U : 1U;
  s_monitor_commands[axis] = command;
  s_monitor_status[axis] = status;
}

// Gimbal tilt compensation parameters
#define GIMBAL_HEIGHT_CM (30.0f)  // 云台距地面高度 30cm
#define COMPENSATION_UPDATE_RATE_MS (100) // 更新补偿值的频率 100ms

// Static state for application
static GimbalCmd s_last_cmd;
static SensorData s_last_sensor;
static bool s_initialized = false;
static bool s_startup_position_captured = false;
static bool s_feedback_stable_seen;
static uint32_t s_feedback_stable_since_ms;

static void yaw_invalidate(void);

/* 无历史状态：前馈取本次速度内环目标，单位是电机协议原始命令刻度。
 * 0限幅直接关闭；启用后的非法参数/计算结果交由调用方停止两轴并重新对齐。
 */
static bool calculate_feedforward(const GimbalFeedforwardConfig *cfg,
                                  float speed_target_rpm, float *output) {
  *output = 0.0f;
  if (cfg->output_max == 0.0f) return true;
  if (!isfinite(cfg->output_max) || cfg->output_max < 0.0f ||
      !isfinite(cfg->velocity_gain) || !isfinite(cfg->bias) ||
      !isfinite(speed_target_rpm)) return false;
  float value = cfg->velocity_gain * speed_target_rpm + cfg->bias;
  if (!isfinite(value)) return false;
  *output = fmaxf(-cfg->output_max, fminf(value, cfg->output_max));
  return true;
}

/* Never close a position loop on a sample older than 100 ms. */
static bool axis_feedback_fresh(uint8_t id, uint32_t now_ms) {
  if (id == 0xFF) return true;
  MotorContext_t *motor = MotorDriver_GetContext(id);
  return motor && motor->initialized && motor->last_feedback_time != 0U &&
         (uint32_t)(now_ms - motor->last_feedback_time) <= 100U;
}

/*
 * The yaw/pitch coupling calculation needs two real yaw samples.  Treating the
 * encoder's power-on value as movement from zero would change the pitch target
 * on the first hold command and can make the gimbal jump during calibration.
 */
static bool s_coupling_history_valid = false;
static float s_last_coupling_yaw_angle = 0.0f;
static uint32_t s_last_coupling_yaw_time_ms = 0U;

/*
 * Capture both configured axes as one startup state.  Holding either axis
 * before its first CAN feedback could use a configured angle with a zero
 * encoder value, so the function keeps all gimbal output disabled until every
 * configured axis has reported once.
 */
static bool capture_startup_position(void) {
  MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
  MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);

  bool yaw_ready = (s_yaw_motor_id == 0xFF) ||
                   (yaw && yaw->initialized && yaw->last_feedback_time != 0U);
  bool pitch_ready = (s_pitch_motor_id == 0xFF) ||
                     (pitch && pitch->initialized &&
                      pitch->last_feedback_time != 0U);
  if (!yaw_ready || !pitch_ready) {
    return false;
  }

  float pitch_target = pitch ? (float)pitch->angle_raw : 0.0f;
  if (pitch && pitch->config) {
    float pitch_angle = (float)pitch->angle_raw;
    /* pitch每次对齐使用配置的绝对编码目标；负数保留锁存实测位置的行为。 */
    if (pitch->config->limits.gm6020.initial_angle >= 0.0f)
      pitch_target = pitch->config->limits.gm6020.initial_angle;
    if (pitch_angle < pitch->config->limits.gm6020.angle_min ||
        pitch_angle > pitch->config->limits.gm6020.angle_max ||
        pitch_target < pitch->config->limits.gm6020.angle_min ||
        pitch_target > pitch->config->limits.gm6020.angle_max) {
      /* 实际位置或启动目标越界时，两轴保持零输出。 */
      return false;
    }
  }

  if (yaw) {
    YawReference_Seed(&s_yaw_reference, yaw->angle_raw,
                      yaw->last_feedback_time, BspTime_NowMs());
    s_yaw_mode_valid = false;
    yaw->angle_target = (float)yaw->angle_raw;
    yaw->angle_initialized = true;
    PID_Reset(&yaw->pid_outer);
    PID_Reset(&yaw->pid_inner);
    s_coupling_history_valid = true;
    s_last_coupling_yaw_angle = (float)yaw->angle_raw;
    s_last_coupling_yaw_time_ms = BspTime_NowMs();
  }
  if (pitch) {
    pitch->angle_target = pitch_target;
    pitch->angle_initialized = true;
    PID_Reset(&pitch->pid_outer);
    PID_Reset(&pitch->pid_inner);
  }

  s_startup_position_captured = true;
  return true;
}

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized,
                                      SensorData *sensor_data, bool disable_yaw_pitch_compensation) {
  (void)sensor_data; // Not used for pitch
  MotorContext_t *c = MotorDriver_GetContext(id);
  MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
  if (!c || !c->angle_initialized) {
    return 0;
  }
  if (!c->config || !isfinite(rate_normalized)) {
    yaw_invalidate();
    return 0;
  }

  // Joystick control with sensitivity scaling
  float sensitivity = 60.0f; // Increased for more responsive tracking
  c->angle_target += c->config->direction * sensitivity * rate_normalized;

  /* 旧视线耦合模型需按机械结构验证，只在配置显式开启且非自瞄时改写目标。
   * 关闭时仍跟踪相邻yaw反馈，避免重新开启时补算关闭期间的累计转角。 */
  if (yaw && yaw->angle_initialized) {
    uint32_t current_time = BspTime_NowMs();
    float current_yaw_angle = (float)yaw->angle_raw;

    if (!s_coupling_history_valid) {
      /*
       * The first valid encoder value is a reference point, not a rotation.
       * Seed the history without applying any pitch correction.
       */
      s_coupling_history_valid = true;
    } else {
      // 计算yaw角度变化（处理0-8192的环绕）
      float yaw_delta = current_yaw_angle - s_last_coupling_yaw_angle;
      if (yaw_delta > 4096.0f) {
        yaw_delta -= 8192.0f;
      } else if (yaw_delta < -4096.0f) {
        yaw_delta += 8192.0f;
      }

      // 只有当yaw有显著变化且时间间隔合理时才计算补偿
      if (c->config->limits.gm6020.enable_yaw_pitch_compensation &&
          !disable_yaw_pitch_compensation && fabsf(yaw_delta) > 1.0f &&
          (current_time - s_last_coupling_yaw_time_ms) > 0U) {
        // 获取当前pitch角度
        // 编码器一圈始终8192刻度，与机械限位及其开关无关。
        const float max_encoder = 8192.0f;
        float current_angle = (float)c->angle_raw;
        float pitch_angle_rad = (current_angle / max_encoder) * (2.0f * (float)M_PI);

        // 将yaw变化转换为弧度
        float yaw_delta_rad = (yaw_delta / 8192.0f) * (2.0f * (float)M_PI);

        // 计算pitch补偿（编码器刻度）
        // Δpitch = sin(Δyaw) * tan(pitch_current)
        float pitch_compensation_rad = sinf(yaw_delta_rad) * tanf(pitch_angle_rad);
        float pitch_compensation_ticks = pitch_compensation_rad * (max_encoder / (2.0f * (float)M_PI));

        // 应用补偿到pitch目标角度（反向补偿以抵消耦合效应）
        c->angle_target -= pitch_compensation_ticks;
      }
    }

    /* Keep the reference current even while compensation is disabled. */
    s_last_coupling_yaw_angle = current_yaw_angle;
    s_last_coupling_yaw_time_ms = current_time;
  }

  // pitch在遥控增量和耦合补偿之后统一裁剪绝对编码目标，不允许绕一圈越过限位。
  bool is_pitch_motor = (c->role == MOTOR_ROLE_GIMBAL_PITCH);
  const float max_encoder = 8192.0f;

  if (is_pitch_motor) {
    if (c->angle_target > c->config->limits.gm6020.angle_max)
      c->angle_target = c->config->limits.gm6020.angle_max;
    if (c->angle_target < c->config->limits.gm6020.angle_min)
      c->angle_target = c->config->limits.gm6020.angle_min;
  } else {
    c->angle_target = fmodf(c->angle_target, max_encoder);
    if (c->angle_target < 0.0f) c->angle_target += max_encoder;
  }

  float current_angle = (float)c->angle_raw;
  float error = c->angle_target - current_angle;
  if (error > max_encoder / 2.0f)
    error -= max_encoder;
  else if (error < -max_encoder / 2.0f)
    error += max_encoder;

  float speed_target =
      PID_CalculateDivided(&c->pid_outer, error, 0.0f,PID_PITCH_OUTER_DIVIDER);
  float feedforward;
  if (!calculate_feedforward(&c->config->feedforward, speed_target, &feedforward)) {
    yaw_invalidate();
    return 0;
  }
  float cmd = PID_Calculate(&c->pid_inner, speed_target, (float)c->speed_rpm)
            + feedforward;

  if (is_pitch_motor) {
    /* 重力相位使用配置的机械零点；非有限配置沿用下方双轴停机路径。 */
    float ang01 = (current_angle - c->config->limits.gm6020.gravity_zero_angle) /
                  max_encoder;
    float ang_rad = ang01 * (2.0f * (float)M_PI);
    float gravity_ff = c->config->direction *
                       c->config->limits.gm6020.gravity_compensation *
                       sinf(ang_rad);
    cmd += gravity_ff;
  }
  if (!isfinite(cmd)) {
    yaw_invalidate();
    return 0;
  }
  float max_abs = (float)MotorDriver_GetCommandLimit(id);
  if (cmd > max_abs)
    cmd = max_abs;
  if (cmd < -max_abs)
    cmd = -max_abs;

  // Pitch PID tuning CSV (20Hz rate limited in main.c) - DISABLED for clean output
  // Format: GIM,timestamp_ms,angle_target,angle_current,speed_rpm,cmd,error,rate_scaled
  // LOG_CSV(LOG_TAG_GIM, "PITCH,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
  //         c->angle_target,
  //         current_angle,
  //         c->speed_rpm,
  //         cmd,
  //         error,
  //         rate_normalized * 300.0f);

  return (int16_t)cmd;
}

/* 配置无效时禁止出力，不能静默回退到旧的按回调次数累加。 */
static bool yaw_config_valid(const YawControlConfig *cfg) {
  return cfg && isfinite(cfg->manual_rate_deg_s) && cfg->manual_rate_deg_s > 0.0f &&
      isfinite(cfg->target_lead_deg) && cfg->target_lead_deg > 0.0f &&
      cfg->target_lead_deg <= 180.0f &&
      isfinite(cfg->manual_speed_rpm) && cfg->manual_speed_rpm > 0.0f &&
      isfinite(cfg->vision_speed_rpm) && cfg->vision_speed_rpm > 0.0f &&
      isfinite(cfg->spin_speed_rpm) && cfg->spin_speed_rpm > 0.0f;
}

static void yaw_invalidate(void) {
  s_yaw_reference.valid = false;
  s_yaw_mode_valid = false;
  s_startup_position_captured = false;
  s_feedback_stable_seen = false;
}

int16_t GimbalController_YawControlWithCompensation(float rate_normalized,
                                                    SensorData *sensor_data,
                                                    YawControlMode mode) {
  MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);
  if (!yaw || !yaw->angle_initialized || !yaw->config) {
    yaw_invalidate();
    return 0;
  }
  const YawControlConfig *cfg = yaw->config->yaw_control;
  float dt_s;
  if (!yaw_config_valid(cfg) || !isfinite(rate_normalized) ||
      (unsigned)mode > YAW_CONTROL_SPIN ||
      !YawReference_Update(&s_yaw_reference, yaw->angle_raw, yaw->speed_rpm,
                            yaw->last_feedback_time, BspTime_NowMs(), &dt_s)) {
    yaw_invalidate();
    return 0;
  }

  /* 速度调试只用电机RPM；视觉和spin位置目标不能偷偷闭合另一条位置环。 */
  if (cfg->speed_loop_only) {
    if (mode != YAW_CONTROL_MANUAL) rate_normalized = 0.0f;
    mode = YAW_CONTROL_MANUAL;
  }
  if ((mode == YAW_CONTROL_VISION && !isfinite(s_last_cmd.vision_yaw_err_rad)) ||
      (mode == YAW_CONTROL_SPIN &&
       (!sensor_data || !isfinite(sensor_data->yaw_total_angle) ||
        !isfinite(sensor_data->g_gz) || !isfinite(s_last_cmd.yaw_target_memo)))) {
    yaw_invalidate();
    return 0;
  }
  float speed_feedback = mode == YAW_CONTROL_SPIN ?
      -sensor_data->g_gz * 30.0f / (float)M_PI : (float)yaw->speed_rpm;
  bool mode_changed = !s_yaw_mode_valid || mode != s_yaw_mode;
  if (mode_changed) {
    s_yaw_reference.target_ticks = s_yaw_reference.position_ticks;
    PID_Reset(&yaw->pid_outer);
    PID_Reset(&yaw->pid_inner);
    /* 模式切换以当前速度初始化D历史，避免运动中从零测量产生尖峰。 */
    yaw->pid_inner.last_measure = speed_feedback;
    if (mode == YAW_CONTROL_SPIN) {
      float error_deg = s_last_cmd.yaw_target_memo - sensor_data->yaw_total_angle;
      float nearest_deg = YawReference_Wrap((error_deg + 180.0f) *
                                            YAW_ENCODER_TICKS / 360.0f) *
                          360.0f / YAW_ENCODER_TICKS - 180.0f;
      s_spin_turn_offset_deg = nearest_deg - error_deg;
    }
    s_yaw_mode = mode;
    s_yaw_mode_valid = true;
  }

  float rpm_limit = cfg->manual_speed_rpm;
  if (mode == YAW_CONTROL_MANUAL) {
    /* 新模式首帧不把上一模式的时间间隔积分到新目标。 */
    YawReference_Advance(&s_yaw_reference, rate_normalized, cfg->manual_rate_deg_s,
                          mode_changed ? 0.0f : dt_s, cfg->target_lead_deg);
  } else if (mode == YAW_CONTROL_VISION) {
    s_yaw_reference.target_ticks = s_yaw_reference.position_ticks +
        s_last_cmd.vision_yaw_err_rad * YAW_ENCODER_TICKS / (2.0f * (float)M_PI);
    rpm_limit = cfg->vision_speed_rpm;
  } else {
    /* 世界航向的圈数只在进入spin时选一次，运动中保留连续误差。 */
    float error_deg = s_last_cmd.yaw_target_memo - sensor_data->yaw_total_angle +
                      s_spin_turn_offset_deg;
    s_yaw_reference.target_ticks = s_yaw_reference.position_ticks +
                                  error_deg * YAW_ENCODER_TICKS / 360.0f;
    rpm_limit = cfg->spin_speed_rpm;
  }
  if (!isfinite(s_yaw_reference.target_ticks) || !isfinite(speed_feedback)) {
    yaw_invalidate();
    return 0;
  }
  yaw->angle_target = YawReference_Wrap(s_yaw_reference.target_ticks);
  float angle_error = s_yaw_reference.target_ticks - s_yaw_reference.position_ticks;
  if (fabsf(angle_error) < 0.02f) angle_error = 0.0f;
  float speed_target;
  if (cfg->speed_loop_only) {
    /* 暂时旁路下面的位置PID；回中目标是0 RPM，不是零电流或锁位置。 */
    speed_target = fmaxf(-1.0f, fminf(rate_normalized, 1.0f)) *
                   cfg->manual_rate_deg_s / 6.0f;
  } else {
    speed_target = PID_CalculateDivided(&yaw->pid_outer, 0.0f, -angle_error,
                                        PID_YAW_OUTER_DIVIDER);
  }
  speed_target = fmaxf(-rpm_limit, fminf(speed_target, rpm_limit));
  /* 用模式限速后的目标做前馈；速度环调试同样生效，不重新闭合位置环。 */
  float feedforward;
  if (!calculate_feedforward(&yaw->config->feedforward, speed_target, &feedforward)) {
    yaw_invalidate();
    return 0;
  }
  float current = PID_Calculate(&yaw->pid_inner, speed_target, speed_feedback)
                + feedforward;
  if (!isfinite(current)) {
    yaw_invalidate();
    return 0;
  }
  float limit = (float)MotorDriver_GetCommandLimit(s_yaw_motor_id);
  return (int16_t)fmaxf(-limit, fminf(current, limit));
}

/**
 * @brief 计算并显示云台倾斜角度补偿值
 *
 * 当云台pitch轴倾斜时，为了锁定地面目标，需要计算yaw角度补偿
 * 补偿量 = atan(sin(yaw_delta) * tan(pitch))
 *
 * 物理模型：
 * - 云台高度：30cm
 * - pitch角度：θ_p (正值向上)
 * - yaw旋转：θ_y
 * - 当yaw旋转时，由于pitch不为0，会产生垂直方向的指向偏移
 */
void GimbalController_CalculateAndDisplayCompensation(void) {
  static uint32_t last_update_time = 0;
  uint32_t current_time = BspTime_NowMs();

  // 限制更新频率，避免刷屏
  if (current_time - last_update_time < COMPENSATION_UPDATE_RATE_MS) {
    return;
  }
  last_update_time = current_time;

  // 获取pitch和yaw电机上下文
  MotorContext_t *pitch = MotorDriver_GetContext(s_pitch_motor_id);
  MotorContext_t *yaw = MotorDriver_GetContext(s_yaw_motor_id);

  if (!pitch || !pitch->angle_initialized || !yaw || !yaw->angle_initialized) {
    return;
  }
  /* 配置关闭时不计算或显示旧耦合模型，避免误认为仍在应用补偿。 */
  if (!pitch->config || !pitch->config->limits.gm6020.enable_yaw_pitch_compensation) {
    return;
  }

  // 计算当前pitch角度（弧度）
  const float max_encoder_pitch = 8192.0f;
  float pitch_angle_normalized = (float)pitch->angle_raw / max_encoder_pitch;
  float pitch_angle_rad = pitch_angle_normalized * (2.0f * (float)M_PI);
  float pitch_angle_deg = pitch_angle_rad * 180.0f / (float)M_PI;

  // 计算当前yaw角度（度）
  float max_encoder_yaw = (yaw->config->limits.gm6020.angle_max > 0.0f)
                            ? yaw->config->limits.gm6020.angle_max
                            : 8192.0f;
  float yaw_angle_deg = ((float)yaw->angle_raw / max_encoder_yaw) * 360.0f;

  // 计算目标水平距离（假设目标在地面上）
  // tan(pitch) = height / distance
  // distance = height / tan(pitch)
  float target_distance_cm = 0.0f;
  float pitch_for_distance = pitch_angle_rad;

  if (fabsf(pitch_for_distance) > 0.01f) { // 避免除零
    target_distance_cm = GIMBAL_HEIGHT_CM / tanf(pitch_for_distance);
  }

  // 计算yaw旋转1度时的补偿量
  // 当pitch不为0时，yaw旋转会导致俯仰角度的视觉偏移
  // 补偿公式：Δpitch ≈ sin(Δyaw) * tan(pitch)
  float yaw_delta_rad = 1.0f * (float)M_PI / 180.0f; // 1度
  float pitch_compensation_rad = sinf(yaw_delta_rad) * tanf(pitch_angle_rad);
  float pitch_compensation_deg = pitch_compensation_rad * 180.0f / (float)M_PI;

  // 计算yaw补偿（用于保持指向同一目标）
  // 当pitch角度改变时，如果要保持指向相同水平距离的点
  // yaw角度需要微调
  float yaw_compensation_per_pitch_deg = 0.0f;
  if (fabsf(cosf(pitch_angle_rad)) > 0.01f) {
    // Δyaw ≈ sin(pitch) * Δpitch / cos(pitch)
    yaw_compensation_per_pitch_deg = sinf(pitch_angle_rad) / cosf(pitch_angle_rad);
  }

  // 显示补偿信息
  USB_CDC_Printf("\r\n=== 云台角度补偿计算 ===\r\n");
  USB_CDC_Printf("云台高度: %.1f cm\r\n", GIMBAL_HEIGHT_CM);
  USB_CDC_Printf("当前Pitch角度: %.2f° (%.4f rad)\r\n", pitch_angle_deg, pitch_angle_rad);
  USB_CDC_Printf("当前Yaw角度: %.2f°\r\n", yaw_angle_deg);

  // 显示pitch控制模式
  if (s_last_cmd.vision_valid) {
    USB_CDC_Printf("Pitch控制: 遥控器手动 (自瞄时视觉不控制pitch)\r\n");
  } else {
    USB_CDC_Printf("Pitch控制: 遥控器手动 + Yaw-Pitch耦合补偿\r\n");
  }

  if (fabsf(pitch_for_distance) > 0.01f && target_distance_cm > 0.0f) {
    USB_CDC_Printf("目标水平距离: %.1f cm\r\n", target_distance_cm);
  } else if (target_distance_cm < 0.0f) {
    USB_CDC_Printf("目标水平距离: %.1f cm (目标在云台后方)\r\n", -target_distance_cm);
  } else {
    USB_CDC_Printf("目标水平距离: 无穷远 (pitch≈0°)\r\n");
  }

  USB_CDC_Printf("\r\n补偿值:\r\n");
  USB_CDC_Printf("- Yaw旋转1°时的Pitch耦合: %.4f° (%.6f rad)\r\n",
                 pitch_compensation_deg, pitch_compensation_rad);
  USB_CDC_Printf("- Pitch变化1°时需要的Yaw补偿系数: %.4f\r\n",
                 yaw_compensation_per_pitch_deg);

  // 计算实际补偿到编码器刻度
  float pitch_comp_ticks = pitch_compensation_rad * (max_encoder_pitch / (2.0f * (float)M_PI));
  USB_CDC_Printf("- Yaw旋转1°的Pitch补偿(编码器刻度): %.2f ticks\r\n", pitch_comp_ticks);

  USB_CDC_Printf("========================\r\n\r\n");

  // 记录到日志（CSV格式，便于后续分析）
  LOG_CSV(LOG_TAG_GIM, "COMPENSATION,%.2f,%.2f,%.1f,%.4f,%.4f,%.2f",
          pitch_angle_deg,
          yaw_angle_deg,
          target_distance_cm,
          pitch_compensation_deg,
          yaw_compensation_per_pitch_deg,
          pitch_comp_ticks);
}

// Application layer: Message subscription callbacks
static void process_gimbal_cmd(const MsgEvent *ev, void *user) {
  (void)user;
  if (ev->size == sizeof(GimbalCmd)) {
    memcpy(&s_last_cmd, ev->data, sizeof(GimbalCmd));

    /*
     * A command may arrive after the startup wait timed out. Capture late CAN
     * feedback here as well, but never energize a configured axis beforehand.
     */
    uint32_t now_ms = BspTime_NowMs();
    bool fresh = axis_feedback_fresh(s_yaw_motor_id, now_ms) &&
                 axis_feedback_fresh(s_pitch_motor_id, now_ms);
    if (!fresh || !s_last_cmd.enabled) {
      s_feedback_stable_seen = false;
      s_startup_position_captured = false;
    } else if (!s_feedback_stable_seen) {
      s_feedback_stable_seen = true;
      s_feedback_stable_since_ms = now_ms;
    }
    bool startup_ready = fresh && s_feedback_stable_seen &&
        (uint32_t)(now_ms - s_feedback_stable_since_ms) >= 100U &&
        (s_startup_position_captured || capture_startup_position());

    // Execute gimbal control when command arrives and both axes are safe.
    if (s_last_cmd.enabled && startup_ready) {
      bool use_vision_target = s_last_cmd.vision_valid;
      bool use_spin_hold = !use_vision_target && s_last_cmd.yaw_rate_memo > 0.5f;
      YawControlMode mode = use_vision_target ? YAW_CONTROL_VISION :
                            use_spin_hold ? YAW_CONTROL_SPIN : YAW_CONTROL_MANUAL;
      int16_t yaw_current = GimbalController_YawControlWithCompensation(
          s_last_cmd.yaw_rate, &s_last_sensor, mode);
      int16_t pitch_current = 0;
      if (s_yaw_reference.valid) {
        pitch_current = GimbalController_PitchControl(
            s_pitch_motor_id, s_last_cmd.pitch_rate, &s_last_sensor, use_vision_target);
      }
      /* 连续角度或任一轴前馈失败，同时停两轴并重新等待稳定反馈。 */
      if (!s_yaw_reference.valid) {
        uint8_t ids[2] = {s_pitch_motor_id, s_yaw_motor_id};
        for (unsigned i = 0U; i < 2U; ++i) {
          MotorContext_t *motor = MotorDriver_GetContext(ids[i]);
          if (!motor) continue;
          PID_Reset(&motor->pid_outer);
          PID_Reset(&motor->pid_inner);
          motor->angle_target = (float)motor->angle_raw;
        }
        s_coupling_history_valid = false;
        command_axis(s_pitch_motor_id, 0);
        command_axis(s_yaw_motor_id, 0);
        return;
      }
      // 计算并显示云台倾斜角度补偿（每100ms更新一次）
      GimbalController_CalculateAndDisplayCompensation();

      // Send motor currents (buffered, will be flushed by message center)
      command_axis(s_pitch_motor_id, pitch_current);
      command_axis(s_yaw_motor_id, yaw_current);
    } else {
      /* Forget old hold targets/integrals; reconnection captures the current angles. */
      uint8_t ids[2] = {s_pitch_motor_id, s_yaw_motor_id};
      for (unsigned i = 0U; i < 2U; ++i) {
        MotorContext_t *motor = MotorDriver_GetContext(ids[i]);
        if (!motor) continue;
        PID_Reset(&motor->pid_outer);
        PID_Reset(&motor->pid_inner);
        motor->angle_target = (float)motor->angle_raw;
      }
      s_yaw_reference.valid = false;
      s_yaw_mode_valid = false;
      s_coupling_history_valid = false;
      command_axis(s_pitch_motor_id, 0);
      command_axis(s_yaw_motor_id, 0);
    }

    // Gimbal position logging (20Hz rate limited in main.c)
    // Format: GIM,timestamp_ms,ENCODER,yaw_raw,pitch_raw,yaw_tgt,pitch_tgt
    // DISABLED to reduce log clutter - use only YAW_CSV for plotting
    // MotorContext_t *yaw_ctx = MotorDriver_GetContext(s_yaw_motor_id);
    // MotorContext_t *pitch_ctx = MotorDriver_GetContext(s_pitch_motor_id);

    // Support both full gimbal (yaw+pitch) and yaw-only configurations
    // if (yaw_ctx && pitch_ctx) {
    //   // Both yaw and pitch exist
    //   LOG_CSV(LOG_TAG_GIM, "ENCODER,%.2f,%.2f,%.2f,%.2f",
    //           (float)yaw_ctx->angle_raw,
    //           (float)pitch_ctx->angle_raw,
    //           yaw_ctx->angle_target,
    //           pitch_ctx->angle_target);
    // } else if (yaw_ctx) {
    //   // Yaw-only configuration (e.g., sentry)
    //   LOG_CSV(LOG_TAG_GIM, "ENCODER,%.2f,0.0,%.2f,0.0",
    //           (float)yaw_ctx->angle_raw,
    //           yaw_ctx->angle_target);
    // } else if (pitch_ctx) {
    //   // Pitch-only configuration (unlikely but handle it)
    //   LOG_CSV(LOG_TAG_GIM, "ENCODER,0.0,%.2f,0.0,%.2f",
    //           (float)pitch_ctx->angle_raw,
    //           pitch_ctx->angle_target);
    // }
  }
}

/* 从派发上下文中的电机反馈快照复制；IRQ不写这些兼容上下文。 */
static GimbalMonitorAxis monitor_axis(uint8_t id, unsigned index, bool active) {
  GimbalMonitorAxis axis = {0};
  MotorContext_t *motor = MotorDriver_GetContext(id);
  axis.motor_id = id;
  axis.command_status = s_monitor_status[index];
  axis.command_raw = s_monitor_commands[index];
  axis.command_unit = MotorService_GetCommandUnit(id);
  if (!motor || !motor->initialized) return axis;
  axis.flags = GIMBAL_MONITOR_PRESENT;
  if (motor->last_feedback_time != 0U) axis.flags |= GIMBAL_MONITOR_FEEDBACK_SEEN;
  if (axis_feedback_fresh(id, BspTime_NowMs())) axis.flags |= GIMBAL_MONITOR_FEEDBACK_FRESH;
  active = active && axis.command_status == ROBOT_STATUS_OK;
  if (active) axis.flags |= GIMBAL_MONITOR_CONTROL_ACTIVE;
  bool yaw = index == 0U;
  bool speed_only = yaw && motor->config && motor->config->yaw_control &&
                    motor->config->yaw_control->speed_loop_only;
  if (active && !speed_only) axis.flags |= GIMBAL_MONITOR_POSITION_ACTIVE;
  if (yaw && active && s_yaw_mode == YAW_CONTROL_SPIN)
    axis.flags |= GIMBAL_MONITOR_SPEED_IMU;
  axis.feedback_ms = motor->last_feedback_time;
  axis.encoder_raw = motor->angle_raw;
  axis.current_actual_raw = motor->feedback_current;
  axis.position_actual_ticks = yaw && s_yaw_reference.valid ?
      s_yaw_reference.position_ticks : (float)motor->angle_raw;
  axis.position_target_ticks = yaw && s_yaw_reference.valid ?
      s_yaw_reference.target_ticks : motor->angle_target;
  axis.speed_actual_rpm = motor->speed_rpm;
  /* PID_Reset保留target/actual，禁用时不能把它们冒充当前有效目标。 */
  axis.speed_target_rpm = active ? motor->pid_inner.target : 0.0f;
  axis.speed_loop_actual_rpm = active ? motor->pid_inner.actual : 0.0f;
  axis.speed_loop_dt_s = active ? motor->pid_inner.dt : 0.0f;
  return axis;
}

static void on_gimbal_cmd(const MsgEvent *ev, void *user) {
  if (!ev || ev->size != sizeof(GimbalCmd)) return;
  for (unsigned i = 0; i < 2U; ++i) {
    s_monitor_commands[i] = 0;
    s_monitor_status[i] = ROBOT_STATUS_NOT_READY;
  }
  process_gimbal_cmd(ev, user);
  uint32_t now = BspTime_NowMs();
  bool active = s_last_cmd.enabled && s_startup_position_captured && s_yaw_reference.valid;
  GimbalMonitorSnapshot next = {0};
  next.magic = 0x474D4F4EU;
  next.version = 1U;
  next.size_bytes = sizeof(next);
  next.callback_count = g_gimbal_monitor.callback_count + 1U;
  next.tick_ms = now;
  next.callback_dt_ms = g_gimbal_monitor.callback_count ? now - g_gimbal_monitor.tick_ms : 0U;
  next.enabled = s_last_cmd.enabled;
  next.startup_ready = s_startup_position_captured;
  next.yaw = monitor_axis(s_yaw_motor_id, 0U, active);
  next.pitch = monitor_axis(s_pitch_motor_id, 1U, active);
  /* Cortex-M4无数据缓存；编译屏障和volatile发布顺序供运行中SWD重读校验。 */
  uint32_t odd = g_gimbal_monitor.sequence + 1U;
  next.sequence = odd;
  g_gimbal_monitor.sequence = odd;
  atomic_signal_fence(memory_order_seq_cst);
  g_gimbal_monitor = next;
  atomic_signal_fence(memory_order_seq_cst);
  g_gimbal_monitor.sequence = odd + 1U;
}

static void on_imu_update(const MsgEvent *ev, void *user) {
  (void)user;
  if (ev->size == sizeof(SensorData)) {
    memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
  }
}

void GimbalApp_Init(void) {
  if (s_initialized) {
    return;
  }

  memset(&s_last_cmd, 0, sizeof(s_last_cmd));
  memset(&s_last_sensor, 0, sizeof(s_last_sensor));
  s_startup_position_captured = false;
  s_coupling_history_valid = false;
  s_last_coupling_yaw_angle = 0.0f;
  s_last_coupling_yaw_time_ms = 0U;

  LOG_INFO(LOG_TAG_GIM, "Gimbal init: searching for motors...");

  // Find gimbal motors by role (module layer handles config)
  uint8_t pitch_motors[1];
  uint8_t yaw_motors[1];

  if (MotorService_FindByRole(MOTOR_ROLE_GIMBAL_PITCH, pitch_motors, 1) > 0) {
    s_pitch_motor_id = pitch_motors[0];
    LOG_INFO(LOG_TAG_GIM, "Found pitch motor: id=%d", s_pitch_motor_id);
  } else {
    LOG_INFO(LOG_TAG_GIM, "No pitch motor configured");
  }

  if (MotorService_FindByRole(MOTOR_ROLE_GIMBAL_YAW, yaw_motors, 1) > 0) {
    s_yaw_motor_id = yaw_motors[0];
    LOG_INFO(LOG_TAG_GIM, "Found yaw motor: id=%d", s_yaw_motor_id);
  } else {
    LOG_ERROR(LOG_TAG_GIM, "ERROR: No yaw motor found!");
  }

  // Subscribe to messages
  (void)MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, on_gimbal_cmd, NULL);
  (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

  LOG_INFO(LOG_TAG_GIM, "Gimbal init complete: yaw=%d pitch=%d", s_yaw_motor_id, s_pitch_motor_id);
  s_initialized = true;
}

/**
 * @brief 等待两轴反馈后对齐：yaw锁存实测角，pitch使用配置初始角。
 * @note pitch初始角为负数时锁存实测角；反馈未就绪或pitch越界时不输出。
 */
void Gimbal_WaitForAlignment(void) {
  const uint32_t TIMEOUT_MS = 5000;        // 5 seconds timeout (reduced from 10s)
  const uint32_t CHECK_INTERVAL_MS = 5;    // 5ms check (reduced from 100ms → 20x faster)

  USB_CDC_Printf("[Gimbal] Waiting for startup position feedback...\r\n");

  // Enable zero-rate holding after each motor has captured its first feedback.
  GimbalCmd cmd = {.enabled = true,
                   .pitch_rate = 0.0f,
                   .yaw_rate = 0.0f,
                   .yaw_rate_memo = 0.0f,
                   .yaw_target_memo = 0.0f,
                   .vision_valid = false,
                   .vision_yaw_err_rad = 0.0f,
                   .vision_pitch_err_rad = 0.0f,
                   .vision_ts_ms = 0};

  uint32_t start_time = BspTime_NowMs();

  while (BspTime_NowMs() - start_time < TIMEOUT_MS) {
    /* Process queued motor feedback before any non-zero holding command. */
    MsgCenter_Dispatch();

    if (s_startup_position_captured || capture_startup_position()) {
      (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD, &cmd, sizeof(cmd));
      MsgCenter_Dispatch();
      USB_CDC_Printf("[Gimbal] Startup position captured!\r\n");
      return;
    }

    BspTime_DelayMs(CHECK_INTERVAL_MS);
  }

  USB_CDC_Printf(
      "[Gimbal] Warning: Startup feedback timeout; output remains zero until feedback arrives.\r\n");
}
