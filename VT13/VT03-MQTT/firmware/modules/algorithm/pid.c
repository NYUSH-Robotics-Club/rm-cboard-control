/*
 * 实现比例、积分、微分控制计算。
 * 输出和积分都会按初始化时给出的上限裁剪。
 */
#include "pid.h"
#include "bsp_time.h"
#include <stddef.h>
#include <math.h>

static uint32_t get_time_us(void) {
  return BspTime_NowUs();
}

/**
 * @brief Initialize PID controller with specified parameters
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param output_max Maximum output value (saturation limit)
 * @param integral_max Maximum integral value (anti-windup limit)
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->output_max = output_max;
  pid->integral_max = integral_max;
  
  pid->target = 0.0f;
  pid->error[0] = 0.0f;
  pid->error[1] = 0.0f;
  pid->error[2] = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
  
  // Initialize new fields
  pid->last_measure = 0.0f;
  pid->last_output = 0.0f;
  pid->last_dout = 0.0f;
  pid->pout = 0.0f;
  pid->iout = 0.0f;
  pid->dout = 0.0f;
  pid->iterm = 0.0f;
  pid->dt = 0.001f;  // Default 1ms
  pid->last_time_us = get_time_us();
  pid->update_divider = 0U;
  pid->skip_remaining = 0U;
  
  // Reduced filter RC values for faster response (trades smoothness for speed)
  pid->output_lpf_rc = 0.002f;     // Reduced from 0.01 → 5x faster response
  pid->derivative_lpf_rc = 0.005f; // Reduced from 0.01 → 2x faster response
}

/**
 * @brief Calculate PID controller output (improved version based on pid_new.c)
 * @param pid Pointer to PID controller structure
 * @param target Target value (setpoint)
 * @param actual Current actual value (feedback)
 * @return PID controller output
 * 
 * Improvements:
 * - Time-based calculation with dt
 * - Trapezoid integration for smoother integral
 * - Derivative on measurement (avoids derivative kick)
 * - Low-pass filtering on derivative and output
 * - Improved integral limiting
 */
float PID_Calculate(PID_Controller *pid, float target, float actual)
{
  // Calculate time delta
  uint32_t current_time_us = get_time_us();
  pid->dt = (current_time_us - pid->last_time_us) / 1000000.0f;
  pid->last_time_us = current_time_us;
  
  // Prevent division by zero or unreasonable dt
  if (pid->dt <= 0.0f || pid->dt > 1.0f) {
    pid->dt = 0.001f;
  }
  
  pid->target = target;
  pid->actual = actual;
  
  // Update error history
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->error[0] = pid->target - pid->actual;
  
  // Proportional term
  pid->pout = pid->Kp * pid->error[0];
  
  // Integral term - using trapezoid integration for smoother response
  pid->iterm = pid->Ki * ((pid->error[0] + pid->error[1]) / 2.0f) * pid->dt;
  
  // Derivative term - based on measurement to avoid derivative kick
  pid->dout = pid->Kd * (pid->last_measure - pid->actual) / pid->dt;
  
  // Apply derivative low-pass filter to reduce noise
  pid->dout = pid->dout * pid->dt / (pid->derivative_lpf_rc + pid->dt) +
              pid->last_dout * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dt);
  
  // Improved integral limiting - prevent windup
  float intended_iout = pid->iout + pid->iterm;
  float intended_output = pid->pout + intended_iout + pid->dout;
  
  // If output would saturate and integral is increasing the error, stop integrating
  if (fabsf(intended_output) > pid->output_max) {
    if (pid->error[0] * pid->iout > 0) {
      pid->iterm = 0;
    }
  }
  
  // Apply integral limits
  if (intended_iout > pid->integral_max) {
    pid->iterm = 0;
    pid->iout = pid->integral_max;
  } else if (intended_iout < -pid->integral_max) {
    pid->iterm = 0;
    pid->iout = -pid->integral_max;
  } else {
    pid->iout += pid->iterm;
  }
  
  // Calculate total output
  pid->output = pid->pout + pid->iout + pid->dout;
  
  // Apply output low-pass filter
  pid->output = pid->output * pid->dt / (pid->output_lpf_rc + pid->dt) +
                pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + pid->dt);
  
  // Output saturation
  if (pid->output > pid->output_max) {
    pid->output = pid->output_max;
  } else if (pid->output < -pid->output_max) {
    pid->output = -pid->output_max;
  }
  
  // Save states for next iteration
  pid->last_measure = pid->actual;
  pid->last_output = pid->output;
  pid->last_dout = pid->dout;
  
  return pid->output;
}


/**
 * @brief Calculate PID controller output for RPM control (improved version)
 * @param pid Pointer to PID controller structure
 * @param target_rpm Target RPM value
 * @param actual_rpm Current actual RPM value
 * @return PID controller output
 */
float PID_RPM_Calculate(PID_Controller *pid, float target_rpm, float actual_rpm)
{
    // Calculate time delta
    uint32_t current_time_us = get_time_us();
    pid->dt = (current_time_us - pid->last_time_us) / 1000000.0f;
    pid->last_time_us = current_time_us;
    
    // Prevent division by zero or unreasonable dt
    if (pid->dt <= 0.0f || pid->dt > 1.0f) {
        pid->dt = 0.001f;
    }
    
    pid->target = target_rpm;
    pid->actual = actual_rpm;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target - pid->actual;

    // Proportional term
    pid->pout = pid->Kp * pid->error[0];
    
    // Integral term with trapezoid integration
    pid->iterm = pid->Ki * ((pid->error[0] + pid->error[1]) / 2.0f) * pid->dt;
    
    // Derivative on measurement
    pid->dout = pid->Kd * (pid->last_measure - pid->actual) / pid->dt;
    
    // Apply derivative filter
    pid->dout = pid->dout * pid->dt / (pid->derivative_lpf_rc + pid->dt) +
                pid->last_dout * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dt);
    
    // Improved integral limiting
    float intended_iout = pid->iout + pid->iterm;
    float intended_output = pid->pout + intended_iout + pid->dout;
    
    if (fabsf(intended_output) > pid->output_max) {
        if (pid->error[0] * pid->iout > 0) {
            pid->iterm = 0;
        }
    }
    
    if (intended_iout > pid->integral_max) {
        pid->iterm = 0;
        pid->iout = pid->integral_max;
    } else if (intended_iout < -pid->integral_max) {
        pid->iterm = 0;
        pid->iout = -pid->integral_max;
    } else {
        pid->iout += pid->iterm;
    }
    
    // Calculate output
    pid->output = pid->pout + pid->iout + pid->dout;
    
    // Apply output filter
    pid->output = pid->output * pid->dt / (pid->output_lpf_rc + pid->dt) +
                  pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + pid->dt);
    
    // Output saturation
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }
    
    // Save states
    pid->last_measure = pid->actual;
    pid->last_output = pid->output;
    pid->last_dout = pid->dout;

    return pid->output;
}

float PID_CalculateDivided(PID_Controller *pid,
                            float target, float actual,
                            uint16_t divider)
  {
      if (pid == NULL) {
          return 0.0f;
      }

      if (divider == 0U) {
          divider = 1U;
      }

      /* 调整比例后立即采样，不沿用旧比例的剩余等待次数。 */
      if (pid->update_divider != divider) {
          pid->update_divider = divider;
          pid->skip_remaining = 0U;
      }

      if (pid->skip_remaining > 0U) {
          --pid->skip_remaining;

          /* 保持输出及历史状态，让下次dt覆盖完整外环间隔。 */
          return pid->output;
      }

      pid->skip_remaining = divider - 1U;
      return PID_Calculate(pid, target, actual);
  }

/**
 * @brief Reset PID controller state
 * @param pid Pointer to PID controller structure
 */
void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output = 0.0f;
    
    // Reset new fields
    pid->last_measure = 0.0f;
    pid->last_output = 0.0f;
    pid->last_dout = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->iterm = 0.0f;
    pid->last_time_us = get_time_us();
    pid->update_divider = 0U;
    pid->skip_remaining = 0U;
}
