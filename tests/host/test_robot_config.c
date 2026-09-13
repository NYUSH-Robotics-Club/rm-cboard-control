/* Check both selectable robot configurations for IDs, channels, and safe modes. */
#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include "robot_config.h"
#include "dji_motor_protocol.h"

int main(void)
{
    const RobotConfig_t *robot = RobotConfig_Get();
    assert(robot && robot->motor_configs && robot->total_motor_count <= 16U);
    bool logical_id_used[16] = {false};
    for (uint8_t index = 0U; index < robot->total_motor_count; ++index) {
        const MotorConfig_t *motor = &robot->motor_configs[index];
        assert(motor->motor_id < 16U && !logical_id_used[motor->motor_id]);
        logical_id_used[motor->motor_id] = true;
        assert(motor->vendor == MOTOR_VENDOR_DJI);
        /* 当前车型均为已知DJI电调，报警标签必须与现有反馈地址吻合。 */
        assert(motor->offline_alarm_id >= 1U && motor->offline_alarm_id <= 8U);
        assert(motor->can_rx_id == motor->offline_alarm_id +
               (motor->type == MOTOR_TYPE_GM6020 ? 0x204U : 0x200U));
        assert(DjiMotor_CommandLimit(motor) > 0);
        assert(motor->control_mode == MOTOR_CONTROL_APPLICATION);
        assert(motor->can_channel < CAN_CHANNEL_COUNT);
        assert(motor->direction == 1 || motor->direction == -1);
        for (uint8_t other = 0U; other < index; ++other) {
            const MotorConfig_t *previous = &robot->motor_configs[other];
            assert(previous->can_channel != motor->can_channel ||
                   previous->can_rx_id != motor->can_rx_id);
        }
        if (motor->role == MOTOR_ROLE_GIMBAL_YAW) {
            assert(motor->limits.gm6020.initial_angle < 0.0f);
        }
        if (motor->role == MOTOR_ROLE_GIMBAL_PITCH &&
            motor->limits.gm6020.initial_angle >= 0.0f) {
            assert(motor->limits.gm6020.initial_angle >= motor->limits.gm6020.angle_min);
            assert(motor->limits.gm6020.initial_angle <= motor->limits.gm6020.angle_max);
        }
    }
#if defined(ROBOT_TYPE_infantry_standard)
    assert(robot->chassis_type == CHASSIS_TYPE_OMNI && robot->omni);
    assert(robot->omni->configured);
    const uint8_t wheel_ids[4] = {2, 1, 4, 3};
    for (unsigned i = 0; i < 4; ++i) {
        assert(robot->omni->wheels[i].motor_id == wheel_ids[i]);
        const MotorConfig_t *wheel = NULL;
        for (uint8_t j = 0; j < robot->total_motor_count; ++j)
            if (robot->motor_configs[j].motor_id == wheel_ids[i]) wheel = &robot->motor_configs[j];
        assert(wheel && wheel->role == MOTOR_ROLE_CHASSIS_DRIVE);
        assert(wheel->type == MOTOR_TYPE_M3508 && wheel->can_channel == CAN_CHANNEL_1);
        assert(wheel->can_rx_id == 0x200 + wheel_ids[i]);
        assert(wheel->can_tx_id == 0x200 && wheel->tx_slot == wheel_ids[i] - 1);
        /* 实车增益允许独立整定；这里只检查有限值、限幅和可选内环的合法性。 */
        assert(isfinite(wheel->pid_outer.kp) && wheel->pid_outer.kp >= 0.0f);
        assert(wheel->pid_outer.output_max > 0.0f);
        assert(wheel->pid_inner.output_max >= 0.0f);
    }
    const MotorConfig_t *yaw = NULL;
    const MotorConfig_t *pitch = NULL;
    for (uint8_t i = 0; i < robot->total_motor_count; ++i) {
        const MotorConfig_t *m = &robot->motor_configs[i];
        if (m->role == MOTOR_ROLE_GIMBAL_YAW) yaw = m;
        if (m->role == MOTOR_ROLE_GIMBAL_PITCH) pitch = m;
    }
    assert(yaw && pitch);
    assert(yaw->can_tx_id == 0x2FE && yaw->can_rx_id == 0x209 && yaw->tx_slot == 0);
    assert(yaw->protocol.dji.gm6020_mode == GM6020_COMMAND_CURRENT);
    assert(DjiMotor_CommandLimit(yaw) > 0 && DjiMotor_CommandLimit(yaw) <= 16384);
    assert(isfinite(yaw->pid_outer.kp) && yaw->pid_outer.kp >= 0.0f);
    assert(isfinite(yaw->pid_inner.kp) && yaw->pid_inner.kp >= 0.0f);
    assert(isfinite(yaw->pid_inner.ki) && yaw->pid_inner.ki >= 0.0f);
    assert(isfinite(yaw->pid_inner.kd) && yaw->pid_inner.kd >= 0.0f);
    assert(yaw->pid_inner.integral_max <= yaw->pid_inner.output_max);
    assert(pitch->can_tx_id == 0x1FF && pitch->tx_slot == 3);
    assert(pitch->protocol.dji.gm6020_mode == GM6020_COMMAND_VOLTAGE);
    assert(DjiMotor_CommandLimit(pitch) == 25000);
    assert(pitch->limits.gm6020.angle_min == 1566.0f);
    assert(pitch->limits.gm6020.angle_max == 2205.0f);
    assert(pitch->limits.gm6020.initial_angle == 1971.0f);
    assert(pitch->limits.gm6020.gravity_zero_angle == 1971.0f);
    assert(!pitch->limits.gm6020.enable_yaw_pitch_compensation);
#endif
    return 0;
}
