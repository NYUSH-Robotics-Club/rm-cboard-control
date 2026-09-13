/* Check motor vendor selection, control modes, and safe unsupported results. */
#include <assert.h>
#include "message_center.h"
#include "motor_service.h"
#include "robot_config.h"

static int16_t s_last_current = 0;
static unsigned int s_flush_count = 0U;
static unsigned int s_can_frame_count = 0U;

static MotorConfig_t s_motors[] = {{
    .motor_id = 0,
    .vendor = MOTOR_VENDOR_DJI,
    .type = MOTOR_TYPE_M3508,
    .role = MOTOR_ROLE_CHASSIS_DRIVE,
    .control_mode = MOTOR_CONTROL_APPLICATION,
    .direction = 1
}};

static const RobotConfig_t s_robot = {
    .name = "host-test",
    .chassis_type = CHASSIS_TYPE_MECANUM,
    .chassis_motor_count = 1,
    .motor_configs = s_motors,
    .total_motor_count = 1
};

const RobotConfig_t *RobotConfig_Get(void)
{
    return &s_robot;
}

static RobotStatus mock_validate(const MotorConfig_t *config)
{
    return config ? ROBOT_STATUS_OK : ROBOT_STATUS_INVALID_ARGUMENT;
}

static RobotStatus mock_command(uint8_t motor_id, int16_t current)
{
    assert(motor_id == 0U);
    s_last_current = current;
    return ROBOT_STATUS_OK;
}

static RobotStatus mock_compute(uint8_t motor_id,
                                MotorControlMode_e mode,
                                float setpoint,
                                int16_t application_current,
                                int16_t *output)
{
    assert(motor_id == 0U);
    if (mode == MOTOR_CONTROL_DISABLED) {
        *output = 0;
        return ROBOT_STATUS_NOT_READY;
    }
    *output = mode == MOTOR_CONTROL_APPLICATION
                  ? application_current
                  : (int16_t)(setpoint * 2.0f);
    return ROBOT_STATUS_OK;
}

static void mock_flush(void)
{
    ++s_flush_count;
}

static void mock_on_can_frame(const CanRxFrame *frame)
{
    assert(frame && frame->dlc == 8U);
    ++s_can_frame_count;
}

static const MotorAdapterOps s_adapter = {
    .vendor = MOTOR_VENDOR_DJI,
    .name = "mock",
    .implemented = true,
    .validate = mock_validate,
    .command_current = mock_command,
    .compute_current = mock_compute,
    .snapshot = 0,
    .on_can_frame = mock_on_can_frame,
    .reset_control = 0,
    .flush = mock_flush
};

const MotorAdapterOps *MotorAdapter_Get(MotorVendor_e vendor)
{
    return vendor == MOTOR_VENDOR_DJI ? &s_adapter : 0;
}
const MotorAdapterOps *DjiMotorAdapter_Get(void) { return &s_adapter; }
const MotorAdapterOps *DmMotorAdapter_Get(void) { return 0; }
const MotorAdapterOps *BenmoMotorAdapter_Get(void) { return 0; }
const MotorAdapterOps *LkMotorAdapter_Get(void) { return 0; }

int main(void)
{
    /* 单位来自实际配置；未知模式不能冒充电流。 */
    assert(MotorService_GetCommandUnit(0) == MOTOR_COMMAND_UNIT_CURRENT_COUNTS);
    assert(MotorService_GetCommandUnit(99) == MOTOR_COMMAND_UNIT_UNKNOWN);
    s_motors[0].type = MOTOR_TYPE_GM6020;
    assert(MotorService_GetCommandUnit(0) == MOTOR_COMMAND_UNIT_VOLTAGE_COUNTS);
    s_motors[0].protocol.dji.gm6020_mode = GM6020_COMMAND_CURRENT;
    assert(MotorService_GetCommandUnit(0) == MOTOR_COMMAND_UNIT_CURRENT_COUNTS);
    s_motors[0].protocol.dji.gm6020_mode = (GM6020CommandMode_e)99;
    assert(MotorService_GetCommandUnit(0) == MOTOR_COMMAND_UNIT_UNKNOWN);
    s_motors[0].protocol.dji.gm6020_mode = GM6020_COMMAND_VOLTAGE;
    s_motors[0].type = MOTOR_TYPE_M3508;

    MsgEvent queue[8];
    MsgCenter_Init(queue, 8U);

    assert(MotorService_CommandConfigured(0U, 10.0f, 123) == ROBOT_STATUS_OK);
    assert(s_last_current == 123);
    MsgCenter_Dispatch();
    assert(s_flush_count == 1U);

    CanRxFrame raw_frame = {
        .channel = CAN_CHANNEL_1, .std_id = 0x201U, .dlc = 8U,
        .is_standard_frame = false, .is_data_frame = true
    };
    assert(MsgCenter_Publish(TOPIC_CAN_RX, &raw_frame, sizeof(raw_frame)) == 0);
    MsgCenter_Dispatch();
    assert(s_can_frame_count == 0U);
    raw_frame.is_standard_frame = true;
    assert(MsgCenter_Publish(TOPIC_CAN_RX, &raw_frame, sizeof(raw_frame)) == 0);
    MsgCenter_Dispatch();
    assert(s_can_frame_count == 1U);

    assert(MotorService_SetControlMode(0U, MOTOR_CONTROL_SPEED) == ROBOT_STATUS_OK);
    assert(MotorService_CommandConfigured(0U, 10.0f, 123) == ROBOT_STATUS_OK);
    assert(s_last_current == 20);
    assert(MotorService_CommandCurrent(0U, 99) == ROBOT_STATUS_MODE_MISMATCH);
    assert(s_last_current == 0);

    assert(MotorService_SetControlMode(0U, MOTOR_CONTROL_DISABLED) == ROBOT_STATUS_OK);
    assert(MotorService_CommandConfigured(0U, 10.0f, 123) == ROBOT_STATUS_NOT_READY);
    assert(s_last_current == 0);
    return 0;
}
