/* Exercise adapter validation, startup sequencing, CAN writes, and snapshots. */
#include <assert.h>
#include <string.h>
#include "bsp_can.h"
#include "bsp_time.h"
#include "motor_adapter.h"

static uint32_t s_now_us;
static unsigned int s_write_count;
static uint16_t s_last_identifier;
static uint8_t s_last_data[8];

uint32_t BspTime_NowUs(void) { s_now_us += 1000U; return s_now_us; }
uint32_t BspTime_NowMs(void) { return s_now_us / 1000U; }
void BspTime_DelayMs(uint32_t milliseconds) { s_now_us += milliseconds * 1000U; }

bool BspCan_Write(BspCanChannel channel, uint16_t standard_id,
                  const uint8_t *data, uint8_t length)
{
    assert(channel == BSP_CAN_CHANNEL_1 || channel == BSP_CAN_CHANNEL_2);
    assert(data && length == 8U);
    ++s_write_count;
    s_last_identifier = standard_id;
    memcpy(s_last_data, data, sizeof(s_last_data));
    return true;
}

static void test_dm(void)
{
    const MotorConfig_t motor = {
        .motor_id = 1U, .vendor = MOTOR_VENDOR_DM,
        .type = MOTOR_TYPE_VENDOR_DEFINED, .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .control_mode = MOTOR_CONTROL_SPEED, .can_channel = CAN_CHANNEL_1,
        .can_rx_id = 0x11U, .can_tx_id = 0x01U, .direction = 1,
        .protocol.dm = {-3.14f, 3.14f, -45.0f, 45.0f, -18.0f, 18.0f},
        .pid_outer = {1.0f, 0.0f, 0.0f, 1000.0f, 0.0f}
    };
    const RobotConfig_t robot = {.name = "dm", .motor_configs = &motor,
                                 .total_motor_count = 1U};
    const MotorAdapterOps *adapter = DmMotorAdapter_Get();
    assert(adapter->validate(&motor) == ROBOT_STATUS_OK);
    assert(adapter->init(&robot) == ROBOT_STATUS_OK);
    assert(adapter->command_current(1U, 1000) == ROBOT_STATUS_OK);
    for (unsigned int index = 0U; index < 10U; ++index) adapter->flush();
    assert(s_last_identifier == 0x01U && s_last_data[7] == 0xFCU);
    assert(adapter->command_current(1U, 1000) == ROBOT_STATUS_OK);
    adapter->flush();
    assert(s_last_identifier == 0x01U && s_last_data[7] != 0xFCU);

    const CanRxFrame frame = {.channel = CAN_CHANNEL_1, .std_id = 0x11U,
                              .dlc = 8U,
                              .data = {0x10U, 0x80U, 0x00U, 0x80U,
                                       0x08U, 0x00U, 30U, 31U},
                              .tick_ms = 42U};
    adapter->on_can_frame(&frame);
    MotorSnapshot snapshot;
    assert(adapter->snapshot(1U, &snapshot) == ROBOT_STATUS_OK);
    assert(snapshot.feedback_valid && snapshot.feedback_timestamp_ms == 42U);

    MotorConfig_t duplicates[2];
    duplicates[0] = motor;
    duplicates[1] = motor;
    duplicates[1].motor_id = 4U;
    duplicates[1].can_rx_id = 0x12U;
    const RobotConfig_t duplicate_robot = {
        .name = "dm-duplicate-tx", .motor_configs = duplicates,
        .total_motor_count = 2U
    };
    assert(adapter->init(&duplicate_robot) == ROBOT_STATUS_INVALID_ARGUMENT);
}

static void test_lk(void)
{
    const MotorConfig_t motor = {
        .motor_id = 2U, .vendor = MOTOR_VENDOR_LK,
        .type = MOTOR_TYPE_VENDOR_DEFINED, .role = MOTOR_ROLE_CHASSIS_DRIVE,
        .control_mode = MOTOR_CONTROL_OPEN_LOOP_CURRENT,
        .can_channel = CAN_CHANNEL_2, .can_rx_id = 0x143U,
        .can_tx_id = 0x280U, .tx_slot = 2U, .direction = 1,
        .protocol.lk = {.command_limit = 2000,
                        .encoder_counts_per_rev = 65536U}
    };
    const RobotConfig_t robot = {.name = "lk", .motor_configs = &motor,
                                 .total_motor_count = 1U};
    const MotorAdapterOps *adapter = LkMotorAdapter_Get();
    assert(adapter->validate(&motor) == ROBOT_STATUS_OK);
    assert(adapter->init(&robot) == ROBOT_STATUS_OK);
    assert(adapter->command_current(2U, 3000) == ROBOT_STATUS_OK);
    adapter->flush();
    assert(s_last_identifier == 0x280U);
    assert(s_last_data[4] == 0xD0U && s_last_data[5] == 0x07U);

    const CanRxFrame frame = {.channel = CAN_CHANNEL_2, .std_id = 0x143U,
                              .dlc = 8U,
                              .data = {0U, 50U, 10U, 0U, 20U, 0U, 0U, 0x80U},
                              .tick_ms = 50U};
    adapter->on_can_frame(&frame);
    MotorSnapshot snapshot;
    assert(adapter->snapshot(2U, &snapshot) == ROBOT_STATUS_OK);
    assert(snapshot.position > 179.9f && snapshot.position < 180.1f);

    MotorConfig_t duplicates[2];
    duplicates[0] = motor;
    duplicates[1] = motor;
    duplicates[1].motor_id = 5U;
    const RobotConfig_t duplicate_robot = {
        .name = "lk-duplicate-slot", .motor_configs = duplicates,
        .total_motor_count = 2U
    };
    assert(adapter->init(&duplicate_robot) == ROBOT_STATUS_INVALID_ARGUMENT);
}

static void test_benmo(void)
{
    const MotorConfig_t motor = {
        .motor_id = 3U, .vendor = MOTOR_VENDOR_BENMO,
        .type = MOTOR_TYPE_VENDOR_DEFINED, .role = MOTOR_ROLE_SHOOTER_FEED,
        .control_mode = MOTOR_CONTROL_OPEN_LOOP_CURRENT,
        .can_channel = CAN_CHANNEL_1, .can_rx_id = 0x97U,
        .can_tx_id = 0x32U, .tx_slot = 0U, .direction = 1,
        .protocol.benmo = {.motor_address = 1U,
                           .command_mode = BENMO_CONTROL_RAW_CURRENT,
                           .feedback_mode = 0x0AU,
                           .command_limit = 1000}
    };
    const RobotConfig_t robot = {.name = "benmo", .motor_configs = &motor,
                                 .total_motor_count = 1U};
    const MotorAdapterOps *adapter = BenmoMotorAdapter_Get();
    assert(adapter->validate(&motor) == ROBOT_STATUS_OK);
    assert(adapter->init(&robot) == ROBOT_STATUS_OK);
    for (unsigned int index = 0U; index < 30U; ++index) adapter->flush();
    assert(adapter->command_current(3U, 500) == ROBOT_STATUS_OK);
    adapter->flush();
    assert(s_last_identifier == 0x32U);
    assert(s_last_data[0] == 0x01U && s_last_data[1] == 0xF4U);

    const CanRxFrame frame = {.channel = CAN_CHANNEL_1, .std_id = 0x97U,
                              .dlc = 8U,
                              .data = {0U, 10U, 0U, 20U, 0x12U, 0x34U, 0U, 1U},
                              .tick_ms = 60U};
    adapter->on_can_frame(&frame);
    MotorSnapshot snapshot;
    assert(adapter->snapshot(3U, &snapshot) == ROBOT_STATUS_OK);
    assert(snapshot.position == 4660.0f && snapshot.speed == 10.0f);

    MotorConfig_t duplicates[2];
    duplicates[0] = motor;
    duplicates[1] = motor;
    duplicates[1].motor_id = 6U;
    duplicates[1].can_rx_id = 0x98U;
    const RobotConfig_t duplicate_robot = {
        .name = "benmo-duplicate-address", .motor_configs = duplicates,
        .total_motor_count = 2U
    };
    assert(adapter->init(&duplicate_robot) == ROBOT_STATUS_INVALID_ARGUMENT);
}

int main(void)
{
    test_dm();
    test_lk();
    test_benmo();
    assert(s_write_count > 0U);
    return 0;
}
