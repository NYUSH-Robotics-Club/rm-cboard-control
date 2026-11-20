/* Verify byte order, limit handling, and round-trip fields without hardware. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "benmo_motor_protocol.h"
#include "dm_motor_protocol.h"
#include "lk_motor_protocol.h"

static void test_dm(void)
{
    const DmMotorLimits limits = {-3.1415926f, 3.1415926f, -45.0f, 45.0f,
                                  -18.0f, 18.0f};
    uint8_t frame[8];
    assert(DmMotorProtocol_EncodeMit(&limits, 0.0f, 0.0f, 0.0f, 0.0f,
                                     0.0f, frame));
    assert(frame[0] == 0x80U || frame[0] == 0x7FU);

    uint8_t state[8] = {0};
    DmMotorProtocol_BuildStateCommand(0xFCU, state);
    for (unsigned int index = 0U; index < 7U; ++index) assert(state[index] == 0xFFU);
    assert(state[7] == 0xFCU);

    const uint8_t feedback_frame[8] = {0x10U, 0x80U, 0x00U, 0x80U,
                                       0x08U, 0x00U, 40U, 41U};
    DmMotorFeedback feedback;
    assert(DmMotorProtocol_DecodeFeedback(&limits, feedback_frame, &feedback));
    assert(feedback.state == 1U);
    assert(fabsf(feedback.position) < 0.01f);
    assert(fabsf(feedback.velocity) < 0.1f);
    assert(fabsf(feedback.torque) < 0.1f);
}

static void test_lk(void)
{
    const int16_t commands[4] = {0x1234, -2, 0, 2000};
    uint8_t frame[8];
    assert(LkMotorProtocol_BuildBroadcastCurrent(commands, frame));
    assert(frame[0] == 0x34U && frame[1] == 0x12U);
    assert(frame[2] == 0xFEU && frame[3] == 0xFFU);

    const uint8_t feedback_frame[8] = {0U, 55U, 0x34U, 0x12U,
                                       0xFEU, 0xFFU, 0xCDU, 0xABU};
    LkMotorFeedback feedback;
    assert(LkMotorProtocol_DecodeFeedback(feedback_frame, &feedback));
    assert(feedback.temperature == 55U);
    assert(feedback.current == 0x1234);
    assert(feedback.speed_deg_s == -2);
    assert(feedback.encoder == 0xABCDU);
}

static void test_benmo(void)
{
    uint8_t config[8] = {0};
    assert(BenmoMotorProtocol_SetSlot(5U, 0x02U, config));
    assert(config[4] == 0x02U);

    const int16_t commands[4] = {0x1234, -2, 0, 300};
    uint8_t frame[8];
    assert(BenmoMotorProtocol_BuildGroup(commands, frame));
    assert(frame[0] == 0x12U && frame[1] == 0x34U);
    assert(frame[2] == 0xFFU && frame[3] == 0xFEU);

    BenmoMotorFeedback feedback;
    assert(BenmoMotorProtocol_DecodeFeedback(frame, &feedback));
    assert(feedback.velocity_raw == 0x1234);
    assert(feedback.current_raw == -2);
}

int main(void)
{
    test_dm();
    test_lk();
    test_benmo();
    return 0;
}
