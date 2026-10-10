/* Exercise the active USB receive path through the vision bridge and command router. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "command_router.h"
#include "legacy_vision_bridge.h"
#include "message_center.h"
#include "seasky_protocol.h"
#include "vision_comm.h"
#include "vision_messages.h"

static VisionTargetMessage last_target;
static unsigned target_count;

static void capture_target(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    assert(event->size == sizeof(last_target));
    memcpy(&last_target, event->data, sizeof(last_target));
    ++target_count;
}

static void dispatch_vision(void)
{
    /* The legacy bridge publishes its target during the first dispatch. */
    MsgCenter_Dispatch();
    MsgCenter_Dispatch();
}

int main(void)
{
    MsgEvent queue[16];
    MsgCenter_Init(queue, 16);
    assert(LegacyVisionBridge_Init() == ROBOT_STATUS_OK);
    assert(MsgCenter_Subscribe(TOPIC_VISION_TARGET, capture_target, NULL) == 0);

    /* nyu-vision's packed VisionToGimbal is 2-byte "SP", mode, six floats,
     * CRC16: 29 bytes. The active USB callback accepts at most 18 bytes. */
    uint8_t sp_packet[29] = {'S', 'P', 1};
    VisionComm_RxCallback(sp_packet, sizeof(sp_packet));
    dispatch_vision();
    assert(target_count == 0);

    float error_rad[2] = {-0.05f, 0.10f}; /* Seasky order: pitch, then yaw. */
    uint8_t packet[18];
    uint16_t length = 0;
    uint16_t flags = AUTO_AIM | (TARGET_CONVERGING << 2) | (INFANTRY3 << 4);
    get_protocol_send_data(0x0001, flags, error_rad, 2, packet, &length);
    assert(length == sizeof(packet));

    uint8_t damaged[18];
    memcpy(damaged, packet, sizeof(damaged));
    damaged[8] ^= 1;
    VisionComm_RxCallback(damaged, sizeof(damaged));
    dispatch_vision();
    assert(target_count == 0);

    VisionComm_RxCallback(packet, sizeof(packet));
    dispatch_vision();
    assert(target_count == 1);
    assert(last_target.valid && last_target.sequence == 1);
    assert(last_target.target_state == VISION_TARGET_CONVERGING);
    assert(fabsf(last_target.pitch_error_rad + 0.05f) < 1e-6f);
    assert(fabsf(last_target.yaw_error_rad - 0.10f) < 1e-6f);

    CommandRouter router;
    CommandRouterInput input = {0};
    CommandRouterOutput output;
    CommandRouter_Init(&router);
    input.remote_online = true;
    input.remote.rc.s[0] = input.remote.rc.s[1] = RC_SW_DOWN;
    input.vision = last_target;
    input.vision_updated = true;
    assert(CommandRouter_Route(&router, &input, 100, &output) == ROBOT_STATUS_OK);
    assert(output.gimbal.enabled && output.gimbal.vision_valid);
    assert(output.gimbal.vision_frame == 1);
    assert(fabsf(output.gimbal.vision_yaw_err_rad - 0.10f) < 1e-6f);
    assert(fabsf(output.gimbal.vision_pitch_err_rad + 0.05f) < 1e-6f);

    input.vision_updated = false;
    assert(CommandRouter_Route(&router, &input, 120, &output) == ROBOT_STATUS_OK);
    assert(output.gimbal.vision_valid && output.gimbal.vision_frame == 1);
    assert(CommandRouter_Route(&router, &input, 121, &output) == ROBOT_STATUS_OK);
    assert(!output.gimbal.vision_valid);

    input.vision_updated = true;
    input.vision = last_target;
    input.vision.yaw_error_rad = NAN;
    assert(CommandRouter_Route(&router, &input, 122, &output) == ROBOT_STATUS_OK);
    assert(!output.gimbal.vision_valid);
    input.vision = last_target;
    input.remote_online = false;
    assert(CommandRouter_Route(&router, &input, 123, &output) == ROBOT_STATUS_NOT_READY);
    assert(!output.gimbal.enabled);

    puts("vision link simulation: PASS (SP rejected; Seasky routed; CRC, timeout, invalid and RC loss safe)");
    return 0;
}
