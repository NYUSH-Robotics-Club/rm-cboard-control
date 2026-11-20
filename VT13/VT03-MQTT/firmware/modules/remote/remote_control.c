/* Publish the copied nyush-rm-control decoder's output using existing robot messages.
 * UART reception and DBUS decoding are implemented in the vendored source files. */
#include "remote_control.h"
#include "nyush_remote.h"
#include "bsp_rc.h"
#include "bsp_time.h"
#include "bsp_critical.h"
#include "message_center.h"
#include <string.h>
RC_ctrl_t rc_ctrl;
static volatile uint32_t rc_frame_count;
static uint8_t last_sbus_frame[RC_FRAME_LENGTH];
static uint32_t s_daemon_tick_ms;
/* A healthy physical receiver always owns TOPIC_RC_UPDATE. The VR adapter
 * never refreshes a command on timeout, so the existing controller watchdog
 * remains authoritative. Channel 3 uses the existing normalized forward units. */
#define VR_RC_TIMEOUT_MS 200U
#define VR_FRAME_BYTES 39U
#define VR_FORWARD_MAX 100
static volatile uint32_t last_dbus_ms;
static volatile uint8_t dbus_seen;
static uint8_t vr_frame[VR_FRAME_BYTES], vr_len, vr_released, vr_seen;
static uint16_t vr_seq;
static uint32_t vr_last_ms;
/* Read-only SWD diagnostics; control does not consume these counters. */
volatile uint32_t vr_uart_valid_count;
volatile uint32_t vr_uart_publish_count;
volatile uint32_t vr_uart_last_axes;
volatile uint32_t vr_uart_mqtt_count;
volatile uint32_t vr_uart_legacy_count;
volatile uint32_t vr_uart_last_command;
volatile uint32_t vr_uart_last_sequence;
volatile uint32_t vr_uart_last_valid_ms;
const volatile uint32_t vr_mqtt_firmware_tag = 0x20261120U;
/* VRD3 adds only a chassis dial input. Channel 0 (gimbal yaw) stays zero. */
const volatile uint32_t vr_dual_firmware_tag = 0x56524433U;
volatile int32_t vr_uart_last_rotation;

static uint16_t vr_crc(const uint8_t *data, uint8_t size, uint16_t value, uint16_t poly)
{
    while (size--) {
        value ^= *data++;
        for (unsigned i = 0; i < 8U; ++i)
            value = (value >> 1) ^ ((value & 1U) ? poly : 0U);
    }
    return value;
}

static void vr_accept(uint32_t now_ms)
{
    const uint8_t *p = &vr_frame[7];
    const uint8_t dual = memcmp(p, "VRD3", 4) == 0;
    if ((!dual && memcmp(p, "VRD2", 4) != 0) || p[6] > 1U || p[7] > 1U || (p[7] && p[6])) return;
    for (unsigned i = dual ? 14U : 12U; i < 30; ++i) if (p[i] != 0U) return;
    uint16_t seq = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    int forward = (int16_t)((uint16_t)p[8] | ((uint16_t)p[9] << 8));
    int right = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));
    int rotation = dual ? (int16_t)((uint16_t)p[12] | ((uint16_t)p[13] << 8)) : 0;
    if (forward < -VR_FORWARD_MAX || forward > VR_FORWARD_MAX ||
        right < -VR_FORWARD_MAX || right > VR_FORWARD_MAX ||
        rotation < -VR_FORWARD_MAX || rotation > VR_FORWARD_MAX ||
        (!p[6] && (forward != 0 || right != 0 || rotation != 0)) ||
        (p[7] && (forward != 0 || right != 0 || rotation != 0))) return;
    ++vr_uart_valid_count;
    vr_uart_last_command = (uint16_t)vr_frame[5] | ((uint16_t)vr_frame[6] << 8);
    vr_uart_mqtt_count += vr_uart_last_command == 0x0311U;
    vr_uart_legacy_count += vr_uart_last_command == 0x0302U;
    vr_uart_last_sequence = seq;
    vr_uart_last_valid_ms = now_ms;
    if (p[7]) return; /* Diagnostic-only frames never publish a control message. */
    if (dbus_seen && (uint32_t)(now_ms - last_dbus_ms) <= VR_RC_TIMEOUT_MS) {
        vr_seen = 0U;
        vr_released = 0U;
        return;
    }
    if (vr_seen && (uint32_t)(now_ms - vr_last_ms) > VR_RC_TIMEOUT_MS) {
        vr_seen = 0U;
        vr_released = 0U;
    }
    if (!vr_seen && p[6]) return;
    uint16_t delta = (uint16_t)(seq - vr_seq);
    if (vr_seen && (delta == 0U || delta > 32767U)) return;
    vr_seen = 1U;
    vr_seq = seq;
    vr_last_ms = now_ms;
    if (!p[6]) vr_released = 1U;
    if (!vr_released) return;
    RC_ctrl_t vr = {0};
    vr.rc.s[0] = RC_SW_DOWN;
    vr.rc.s[1] = RC_SW_DOWN;
    vr.rc.ch[3] = (int16_t)forward;
    /* The existing router negates channel 2 to produce left-positive vy. */
    vr.rc.ch[2] = (int16_t)right;
    /* Original command router owns the dial deadband/ramp and chassis wz. */
    vr.rc.ch[4] = (int16_t)rotation;
    vr_uart_last_rotation = rotation;
    vr_uart_last_axes = ((uint32_t)(uint16_t)right << 16) | (uint16_t)forward;
    ++vr_uart_publish_count;
    (void)MsgCenter_Publish(TOPIC_RC_UPDATE, &vr, sizeof(vr));
}

static void vr_poll(uint32_t now_ms)
{
    for (unsigned budget = 0; budget < 256U; ++budget) {
        int byte = BspRc_VtmReadByte();
        if (byte < 0) break;
        if (vr_len == 0U && byte != 0xA5) continue;
        vr_frame[vr_len++] = (uint8_t)byte;
        if (vr_len == 7U &&
            (vr_frame[1] != 30U || vr_frame[2] != 0U ||
             (vr_frame[5] != 2U && vr_frame[5] != 0x11U) || vr_frame[6] != 3U ||
             vr_crc(vr_frame, 4, 0xFF, 0x8C) != vr_frame[4])) {
            uint8_t i = 1;
            while (i < vr_len && vr_frame[i] != 0xA5U) ++i;
            vr_len -= i;
            memmove(vr_frame, vr_frame + i, vr_len);
        }
        if (vr_len == VR_FRAME_BYTES) {
            uint16_t tail = (uint16_t)vr_frame[37] | ((uint16_t)vr_frame[38] << 8);
            if (vr_crc(vr_frame, 37, 0xFFFF, 0x8408) == tail) vr_accept(now_ms);
            vr_len = 0U;
        }
    }
}

static void on_frame(const uint8_t *frame, uint16_t length)
{
    NyushRemoteSnapshot decoded;
    if (length != RC_FRAME_LENGTH || !NyushRemote_GetSnapshot(&decoded)) return;
    memcpy(rc_ctrl.rc.ch, decoded.channels, sizeof(rc_ctrl.rc.ch));
    rc_ctrl.rc.s[0] = decoded.switches[0];
    rc_ctrl.rc.s[1] = decoded.switches[1];
    rc_ctrl.mouse.x = decoded.mouse[0];
    rc_ctrl.mouse.y = decoded.mouse[1];
    rc_ctrl.mouse.z = decoded.mouse[2];
    rc_ctrl.mouse.press_l = decoded.buttons[0];
    rc_ctrl.mouse.press_r = decoded.buttons[1];
    rc_ctrl.key.v = decoded.keys;
    memcpy(last_sbus_frame, frame, RC_FRAME_LENGTH);
    last_dbus_ms = BspTime_NowMs();
    dbus_seen = 1U;
    rc_frame_count++;
    (void)MsgCenter_Publish(TOPIC_RC_UPDATE, &rc_ctrl, sizeof(rc_ctrl));
}
static void poll_receiver(void *user)
{
    (void)user;
    uint32_t now_ms = BspTime_NowMs();
    vr_poll(now_ms);
    if ((uint32_t)(now_ms - s_daemon_tick_ms) >= 10U) {
        s_daemon_tick_ms = now_ms;
        NyushRemote_DaemonTick();
    }
}
void remote_control_init(void)
{
    (void)vr_mqtt_firmware_tag;
    (void)vr_dual_firmware_tag;
    BspRc_SetFrameCallback(on_frame);
    s_daemon_tick_ms = BspTime_NowMs();
    /* Upstream RobotInit excludes IRQs until instance and daemon pointers exist. */
    BspCriticalState state = BspCritical_Enter();
    NyushRemote_Init();
    BspRc_VtmInit();
    BspCritical_Exit(state);
    (void)MsgCenter_AddAfterDispatchHook(poll_receiver, NULL);
}
uint32_t RC_GetFrameCount(void) { return rc_frame_count; }
const RC_ctrl_t *get_remote_control_point(void) { return &rc_ctrl; }
void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH])
{
    if (out) memcpy(out, last_sbus_frame, RC_FRAME_LENGTH);
}
