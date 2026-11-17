/* Exercise the actual receiver source with host-only byte/time/message stubs. */
#include "remote_control.h"
#include "nyush_remote.h"
#include "bsp_rc.h"
#include "bsp_time.h"
#include "bsp_critical.h"
#include "message_center.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t bytes[4096];
static size_t cursor, size;
static uint32_t now_ms;
static BspRcFrameCallback dbus_callback;
static MsgDispatchHook poll_hook;
static void *poll_user;
static RC_ctrl_t latest;
extern volatile uint32_t vr_uart_valid_count, vr_uart_publish_count;
uint32_t BspTime_NowMs(void) { return now_ms; }
BspCriticalState BspCritical_Enter(void) { return 0; }
void BspCritical_Exit(BspCriticalState previous) { (void)previous; }
void BspRc_SetFrameCallback(BspRcFrameCallback callback) { dbus_callback=callback; }
void BspRc_VtmInit(void) {}
int BspRc_VtmReadByte(void) { return cursor<size ? bytes[cursor++] : -1; }
void NyushRemote_Init(void) {}
void NyushRemote_DaemonTick(void) {}
int NyushRemote_GetSnapshot(NyushRemoteSnapshot *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot)); snapshot->channels[3]=66;
    snapshot->switches[0]=snapshot->switches[1]=RC_SW_DOWN; return 1;
}
int MsgCenter_AddAfterDispatchHook(MsgDispatchHook hook, void *user) { poll_hook=hook; poll_user=user; return 0; }
int MsgCenter_Publish(MsgTopic topic, const void *data, size_t length) {
    assert(topic==TOPIC_RC_UPDATE && length==sizeof(latest)); memcpy(&latest,data,length); return 0;
}
int main(int argc, char **argv) {
    assert(argc==2); FILE *file=fopen(argv[1],"rb"); assert(file); remote_control_init();
    uint8_t header[6];
    while (fread(header,1,6,file)==6) {
        now_ms=(uint32_t)header[0]|((uint32_t)header[1]<<8)|((uint32_t)header[2]<<16)|((uint32_t)header[3]<<24);
        unsigned length=(unsigned)header[4]|((unsigned)header[5]<<8);
        if (length==65535) { uint8_t frame[18]={0}; dbus_callback(frame,18); }
        else { assert(length<=sizeof(bytes)); size=length; cursor=0; assert(fread(bytes,1,size,file)==size); poll_hook(poll_user); }
        printf("%u %u %d %d\n",(unsigned)vr_uart_valid_count,(unsigned)vr_uart_publish_count,latest.rc.ch[3],latest.rc.ch[2]);
    }
    fclose(file); return 0;
}
