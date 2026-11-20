/* Keep upstream RC_ctrl_t private; expose decoded values for the message bridge. */
#ifndef NYUSH_REMOTE_ADAPTER_H
#define NYUSH_REMOTE_ADAPTER_H
#include <stdint.h>
typedef struct {
    int16_t channels[5], mouse[3];
    uint8_t switches[2], buttons[2];
    uint16_t keys;
} NyushRemoteSnapshot;
/* Initialize the copied DBUS module on USART3. Call with receiver IRQs excluded. */
void NyushRemote_Init(void);
/* Copy decoded values. Returns zero before initialization. */
int NyushRemote_GetSnapshot(NyushRemoteSnapshot *out);
/* Advance the original daemon once; the bare-metal caller supplies its 10 ms cadence. */
void NyushRemote_DaemonTick(void);
#endif
