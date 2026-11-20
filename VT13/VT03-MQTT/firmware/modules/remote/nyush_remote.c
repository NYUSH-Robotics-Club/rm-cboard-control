/* Compile the upstream decoder unchanged and adapt its data at the boundary.
 * Prefix its optional VTM CRC dependency to avoid the vision module's CRC symbols. */
#define crc_16 Nyush_crc_16
#include "../../third_party/nyush_remote/modules/remote/remote_control.c"
#include "nyush_remote.h"
void NyushRemote_Init(void) { (void)RemoteControlInit(&huart3); }
int NyushRemote_GetSnapshot(NyushRemoteSnapshot *out)
{
    const RC_ctrl_t *data = RemoteControlGetData();
    if (!data || !out) return 0;
    out->channels[0] = data->rc.rocker_r_;
    out->channels[1] = data->rc.rocker_r1;
    out->channels[2] = data->rc.rocker_l_;
    out->channels[3] = data->rc.rocker_l1;
    out->channels[4] = data->rc.dial;
    out->switches[0] = data->rc.switch_right;
    out->switches[1] = data->rc.switch_left;
    out->mouse[0] = data->mouse.x;
    out->mouse[1] = data->mouse.y;
    out->mouse[2] = data->mouse.z;
    out->buttons[0] = data->mouse.press_l;
    out->buttons[1] = data->mouse.press_r;
    out->keys = data->key[KEY_PRESS].keys;
    return 1;
}
void NyushRemote_DaemonTick(void) { DaemonTask(); }
