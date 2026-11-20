/*
 * 实现固定内存的消息排队、订阅和派发。
 * 逐条事件保留有界队列，最新反馈/命令独立存储；不执行控制算法。
 */
#include "message_center.h"
#include "bsp_critical.h"
#include <string.h>

#define MC_MAX_SUBS_PER_TOPIC 8
#define MC_MAX_DISPATCH_HOOKS 4

typedef struct {
    MsgCallback cb;
    void *user;
} Subscriber;

static MsgEvent *mc_queue = NULL;
static size_t mc_len = 0;
static volatile size_t mc_head = 0; // write index (next free)
static volatile size_t mc_tail = 0; // read index (oldest)

static Subscriber mc_subs[TOPIC_NUM_TOPICS][MC_MAX_SUBS_PER_TOPIC];
static uint8_t mc_inited = 0;
static MsgDispatchHook mc_after_dispatch_hooks[MC_MAX_DISPATCH_HOOKS];
static void *mc_after_dispatch_users[MC_MAX_DISPATCH_HOOKS];
static MsgCenterDiagnostics mc_diagnostics;

typedef struct {
    MsgEvent event;
    uint16_t key;
    uint8_t phase;
    uint8_t pending;
} LatestSlot;

/* 槽只在初始化后分配，不释放；所有载荷读写及 pending 交接由临界区保护。 */
static LatestSlot mc_latest[MC_LATEST_CAPACITY];
static size_t mc_latest_count;
static int8_t mc_latest_topic[TOPIC_NUM_TOPICS];

#define MC_CS_ENTER() BspCriticalState _critical_state = BspCritical_Enter()
#define MC_CS_EXIT() BspCritical_Exit(_critical_state)

void MsgCenter_Init(MsgEvent *buffer, size_t length) {
    mc_queue = buffer;
    mc_len = (buffer && length) ? length : 0;
    mc_head = 0;
    mc_tail = 0;
    memset(&mc_diagnostics, 0, sizeof(mc_diagnostics));
    memset(mc_latest, 0, sizeof(mc_latest));
    memset(mc_latest_topic, -1, sizeof(mc_latest_topic));
    mc_latest_count = 0U;
    for (size_t t = 0; t < (size_t)TOPIC_NUM_TOPICS; ++t) {
        for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
            mc_subs[t][i].cb = NULL;
            mc_subs[t][i].user = NULL;
        }
    }
    mc_inited = (mc_len > 0);
    for (size_t i = 0; i < MC_MAX_DISPATCH_HOOKS; ++i) {
        mc_after_dispatch_hooks[i] = NULL;
        mc_after_dispatch_users[i] = NULL;
    }
}

static int mc_is_full(void) {
    size_t next = (mc_head + 1U) % mc_len;
    return (next == mc_tail);
}

static int mc_is_empty(void) {
    return (mc_head == mc_tail);
}

/* 调用者已关闭中断，查找/分配都限制在固定 32 槽内。 */
static int latest_slot(MsgTopic topic, uint16_t key, MsgLatestPhase phase)
{
    for (size_t i = 0U; i < mc_latest_count; ++i) {
        if (mc_latest[i].event.topic == topic && mc_latest[i].key == key) {
            return mc_latest[i].phase == phase ? (int)i : -2;
        }
    }
    if (mc_latest_count == MC_LATEST_CAPACITY) return -3;
    size_t i = mc_latest_count++;
    mc_latest[i].event.topic = (uint16_t)topic;
    mc_latest[i].key = key;
    mc_latest[i].phase = (uint8_t)phase;
    return (int)i;
}

int MsgCenter_UseLatest(MsgTopic topic, MsgLatestPhase phase)
{
    if (!mc_inited) return -1;
    if (topic < 0 || topic >= TOPIC_NUM_TOPICS ||
        (phase != MC_LATEST_STATE && phase != MC_LATEST_CONTROL)) return -2;
    MC_CS_ENTER();
    int slot = latest_slot(topic, 0U, phase);
    if (slot >= 0) mc_latest_topic[topic] = (int8_t)slot;
    else mc_diagnostics.rejected_by_topic[topic]++;
    MC_CS_EXIT();
    return slot >= 0 ? 0 : slot;
}

static void store_latest(int slot, const void *data, size_t size)
{
    LatestSlot *s = &mc_latest[slot];
    if (s->pending) mc_diagnostics.coalesced_by_topic[s->event.topic]++;
    s->event.size = (uint16_t)size;
    if (size) memcpy(s->event.data, data, size);
    s->pending = 1U;
    mc_diagnostics.published_by_topic[s->event.topic]++;
}

int MsgCenter_PublishLatest(MsgTopic topic, uint16_t key, MsgLatestPhase phase,
                           const void *data, size_t size)
{
    if (!mc_inited) return -1;
    if (topic < 0 || topic >= TOPIC_NUM_TOPICS || size > MC_MAX_PAYLOAD ||
        (size && !data) || (phase != MC_LATEST_STATE && phase != MC_LATEST_CONTROL)) return -2;
    MC_CS_ENTER();
    int slot = latest_slot(topic, key, phase);
    if (slot >= 0) store_latest(slot, data, size);
    else mc_diagnostics.rejected_by_topic[topic]++;
    MC_CS_EXIT();
    return slot >= 0 ? 0 : slot;
}

int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size) {
    if (!mc_inited || !mc_queue || mc_len == 0) return -1;
    if (size > MC_MAX_PAYLOAD || topic < 0 || topic >= TOPIC_NUM_TOPICS ||
        (size && !data)) return -2;

    MC_CS_ENTER();
    int slot = mc_latest_topic[topic];
    if (slot >= 0) {
        store_latest(slot, data, size);
    } else {
        if (mc_is_full()) {
            mc_diagnostics.overwritten_by_topic[mc_queue[mc_tail].topic]++;
            mc_tail = (mc_tail + 1U) % mc_len;
            mc_diagnostics.overwritten++;
        }
        MsgEvent *ev = &mc_queue[mc_head];
        ev->topic = (uint16_t)topic;
        ev->size = (uint16_t)size;
        if (size) memcpy(ev->data, data, size);
        mc_head = (mc_head + 1U) % mc_len;
        mc_diagnostics.published_by_topic[topic]++;
    }
    MC_CS_EXIT();
    return 0;
}

int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data) {
    if (!mc_inited || (topic < 0) || (topic >= TOPIC_NUM_TOPICS) || !cb) {
        return -1;
    }
    for (size_t i = 0; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
        if (mc_subs[topic][i].cb == NULL) {
            mc_subs[topic][i].cb = cb;
            mc_subs[topic][i].user = user_data;
            return 0;
        }
    }
    return -2;
}

static void deliver(const MsgEvent *ev)
{
    MsgTopic topic = (MsgTopic)ev->topic;
    if (topic < 0 || topic >= TOPIC_NUM_TOPICS) return;
    mc_diagnostics.events++;
    mc_diagnostics.delivered_by_topic[topic]++;
    for (size_t i = 0U; i < MC_MAX_SUBS_PER_TOPIC; ++i) {
        if (mc_subs[topic][i].cb) mc_subs[topic][i].cb(ev, mc_subs[topic][i].user);
    }
}

static void dispatch_latest(MsgLatestPhase phase)
{
    /* 每个槽本轮只访问一次；回调期间再次发布留到下一轮，避免生产者饿死控制。 */
    for (size_t i = 0U; i < MC_LATEST_CAPACITY; ++i) {
        MsgEvent ev;
        uint8_t ready = 0U;
        MC_CS_ENTER();
        if (i < mc_latest_count && mc_latest[i].phase == phase && mc_latest[i].pending) {
            ev = mc_latest[i].event;
            mc_latest[i].pending = 0U;
            ready = 1U;
        }
        MC_CS_EXIT();
        if (ready) deliver(&ev);
    }
}

void MsgCenter_Dispatch(void) {
    if (!mc_inited || !mc_queue || mc_len == 0) return;
    mc_diagnostics.dispatches++;
    dispatch_latest(MC_LATEST_STATE);
    size_t processed = 0U;
    for (; processed < MC_DISPATCH_BUDGET; ++processed) {
        MC_CS_ENTER();
        if (mc_is_empty()) {
            MC_CS_EXIT();
            break;
        }
        MsgEvent ev = mc_queue[mc_tail];
        mc_tail = (mc_tail + 1U) % mc_len;
        uint8_t obsolete = mc_latest_topic[ev.topic] >= 0;
        if (obsolete) mc_diagnostics.coalesced_by_topic[ev.topic]++;
        MC_CS_EXIT();
        if (!obsolete) deliver(&ev);
    }
    if (processed == MC_DISPATCH_BUDGET) mc_diagnostics.budget_hits++;
    dispatch_latest(MC_LATEST_CONTROL);
    for (size_t i = 0U; i < MC_MAX_DISPATCH_HOOKS; ++i) {
        if (mc_after_dispatch_hooks[i]) mc_after_dispatch_hooks[i](mc_after_dispatch_users[i]);
    }
}

const MsgCenterDiagnostics *MsgCenter_GetDiagnostics(void) { return &mc_diagnostics; }

void MsgCenter_SetAfterDispatchHook(MsgDispatchHook hook, void *user_data) {
    MC_CS_ENTER();
    for (size_t i = 0; i < MC_MAX_DISPATCH_HOOKS; ++i) {
        mc_after_dispatch_hooks[i] = NULL;
        mc_after_dispatch_users[i] = NULL;
    }
    mc_after_dispatch_hooks[0] = hook;
    mc_after_dispatch_users[0] = user_data;
    MC_CS_EXIT();
}

int MsgCenter_AddAfterDispatchHook(MsgDispatchHook hook, void *user_data) {
    if (!hook) {
        return -1;
    }
    MC_CS_ENTER();
    for (size_t i = 0; i < MC_MAX_DISPATCH_HOOKS; ++i) {
        if (mc_after_dispatch_hooks[i] == hook) {
            mc_after_dispatch_users[i] = user_data;
            MC_CS_EXIT();
            return 0;
        }
    }
    for (size_t i = 0; i < MC_MAX_DISPATCH_HOOKS; ++i) {
        if (!mc_after_dispatch_hooks[i]) {
            mc_after_dispatch_hooks[i] = hook;
            mc_after_dispatch_users[i] = user_data;
            MC_CS_EXIT();
            return 0;
        }
    }
    MC_CS_EXIT();
    return -2;
}
