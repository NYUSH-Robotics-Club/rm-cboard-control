/* Check queue order, full-queue behavior, subscriptions, and dispatch hooks. */
#include <assert.h>
#include <stdint.h>
#include "message_center.h"

static int s_received = 0;
static int s_hook_calls = 0;
static unsigned s_refill_calls;
static unsigned feedback_count, command_count, fresh_value, command_value;
static unsigned latest_refills;

static void feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    fresh_value = *(const uint32_t *)ev->data;
    ++feedback_count;
}
static void command(const MsgEvent *ev, void *user)
{
    (void)user;
    command_value = *(const uint32_t *)ev->data;
    /* 控制必须在本轮最新反馈之后执行，即便普通队列已经溢出。 */
    assert(fresh_value == 999U);
    ++command_count;
}
static void latest_refill(const MsgEvent *ev, void *user)
{
    (void)user;
    ++latest_refills;
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 1, MC_LATEST_STATE,
                                  ev->data, ev->size) == 0);
}

static void after_dispatch(void *user_data);
static void refill_queue(const MsgEvent *event, void *user_data);

static void test_latest_under_overload(void)
{
    MsgEvent queue[8];
    MsgCenter_Init(queue, 8);
    assert(MsgCenter_UseLatest(TOPIC_GIMBAL_CMD, MC_LATEST_CONTROL) == 0);
    assert(MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, feedback, 0) == 0);
    assert(MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, command, 0) == 0);
    assert(MsgCenter_AddAfterDispatchHook(after_dispatch, 0) == 0);
    for (uint32_t cycle = 0; cycle < 100; ++cycle) {
        /* 交替使能/停机命令，均不能被反馈洪水或普通队列溢出挤掉。 */
        uint32_t enabled = cycle & 1U;
        assert(MsgCenter_Publish(TOPIC_GIMBAL_CMD, &enabled, sizeof(enabled)) == 0);
        for (uint32_t value = 0; value < 1000; ++value) {
            for (uint16_t id = 1; id <= 9; ++id) {
                assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, id, MC_LATEST_STATE,
                                              &value, sizeof(value)) == 0);
            }
            assert(MsgCenter_Publish(TOPIC_ROBOT_STATE, &value, sizeof(value)) == 0);
        }
        MsgCenter_Dispatch();
        assert(feedback_count == (cycle + 1U) * 9U);
        assert(command_count == cycle + 1U && command_value == enabled);
    }
    const MsgCenterDiagnostics *d = MsgCenter_GetDiagnostics();
    assert(d->overwritten_by_topic[TOPIC_GIMBAL_CMD] == 0);
    assert(d->overwritten_by_topic[TOPIC_ROBOT_STATE] > 0);
    assert(d->delivered_by_topic[TOPIC_GIMBAL_CMD] == 100);
    assert(d->coalesced_by_topic[TOPIC_MOTOR_FEEDBACK] == 100U * 9U * 999U);
    assert(d->rejected_by_topic[TOPIC_MOTOR_FEEDBACK] == 0);

    /* 普通事件在回调里持续补发，控制和发送 hook 仍在同一轮获得执行机会。 */
    MsgCenter_Init(queue, 8);
    uint32_t ready = 999, stop = 0;
    s_refill_calls = 0;
    unsigned hooks_before = (unsigned)s_hook_calls;
    assert(MsgCenter_UseLatest(TOPIC_GIMBAL_CMD, MC_LATEST_CONTROL) == 0);
    MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, feedback, 0);
    MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, command, 0);
    MsgCenter_Subscribe(TOPIC_ROBOT_STATE, refill_queue, 0);
    MsgCenter_AddAfterDispatchHook(after_dispatch, 0);
    MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 1, MC_LATEST_STATE, &ready, 4);
    MsgCenter_Publish(TOPIC_GIMBAL_CMD, &stop, 4);
    MsgCenter_Publish(TOPIC_ROBOT_STATE, &stop, 4);
    MsgCenter_Dispatch();
    assert(command_count == 101 && command_value == 0);
    assert(s_refill_calls == MC_DISPATCH_BUDGET && (unsigned)s_hook_calls == hooks_before + 1U);

    /* 回调补发到同一最新槽，不允许一个派发循环永不返回。 */
    MsgCenter_Init(queue, 8);
    uint32_t value = 999;
    assert(MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, latest_refill, 0) == 0);
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 1, MC_LATEST_STATE, &value, 4) == 0);
    MsgCenter_Dispatch(); assert(latest_refills == 1);
    MsgCenter_Dispatch(); assert(latest_refills == 2);

    /* 启用最新策略前的旧 FIFO 消息不能随后覆盖新状态。 */
    MsgCenter_Init(queue, 8);
    value = 7;
    assert(MsgCenter_Publish(TOPIC_RC_UPDATE, &value, 4) == 0);
    assert(MsgCenter_UseLatest(TOPIC_RC_UPDATE, MC_LATEST_STATE) == 0);
    assert(MsgCenter_Subscribe(TOPIC_RC_UPDATE, feedback, 0) == 0);
    value = 999;
    assert(MsgCenter_Publish(TOPIC_RC_UPDATE, &value, 4) == 0);
    unsigned before = feedback_count;
    MsgCenter_Dispatch(); assert(fresh_value == 999 && feedback_count == before + 1U);

    /* 键空间耗尽必须明确失败，不能复用其他电机/控制槽。Init 应释放全部槽。 */
    MsgCenter_Init(queue, 8);
    for (uint16_t id = 0; id < MC_LATEST_CAPACITY; ++id)
        assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, id, MC_LATEST_STATE, &value, 4) == 0);
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 100, MC_LATEST_STATE, &value, 4) < 0);
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 0, MC_LATEST_CONTROL, &value, 4) < 0);
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 0, MC_LATEST_STATE, 0, 4) < 0);
    assert(MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, 0, MC_LATEST_STATE, &value, 4) == 0);
    MsgCenter_Init(queue, 8);
    assert(MsgCenter_UseLatest(TOPIC_GIMBAL_CMD, MC_LATEST_CONTROL) == 0);
}


static void refill_queue(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    ++s_refill_calls;
    if (s_refill_calls < 1000U) {
        assert(MsgCenter_Publish(TOPIC_ROBOT_STATE, event->data, event->size) == 0);
    }
}

static void on_message(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    assert(event->topic == TOPIC_ROBOT_STATE);
    assert(event->size == sizeof(uint32_t));
    s_received = (int)*(const uint32_t *)event->data;
}

static void after_dispatch(void *user_data)
{
    (void)user_data;
    ++s_hook_calls;
}

int main(void)
{
    MsgEvent queue[8];
    uint32_t value = 42U;
    MsgCenter_Init(queue, 8U);
    assert(MsgCenter_Subscribe(TOPIC_ROBOT_STATE, on_message, 0) == 0);
    assert(MsgCenter_AddAfterDispatchHook(after_dispatch, 0) == 0);
    assert(MsgCenter_Publish(TOPIC_ROBOT_STATE, &value, sizeof(value)) == 0);
    MsgCenter_Dispatch();
    assert(s_received == 42);
    assert(s_hook_calls == 1);

    MsgCenter_Init(queue, 8U);
    s_hook_calls = 0;
    assert(MsgCenter_Subscribe(TOPIC_ROBOT_STATE, refill_queue, 0) == 0);
    assert(MsgCenter_AddAfterDispatchHook(after_dispatch, 0) == 0);
    MsgCenter_Publish(TOPIC_ROBOT_STATE, &value, sizeof(value));
    MsgCenter_Dispatch();
    assert(s_refill_calls == MC_DISPATCH_BUDGET && s_hook_calls == 1);
    assert(MsgCenter_GetDiagnostics()->budget_hits == 1);
    MsgCenter_Dispatch();
    assert(s_refill_calls == 2U * MC_DISPATCH_BUDGET && s_hook_calls == 2);
    test_latest_under_overload();
    return 0;
}
