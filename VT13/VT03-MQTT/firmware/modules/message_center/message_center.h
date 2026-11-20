/*
 * 声明模块间使用的消息中心。
 * 它不是 RTOS 调度器；回调由唯一的控制任务集中派发。
 */
#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Topics aligned with plan; extend as needed
typedef enum {
    TOPIC_RC_UPDATE = 0,
    TOPIC_IMU_UPDATE,
    TOPIC_CAN_RX,
    TOPIC_MOTOR_FEEDBACK,
    TOPIC_GM6020_FEEDBACK,
    TOPIC_CHASSIS_CMD,
    TOPIC_SHOOT_CMD,
    TOPIC_GIMBAL_CMD,
    TOPIC_VISION_DATA,     // Vision data from upper computer
    TOPIC_VISION_TARGET,   // Transport-neutral VisionTargetMessage
    TOPIC_ROBOT_STATE,     // Transport-neutral VisionRobotStateMessage
    TOPIC_NUM_TOPICS
} MsgTopic;

#ifndef MC_MAX_PAYLOAD
#define MC_MAX_PAYLOAD 128
#endif

#ifdef __cplusplus
#define MC_PAYLOAD_ALIGN alignas(4)
#else
#define MC_PAYLOAD_ALIGN _Alignas(4)
#endif

typedef struct {
    uint16_t topic;     // MsgTopic
    uint16_t size;      // bytes used in data
    MC_PAYLOAD_ALIGN uint8_t data[MC_MAX_PAYLOAD];
} MsgEvent;

#undef MC_PAYLOAD_ALIGN

typedef void (*MsgCallback)(const MsgEvent *ev, void *user_data);
typedef void (*MsgDispatchHook)(void *user_data);

// Initialize message center with user-provided ring buffer
void MsgCenter_Init(MsgEvent *buffer, size_t length);

// Publish event (ISR-safe). Returns 0 on success.
int MsgCenter_Publish(MsgTopic topic, const void *data, size_t size);

/* 最新状态不进入普通队列。同一 topic/key 只保留最后一份，不能用于逐条事件。
 * STATE 在普通消息前派发；CONTROL 在状态和普通消息后派发，随后执行发送 hook。
 * key 由发布者定义（例如软件电机编号），不由消息中心解释载荷。
 */
typedef enum {
    MC_LATEST_STATE = 0,
    MC_LATEST_CONTROL = 1
} MsgLatestPhase;
#define MC_LATEST_CAPACITY 32U

/* 初始化时将整个主题切为单份最新状态。0 成功；未初始化/参数错误/容量不足返回负值。
 * 已在普通队列内的该主题旧消息会被跳过，防止覆盖新状态。重复配置必须使用相同 phase。
 */
int MsgCenter_UseLatest(MsgTopic topic, MsgLatestPhase phase);
/* 中断内可调用，复制载荷后返回。首次使用 topic/key 占用一个固定槽，直到 Init 才释放。
 * 0 成功；非法载荷、phase 冲突或 32 槽耗尽返回负值，不挤掉其他键或控制命令。
 */
int MsgCenter_PublishLatest(MsgTopic topic, uint16_t key, MsgLatestPhase phase,
                           const void *data, size_t size);

// Subscribe to a topic; returns 0 on success.
int MsgCenter_Subscribe(MsgTopic topic, MsgCallback cb, void *user_data);

/* A bounded batch guarantees the control loop and transport hooks get CPU time. */
#define MC_DISPATCH_BUDGET 64U
typedef struct {
    uint32_t dispatches;
    uint32_t events;
    uint32_t budget_hits;
    uint32_t overwritten;
    /* 所有计数均为消息条数；合并旧状态不等于普通队列丢事件。 */
    uint32_t published_by_topic[TOPIC_NUM_TOPICS];
    uint32_t delivered_by_topic[TOPIC_NUM_TOPICS];
    uint32_t overwritten_by_topic[TOPIC_NUM_TOPICS];
    uint32_t coalesced_by_topic[TOPIC_NUM_TOPICS];
    uint32_t rejected_by_topic[TOPIC_NUM_TOPICS];
} MsgCenterDiagnostics;
const MsgCenterDiagnostics *MsgCenter_GetDiagnostics(void);
/* 每次最多派发 64 条普通消息，并各处理一次最新状态槽；回调补发不会无限延长派发。
 * 只有唯一主循环/任务可以调用，不能在回调或中断中递归调用。
 */
void MsgCenter_Dispatch(void);

/*
 * Register a generic end-of-dispatch hook. This is intended for composition
 * concerns such as flushing buffered transports. The message center does not
 * know which service owns the hook.
 */
void MsgCenter_SetAfterDispatchHook(MsgDispatchHook hook, void *user_data);
int MsgCenter_AddAfterDispatchHook(MsgDispatchHook hook, void *user_data);

#ifdef __cplusplus
}
#endif

#endif // MESSAGE_CENTER_H
