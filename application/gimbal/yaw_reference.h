/*
 * 保存yaw连续位置和目标，处理编码器跨零与按时间累加输入。
 * 状态由云台应用独占；不访问电机、PID或硬件。
 */
#ifndef YAW_REFERENCE_H
#define YAW_REFERENCE_H
#include <stdbool.h>
#include <stdint.h>

/* 这是跟踪有效性限制，不是电机物理最大速度或机械限位。 */
#define YAW_REFERENCE_MAX_GAP_MS 20U
#define YAW_REFERENCE_MAX_SPEED_RPM 500.0f
#define YAW_ENCODER_TICKS 8192.0f

typedef struct {
    bool valid;
    uint16_t last_raw;
    uint32_t feedback_ms;
    uint32_t control_ms;
    float position_ticks; /* 连续角，允许超出一圈。 */
    float target_ticks;   /* 与position_ticks同一坐标；误差不做半圈归一化。 */
} YawReference;

/* 首次有效反馈时锁当前位置；调用者已完成反馈新鲜度和启动检查。 */
void YawReference_Seed(YawReference *ref, uint16_t raw,
                       uint32_t feedback_ms, uint32_t now_ms);
/*
 * 读取新的反馈并返回本次控制间隔（秒）。重复毫秒dt=0，不额外积分。
 * 超时、越界、超出跟踪速度范围或跳变有歧义时返回false并作废，须重新Seed。
 */
bool YawReference_Update(YawReference *ref, uint16_t raw, int16_t rpm,
                         uint32_t feedback_ms, uint32_t now_ms, float *dt_s);
/* 按输入积分连续目标；input_gain为无量纲摇杆放大倍数，stick=0严格保留目标。 */
void YawReference_Advance(YawReference *ref, float stick, float rate_deg_s,
                          float dt_s, float input_gain);
/* 输出0..8192的单圈兼容角，不改变内部连续位置/目标。 */
float YawReference_Wrap(float ticks);
#endif
