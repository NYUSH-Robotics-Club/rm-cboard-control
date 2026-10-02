# 云台上电与耦合补偿

实现位于 `application/gimbal/gimbal_controller.c`；车型参数位于
`config/robots/infantry_standard.c`。

## 当前上电目标

两轴都要等到有效反馈后才允许输出。yaw 的 `initial_angle=-1`，
因此从实际位置锁存目标；步兵 pitch 的 `initial_angle=1971`，
因此在反馈和目标有效后使用固定启动编码目标。pitch 的软件目标
限位为 1607～2374。两轴的 PID 状态在重新对齐时清理。

固定启动目标可能使机构运动；不能把 pitch 描述为“始终锁存当前位置”。
软件目标限位也不能替代机械端点、线束和惯性越界的实车验证。

## 耦合与重力

步兵配置中的 `enable_yaw_pitch_compensation=false`，当前关闭 yaw
转动引起的 pitch 目标修正；`gravity_compensation=0`，当前关闭
重力前馈。相关算法仍在控制器内，启用前必须核对几何、符号、单位、
补偿方向和机械行程。不要只凭旧调参记录直接打开。

## Pitch经验负载前馈

pitch另有`s_pitch_control.load_*`经验电压补偿，独立于上述重力正弦项；
补偿按实际位置变化、实测区间外钳位，总输出仍受原内环上限约束。
数据来源、拟合误差及启用比例见[GM6020说明](../gm6020-control-modes.md)。
经验曲线同时包含摩擦与弹簧等负载，不能解释为纯弹簧力或任意更换零点。
