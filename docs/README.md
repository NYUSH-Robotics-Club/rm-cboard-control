# 文档索引

这里仅列维护中的说明。文档描述仓库当前源码；上板状态必须以对应固件与当次
实测记录为准。旧实验、参数和已删除文档可从 Git 历史查阅，不作为当前依据。

## 入门与架构

- [构建、配置与烧录](quickstart.md)
- [程序结构和数据流](architecture/overview.md)
- [未启用的 RTOS 设计](architecture/rtos-migration.md)
- [项目约束与未决问题](project/PROJECT_MEMO.md)
- [代码注释约定](project/COMMENTING_STANDARD.md)

## 控制与安全

- [全向轮底盘](omni-chassis.md)、[云台方向平移跟随](chassis-follow.md)
- [GM6020 模式与当前配置](gm6020-control-modes.md)
- [CAN 故障恢复](can-recovery.md)、[电机失联报警](motor-offline-alarm.md)
- [拨弹堵转处理](feed-stall-recovery.md)、[云台上电与耦合补偿](guides/gimbal-compensation.md)

## 观测与接口

- [启动顺序](guides/boot-sequence.md)
- [RTT 和串口日志](guides/logger-guide.md)、[云台监控](gimbal-monitor.md)
- [消息中心](protocols/message-center.md)、[现行 Seasky 视觉协议](protocols/seasky-vision.md)
- [视觉诊断与 Jetson 接口边界](guides/vision-diagnostics.md)
- [CAN 与电机协议说明](tutorials/can.md)

`official-docs/` 保存厂商手册。它们是协议参考，不能替代本车配置或实测。
