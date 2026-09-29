# FreeRTOS 预留设计（尚未启用）

## 当前状态

当前 main 没有调用 `RobotRtos_Start()`。以下为仓库保留的配置和未来
接入设计，不是当前运行状态。
底层冻结，安装最新开发工具不会升级或启用这份内核。

- 内核：官方 FreeRTOS Kernel V11.3.0，MIT，固定源码位于
  `Middlewares/Third_Party/FreeRTOS-Kernel/`；
- API：原生 FreeRTOS；
- tick：1 kHz，沿用 Cortex-M SysTick；
- RTOS 对象内存：只允许静态任务/栈，FreeRTOS 动态分配关闭；
- 预留入口：`RobotRtos_Start() -> vTaskStartScheduler()`，main 尚未调用；
- 业务任务：一个优先级 4、栈 1024 words、周期 1 ms 的 `control` 任务；
- 系统任务：FreeRTOS idle task，栈 128 words。

当前没有“动态 RTOS 任务管理”：不会在运行时创建、删除任务，FreeRTOS 也不
使用动态分配路径。这是预留设计；当前确实没有启动 RTOS。
需要区分一个旧遗留：Quaternion EKF 首次更新仍调用 C 库 `malloc` 分配矩阵；
它不是 FreeRTOS heap，但仍需检查 RAM 余量和分配失败行为。

## 中断和时基

当前 `Src/stm32f4xx_it.c` 的 SVC、PendSV 为空，SysTick 只调用 `HAL_IncTick()`。
未来接入必须另行获准修改冻结底层并验证异常转接。预期时基规则为：

- HAL/BSP 毫秒时间从上电连续计时；
- RTOS tick 从调度器启动计时，只用于任务调度；
- 应用超时继续用 `BspTime_NowMs()`，避免启动前后时间基准突变；
- `USE_RTOS` 这个 STM32 HAL 宏保持 0，它不是 FreeRTOS 是否启用的开关。

FreeRTOS 最大可调用系统 API 的中断优先级配置为库优先级 5，最低优先级 15。
当前 CAN/UART/USB ISR 不调用 FreeRTOS API，仍只采集/发布并使用很短临界区。

## 预留控制任务为何不拆分

未启用的控制任务定义以下顺序（不等于当前 main 的执行链）：

```text
IMU -> 可选应用步进 -> Cmd -> 消息派发 -> 电机刷新 -> 蜂鸣器 -> 限流诊断
```

这样可保证：

- 消息回调只有一个执行者；
- 同一 CAN 分组帧只有一个刷新点；
- 上电改成 RTOS 时不同时改变控制时序和共享状态。

后续不能仅因“RTOS 能多任务”就拆任务。先测量每段最坏执行时间、控制周期抖动、
栈高水位和总线负载。只有明确的低优先级、可丢弃或会阻塞工作，才适合独立任务，
例如日志输出和未来 Jetson 大帧解析。

## 未来获准接入后的新增任务规则

1. 使用 `xTaskCreateStatic()` 和静态栈；禁止新增 `xTaskCreate()`、`malloc()`。
2. 在 `runtime/rtos/` 集中创建，不在业务模块自行启动任务。
3. 每个可写资源只能有一个任务所有者；跨任务用标准消息或静态快照。
4. ISR 需要唤醒任务时必须使用 `...FromISR` API，并重新核对 IRQ 优先级。
5. 电机发送和 `MsgCenter_Dispatch()` 仍保持单一所有者。
6. 写明周期、优先级、栈预算、超时和失效时安全行为。

## 未来启用后的上板验收

- 两种 `ROBOT_TYPE` 均完成 ARM 编译、链接并检查 RAM/Flash；
- 观察 `control` 任务确实运行，HAL tick 和 RTOS tick 都递增；
- 测量 1 ms 周期的平均值、最坏值和抖动；
- 读取栈高水位，给 1024-word 栈留下可解释余量；
- 检查 CAN、USB、UART 中断优先级和 HardFault；
- 遥控掉线、队列满、传感器离线时输出安全归零；
- 当前全向轮、现有舵轮和云台上电无行为回归。

上述任务周期、栈和中断行为尚未实测。
