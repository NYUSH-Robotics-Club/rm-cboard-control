# FreeRTOS 运行与验证

## 源码中的启动路径

`Src/main.c` 完成外设、控制器、IMU 校准和通信初始化后调用
`RobotRtos_Start()`。该函数创建一个静态 `control` 任务，再调用
`vTaskStartScheduler()`；调度器若意外返回，进入 `Error_Handler()`。
`Src/stm32f4xx_it.c` 将 SVC、PendSV 转接到 FreeRTOS Cortex-M4F port；
SysTick 先递增 HAL tick，调度器启动后再递增 FreeRTOS tick。
这描述的是当前源码，尚不证明目标板已烧录或调度器已在实机运行。

- 内核：仓库固定的 FreeRTOS Kernel V11.3.0，位于
  `Middlewares/Third_Party/FreeRTOS-Kernel/`。
- tick：1 kHz，沿用 SysTick；HAL/BSP 毫秒时基从上电连续计时，
  任务调度 tick 从调度器启动时计时。`USE_RTOS=0` 是 HAL 宏，不表示调度器关闭。
- `control`：优先级 4，静态栈 1024 words，每轮完成后 `vTaskDelay(2)`。
- `idle`：静态栈 128 words。FreeRTOS 动态分配关闭，不在运行时创建或删除任务。
- Quaternion EKF 首次更新仍可能使用 C 库 `malloc`；它不是 FreeRTOS heap，
  但需另行检查 RAM 余量和分配失败路径。

## 控制任务

任务以 `BspTime_NowMs()` 为应用时间戳，顺序为：

```text
IMU 更新 -> 可选应用步进 -> Cmd 路由 -> 消息派发及电机刷新 hook
-> 电机离线报警 -> 限流 CAN 统计 -> vTaskDelay(2 ticks)
```

上电时仍在调度器启动前执行云台对齐和 IMU 校准。当前可选应用清单为空，
`AppRuntime_Step()` 不会启动额外业务。消息回调和电机发送仍由同一任务拥有；
中断只采集/发布，不等待订阅者。旧 `HAL_Delay(1)` 实际等待两次毫秒 tick；这里用任务阻塞保留这一等待
语义，不追赶过期周期，避免改变按回调次数累加的 pitch 目标和发射斜坡。
周期包含执行时间和等待时间，1 kHz tick 不表示控制循环为 1 kHz。

FreeRTOS 最大系统调用中断优先级为库优先级 5，最低为 15。
目前 CAN/UART/USB 中断不调用 FreeRTOS API。新增 ISR 唤醒任务时必须
使用 `...FromISR` API，并重新核对 IRQ 优先级。

## 后续任务规则

1. 在 `runtime/rtos/` 集中创建任务，使用静态栈并写明周期、优先级、
   栈预算、超时与失效时的安全行为。
2. 每个可写资源保持单一任务所有者；跨任务传递数据须定义消息或静态快照。
3. 保持 `MsgCenter_Dispatch()` 与电机发送只有一个执行者。
4. 拆分任务前测最坏执行时间、周期抖动、栈高水位和总线负载。

## 验证边界

两种车型都需 ARM 编译、链接并检查 RAM/Flash。上板后还需验证：
调度器和控制任务实际运行，HAL/RTOS tick 均递增；控制周期的平均值、
最坏值和抖动；控制任务栈高水位；CAN、USB、UART 中断与 HardFault；
遥控掉线、队列满、传感器离线时安全归零；底盘、云台和哨兵转向无行为回归。
源码与构建通过不能替代这些实测。
