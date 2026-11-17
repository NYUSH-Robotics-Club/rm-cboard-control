# 诊断固件结构

依赖方向为 `runtime -> services/core -> bsp -> HAL`。诊断是有意暴露原始 CAN
的工具，原始帧契约在 `core/diagnosis/`；不经过整车电机控制器或厂商适配器。

| 目录 | 职责 |
| --- | --- |
| `runtime/diagnosis/` | 独立入口、72 MHz 时钟树、中断向量实现、主循环 |
| `services/diagnosis/` | 校验主机请求、发布状态、环形事件记录 |
| `core/diagnosis/` | 固定 32 位字的 SWD 内存契约 |
| `bsp/diagnosis/` | 两路 CAN、接收回调、发送完成/失败/超时、寄存器快照 |
| `tools/diagnosis/` | OpenOCD Tcl 连接、主机校验、本地网页、明确烧录入口 |
| `tests/host/` | 主机服务测试、HAL 替身下的 CAN 行为测试 |

顶层 CMake 调用冻结的生成文件，再筛选目标源码：只使用 GPIO、CAN、MSP、
syscalls/sysmem 与启动汇编。生成的 HAL/USB 库保留原构建，未使用的代码由链接器
回收。旧 `Src/main.c`、中断文件和 USB 应用源码不编译；没有 RTOS、IMU 或遥控控制。
底层文件没有因为诊断入口而改写。

## 运行与所有权

两路 CAN 以 1 Mbps 只听启动。正常模式和内部静默回环仅由显式配置请求选择。
正常模式会参与 ACK；只听模式不 ACK。CAN 使用固定 12 TQ、APB1=36 MHz，
支持 125/200/250/500/1000 kbps，关闭自动重传和自动 Bus-off 恢复。

RX0 ISR 经 HAL 回调将时间戳、ID、标准/扩展、数据/远程标志、DLC 和八字节原始
数据写入容量 64 的 BSP 队列。两路 IRQ 同优先级、不互相抢占。队列满丢弃新帧，
`rx_dropped` 累计软件溢出与已观察到的硬件 FIFO overrun 次数，后者不是精确丢帧数。
主循环在短临界区取出事件，每轮最多处理 64 个。ISR 不访问主机命令或 SWD 事件环。

每总线最多一个在途发送。`TX_QUEUED` 仅代表 HAL 邮箱接收；轮询 TSR 的完成位
决定 `TX_OK` 或 `TX_ERROR`，TX IRQ 不启用以保留完成位的所有权。
`last_tx_tsr` 锁存完成/超时时的 TSR，`tsr` 为当前值。50 ms 未完成则请求取消并
报告 `TX_TIMEOUT`，邮箱仍被占用，直至硬件确认。取消失败持续 BUSY，不自动重发；
取消与成功竞争可随后产生 `TX_OK`。Bus-off 由操作员排除原因后重新应用配置。

## SRAM ABI v1

全局符号 `g_diagnosis` 位于普通 SRAM，不在 CCM。主机从所选 ELF 的符号表定位，
启动时校验 magic `0x44494147`、ABI `1` 和长度 `5376`，不猜固定地址。

| 字偏移（每字 4 字节） | 内容 |
| --- | --- |
| 0–15 | magic、ABI、长度、version、uptime_ms、loops、event_head、ack、result、保留 |
| 16–47 | CAN1/CAN2 各 16 字状态，定义见 `DiagnosisBus` |
| 48–63 | 主机请求，定义见 `DiagnosisRequest` |
| 64–1343 | 128 个事件，每条 10 字，定义见 `DiagnosisEvent` |

主机先写请求参数，最后单独写非零 `seq`；固件对每个新序号执行一次并返回
`ack/result`。主机只在旧请求已确认后提交下一个请求。状态用 version 奇偶数
发布，读前后相同且为偶数才接受。固件循环间的 HAL 延时提供读取窗口；主机
读取不到稳定快照时报错，不通过暂停 MCU 来读取。

事件槽写入前 seq=0，写完后提交序号，最后推进 event_head。主机按序号检查覆盖，
并复核可能正在被重写的最旧槽的提交字；检测到的遗漏计数可见。序号回绕或
观察到 MCU 时间倒退会清空主机历史，重新开始读取。低速 SWD 或满载 CAN 会丢记录，
因此网页和 JSON 不能替代总线分析仪。主机保留最近 1000 条，导出的是这一窗口。

一个 OpenOCD 会话独占 ST-Link，一个 Python 进程独占命令邮箱。不要同时运行 IDE
调试会话或多个主机进程。网页操作由主机锁串行化。网页只绑定 127.0.0.1，写请求
使用随机会话 token 与本地 Origin 校验。连接、状态读取和网页启动不含复位、暂停、
恢复或烧录命令；`manage.py flash` 是独立且明确的写板操作。
