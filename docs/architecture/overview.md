# 诊断固件结构

依赖方向为 `runtime -> services/core -> bsp -> HAL`。诊断是有意暴露原始 CAN
的工具，原始帧契约在 `core/diagnosis/`；不经过整车电机控制器或厂商适配器。

| 目录 | 职责 |
| --- | --- |
| `runtime/diagnosis/` | 独立入口、72 MHz 时钟树、中断向量实现、主循环 |
| `services/diagnosis/` | 校验主机请求、发布状态、环形事件记录 |
| `core/diagnosis/` | 内部请求/状态与 RTT 记录的字段顺序 |
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

## RTT 双向终端协议 v2

电脑经 OpenOCD 的 RTT TCP server 访问通道 0；OpenOCD 通过 SWD 搬运 SEGGER RTT
缓冲区。主机不读取/写入 `g_diagnosis`，也不做周期性状态探测。`g_diagnosis` 保留为
内部服务状态和调试器观察变量，RTT 终端是其请求槽的唯一写入者。

OpenOCD 按匹配 ELF 中的 `_SEGGER_RTT` 符号定位控制块。`manage.py connect`
启动通道 0 的 TCP server（默认 9090），5 ms 的 RTT 上行检查间隔是调试器搬运字节流
的频率，不是电机查询周期。固件没有自动发电机查询的定时任务。

命令为 ASCII、换行结尾，最多 254 字节；每条包含唯一非零 uint32 token。数字支持
十进制和 0x 前缀。输出为 JSON Lines：hello/status/reply/event。示例：

```text
hello 1
status 2
config 3 1 1000000 1
read 4 1 1 7
status 5
```

- `hello TOKEN`：返回 protocol=2，不访问 CAN。
- `status TOKEN`：返回 uptime_ms、loops、两路 CAN 各 16 字状态与 rtt_dropped，不发送 CAN。
- `config TOKEN CHANNEL BITRATE MODE`：显式配置，MODE=0 只听、1 正常、2 内部静默回环。
- `read TOKEN CHANNEL MOTOR_ID RID`：单次读参数，ID=0x7FF，DLC=8，数据为电机 ID
  的低/高字节、33、RID、四个 00。长度与已验证的达妙 SDK 一致。
- `send TOKEN CHANNEL CAN_ID FLAGS HEX`：任意单次帧，FLAGS 位 0 表示扩展帧、位 1
  表示远程帧；HEX 是无空格的十六进制串，`-` 表示零字节。可能运动或修改参数。

reply 的 result 数值沿用 DiagnosisStatus：0 接受、1 无效、2 忙、3 硬件错误、4 只听。
发送接受与 TX_OK 分开；CAN 事件独立推送为 `event.words`，顺序是
seq/ms/kind/channel/id/flags/length/data_low/data_high/token。
status.bus 的字段顺序见 `DiagnosisBus`，与主机 `BUS_FIELDS` 一致。

终端在前台解析命令，分段输入跨循环拼接；超长/含 NUL/错误格式整行拒绝。每轮最多
读 256 字节、输出 16 个 CAN 事件，CAN ISR 只操作 BSP 队列。上行缓冲满时不阻塞
CAN；命令回复保留重试、CAN 事件丢弃并累计 rtt_dropped。128 条内部事件环覆盖也计入。
下行 512 字节，上行 8192 字节。主机不得批量灌入命令，收到对应 token 回复后才提交下一条。

只允许一个终端或网页后端使用同一 RTT 通道，避免多个 TCP 客户端向下行交错写入。
网页后端用独立接收线程消费事件，HTTP 轮询只返回主机缓存；status 由按钮或用户操作
触发。网页主机保留最近 1000 条事件，导出的是这一窗口，不是无损总线记录仪。

连接/状态/网页启动不含复位、暂停、恢复或烧录命令。`manage.py flash` 是明确写板操作。
若 macOS USB 元数据为空但 OpenOCD 能连接，确认物理上只有一个探针后可显式使用
`--probe-first`；多个探针必须指定实际序列号，不能猜选。

参考：[SEGGER RTT](https://kb.segger.com/RTT)、[OpenOCD RTT 命令](https://openocd.org/doc/html/General-Commands.html#Real-Time-Transfer-_0028RTT_0029)。
