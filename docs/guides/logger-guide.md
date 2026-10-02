# Logger 与 RTT 遥测

固件文本日志、USB CDC 串口工具和 RTT 仪表盘是两条不同链路。当前实现以
`modules/logger/`、`script/logger.py` 和 `scripts/dashboard/` 为准。
启动前先按[快速开始](../quickstart.md)配置 Python 环境和工具链。

## RTT 仪表盘

```bash
source tools/activate.sh
python -m pyocd pack install stm32f407ighx
just logger
```

打开 `http://127.0.0.1:8080/`。通过 SSH 使用时转发 8080 端口。终端版入口是
`just logger-cli`。`just logger --help` 只检查脚本依赖和参数，不能验证板卡或遥测。
仪表盘要求探针可用且板上运行的固件包含 RTT；`Control block not found` 时先检查
烧录镜像和运行状态。`just flash` 会构建、校验并按本机 `run_after` 配置决定是否复位运行；
运行值可用 `just flash-plan` 核对。烧录与仪表盘不能同时占用同一探针。

默认连接方式保持 CPU 原运行状态。显式 `halt`、`under-reset` 和 GDB 断点会改变控制循环，
应先隔离动力输出。RTT 读取会更新缓冲区读指针。更换固件后重启 logger 并刷新网页；
旧镜像未发送的字段不能由网页补出。网页无新帧超过 1 秒会提示数据未更新。

### 读数判断

- `IMU & Gimbal` 的 yaw 为连续角度，pitch 为单圈编码角度；速度目标来自实际内环。
  当前 `speed_loop_only=false`，yaw 位置环闭环目标可以显示。禁用、未对齐或反馈过期时，
  相应字段为空，不能把历史值当作当前控制输出。
- `Motors` 的速度是转子 RPM；`Chassis Cmd` 是归一化命令，不是 m/s。
- `CAN1 RX`、`CAN2 RX` 表示配置电机近期有反馈，不等于 CAN 总线无故障或遥控已解锁。
- 日志中的 `nan` 表示当前字段无效或未接入；服务接受命令不证明 CAN 已发送或电调已执行。

当前遥测包含 yaw 来源、速度 PID 与输出诊断，以及 pitch PID 诊断。网页和
`monitor/logger_*.txt` 会保存这些字段；终端版默认保存为 `monitor/logger_cli_*.txt`。
排查 yaw 路由时优先比较
`yaw_rc_ch0`、`yaw_route_rate`、`yaw_rc_sequence`、`yaw_route_sequence` 和各来源年龄；
最新遥控摇杆快照不一定是当前 PID 回调采用的输入。比较 `yaw_callback_count` 和
`telemetry_drop_count` 可区分未采到的控制回调与 RTT 写入失败；后者不是 CAN 丢帧数。
`yaw_pid_*`、`yaw_command_raw` 和 `yaw_current_actual_raw` 使用电机协议刻度，不能直接
解释成安培。pitch 诊断同样保留命令单位、状态、原始命令、反馈电流、内外环速度、
PID 各项、增益、限幅和 dt；无效字段为 `nan`。采样通常约 20 ms 一次，不能据此推断
每一次控制回调。当前参数和硬件未决事项见[项目状态](../project/PROJECT_MEMO.md)。

## yaw 命令到电流的事件记录（v12）

烧录含 v12 遥测的固件后，重新启动 `just logger`。原 `logger_*.txt` 保留全部
控制/PID字段，追加 `yaw_transport_*` 总线元数据：通道、TX/RX ID、槽位、批次数量、
累计事件覆盖数 `lost`、ESR、TSR、HAL错误及空闲邮箱数。网页原始字段区同步显示这些值。
旧 v3～v11 固件仍可解析，但不产生有效的传输事件。

同目录新增同名 `logger_*.yaw_transport.tsv`（终端版同样保存）。每行是一条事件：
`ms` 是 MCU 毫秒时间，`event_seq` 是事件序号，`mailbox` 为0～2，-1表示不适用。
`host_rx_ms` 是主机收到批次的时间，不能用于推算电机执行延迟。

| kind | 含义 | raw | detail |
|---|---|---|---|
| 1 | CAN聚合器接受已限幅的yaw命令 | 协议命令刻度 | 0 |
| 2 | HAL接受帧并分配发送邮箱 | 实际帧中yaw槽位的命令 | 最近kind=1的事件序号 |
| 3 | 观察到TXOK | 原始TSR位 | 对应kind=2的事件序号 |
| 4 | 观察到仲裁丢失ALST | 原始TSR位 | 对应kind=2的事件序号 |
| 5 | 观察到发送错误TERR | 原始TSR位 | 对应kind=2的事件序号 |
| 6 | 协议层解析收到的合法yaw反馈帧 | 反馈电流刻度 | 有符号RPM的uint32表示 |
| 7 | BSP/HAL拒绝提交帧 | 待发送yaw命令 | 最近kind=1的事件序号 |
| 8 | 邮箱结束/复用但未保留可确认的结果 | TSR或0 | 对应kind=2的事件序号 |

负RPM在detail中表示为二进制补码：值大于等于2^31时减去2^32。
原始线协议的kind低8位为事件类型、位8～9为邮箱号加1；Python侧文件已拆成两列。
类型3～5的raw也是位模式，解释TSR时先按uint32读取。

记录器根据当前车型的DJI GM6020 yaw配置选择总线、ID和槽位，不写死步兵ID。
RX在既有协议解析处逐帧记录，保留接收时间，不在BSP解析电机反馈字段；TX结果在任务上下文中、下次邮箱提交前或批次读取时轮询，不启用新中断。
**结果时间是观察时刻，不是精确的总线发送完成时刻。** kind=2只证明邮箱接受；
kind=3证明发送成功，但不能证明电调已经执行；反馈电流没有回显命令序号，不能逐帧
直接建立命令与电流的一一对应。kind=8必须作为测量缺口，不能算成功或仲裁失败。

256条静态环形缓存每帧最多导出48条事件，RTT缓冲为8192字节；不阻塞控制、不动态分配。
缓存满覆盖最旧事件并增加`yaw_transport_lost`。RTT写入失败丢弃该批次并增加原有
`telemetry_drop_count`；主机CRC失败/漏读还会表现为帧或事件序号缺口。测试有效性必须同时
检查这些计数的增量，不能把日志丢失当成CAN丢帧。上电到开始采集前的累计丢失单独看待。
新增记录会增加处理和RTT带宽开销，实测时同时比较`yaw_callback_dt_ms`，不要假定零开销。

建议先记录静止保持，再分别给小幅正向、反向和连续换向目标；保留主日志与事件文件。
用序号连接1→2→3/4/5/8，检查7是否出现，再用MCU时间比较原始反馈电流变化。
本功能只观察，未改变PID、分段、发送重试、自动重传或安全解锁行为。

## 固件文本日志和 USB CDC

```c
#include "logger.h"
LOG_WARN(LOG_TAG_CAN, "CAN start failed: %d", status);
LOG_CSV(LOG_TAG_GIM, "YAW,%.2f,%.2f", target, feedback);
Logger_SetRate(LOG_TAG_GIM, 50); // 最短间隔 50 ms
```

`modules/logger/logger_config.h` 管理编译期开关和默认限流；目前只启用 `GIM` 标签。
文本格式为 `[TAG][LEVEL] message`，CSV 为 `TAG,timestamp_ms,fields...`。
`--tags all` 只能取消主机端过滤，无法启用固件中关闭的标签。

文本链路是 `Logger -> Debug_SendString -> BspUsb_Write -> USB CDC`，使用 C 板自己的
USB 数据口；ST-Link 的虚拟串口不会自动转发它。先确认设备归属，再显式传入端口：

```bash
source tools/activate.sh
python -m serial.tools.list_ports -v
python script/logger.py /dev/serial/by-id/实际C板设备 --tags GIM
python script/logger.py /dev/serial/by-id/实际C板设备 --tags GIM --save gimbal_session.csv
```

没有输出时依次检查：固件是否运行、是否接入 C 板 USB CDC、标签开关和实际调用路径。
大量文本日志会占用 USB CDC 带宽；调试结束后关闭不需要的标签。旧绘图脚本位于
`script/deprecated/`，没有随当前消息契约维护。
