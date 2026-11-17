# C 板 / ST-Link / RTT

## 目的与数据流

```text
电脑终端或网页 ←→ OpenOCD RTT TCP :9090 ←→ ST-Link / SWD
                                                ↕
                                     C 板固件 / CAN1 或 CAN2
                                                ↕
                                           达妙电机
```

ST-Link 访问 SEGGER RTT 的上/下行缓冲；固件接收命令后处理 CAN，并把完成/接收事件
返回终端。RTT 仍需要固件支持，ST-Link 本身不是 USB-CAN。OpenOCD 的 RTT 搬运间隔
（当前 5 ms）不是电机查询或控制周期；`status` 是显式查询，不周期刷新整块诊断 RAM。

`Diagnosis` 分支上层结构：

- `runtime/diagnosis/main.c` 初始化 GPIO/CAN，启动诊断前台循环，无整车控制任务。
- `services/diagnosis/diagnosis_terminal.c` 解析 RTT 文本，发送 JSON Lines 回复和事件。
- `services/diagnosis/diagnosis.c` 校验请求、处理结果与内部事件环。
- `bsp/diagnosis/diagnosis_can.c` 管理 CAN、RX 回调、硬件发送完成/失败/超时。
- `bsp/diagnosis/diagnosis_rtt.c` 包装 SEGGER RTT 非阻塞收发。
- `tools/diagnosis/manage.py` 构建、明确烧录、只连接并启动 RTT TCP。
- `tools/diagnosis/terminal.py` 原始双向终端；`interface.py` 为 RTT 流的网页后端。
- `tools/diagnosis/slow_rotate.py` 为已确认 CAN1 J3507 的显式低速运行工具。

冻结的 HAL/CubeMX 文件保持原样。旧 `Src/main.c` 不参与当前入口构建。依赖方向为
runtime → services/core → bsp → HAL，RTT 不从 CAN ISR 内调用。

## 接入与命令

先定位仓库，检查当前分支与进程；一个 OpenOCD 独占探针，一个终端/网页/控制进程
独占 RTT 通道。若已有低速控制进程，不再打开第二个通道写入者。

```sh
cd /实际路径/rm-cboard-control
cmake --preset Debug
cmake --build --preset Debug
sh tests/host/run_tests.sh
```

连接已经烧有匹配固件的板子：

```sh
python3 tools/diagnosis/manage.py connect --serial 实际STLink序列号
```

此进程保持运行，不能与 IDE 调试器抢占探针。连接不暂停/复位/恢复目标；目标若暂停，
由操作者明确恢复。默认使用 Debug ELF 的 `_SEGGER_RTT` 符号，Release 必须同时
指定 `--mode Release`；每次固件变更可能改变控制块地址，不能沿用旧 ELF。

macOS USB 元数据为空但 OpenOCD 可以连接时，不能据元数据判定断线。只有确认
物理上仅一个探针，才可用 `connect --probe-first` 明确让 OpenOCD 选第一个；多个
探针须指定序列号。

烧录是独立操作，先完成构建与目标确认；仅在已授权时执行：

```sh
python3 tools/diagnosis/manage.py flash --serial 实际STLink序列号
```

会检查家族 ID 0x413 / Flash 1024 KiB，复位并再次核对后擦写、校验、启动默认只听固件。
这两个字段不能区分所有具体板型，仍须确认物理目标。旧固件需要保留时，先只读备份。

第二个终端运行 `just terminal`，逐行输入，每条使用不同的非零 token，并等待其回复：

```text
hello 1001
status 1002
config 1003 1 1000000 1
read 1004 1 1 7
status 1005
```

含义是协议握手、查状态、CAN1/1 Mbps/正常模式、读取电机 1 的 RID 7。这是只读
电机诊断，不使能、不修改电机参数；`config` 只改变 C 板通道状态。

| 命令 | 行为 |
| --- | --- |
| `hello TOKEN` | 返回 RTT 协议版本 2，不发 CAN |
| `status TOKEN` | 返回状态/计数，不发 CAN |
| `config TOKEN CHANNEL BITRATE MODE` | MODE 0=只听，1=正常，2=内部静默回环 |
| `read TOKEN CHANNEL MOTOR_ID RID` | 向 0x7FF 发一条八字节达妙参数读取 |
| `send TOKEN CHANNEL CAN_ID FLAGS HEX` | 任意单次 CAN 帧；HEX 无空格，`-` 为零字节 |

FLAGS 位 0=扩展、位 1=远程。原始 send 可能运动/改参数，不当作只读命令。
ASCII 每行最多 254 字节。reply.result：0 接受、1 无效、2 忙、3 硬件错误、4 只听。

CAN 事件独立推送。`event.words` 顺序为
seq/ms/kind/channel/id/flags/length/data_low/data_high/token。
kind：1 RX，2 TX_QUEUED，3 TX_OK，4 TX_ERROR，5 TX_TIMEOUT。
使用仓库 `protocol.decode_event()` 和 `BUS_FIELDS` 解码，不猜字段位置。

网页可选：`just interface` 后打开 http://127.0.0.1:8765；停掉原始终端再使用。
HTTP 刷新只返回主机缓存，计数通过「查询状态」更新，报文由 RTT 推送。

## 运行、停止和边界

当前固件上电两路 1 Mbps 只听，不自动发电机查询、使能或速度。正常模式参与 ACK，
只听不 ACK；两节点总线用只听观察时，电机可能一直不回复。

内部静默回环可测试 TX_OK、RX、回调计数，但不验证外部收发器/线缆。每总线一个
在途发送，关闭自动重传；50 ms 超时请求取消，硬件未确认前保持 BUSY。不要盲目重发
运动命令或反复重启掩盖错误。输出缓冲满时不阻塞 CAN；回复保留，CAN 事件可丢弃，
查看 rx_dropped 与 rtt_dropped。不能把网页当无损总线分析仪。

用户明确要求低速旋转且目标确为该脚本支持的 J3507 时：

```sh
python3 -u tools/diagnosis/slow_rotate.py
```

它读回 ID=1、反馈 ID=0、P/V/T，失能后临时切速度模式 3，读回确认，先发零速再使能，
持续 +0.1 rad/s。没有参数保存命令。打印当前 PID；Ctrl-C 或 `kill -INT 当前PID`
退出，分别尝试零速和失能。0.5 s 无新反馈、异常状态、发送失败或实际速度超过
1 rad/s 则退出。`DISABLED_FEEDBACK_CONFIRMED` 才表示收到失能反馈；
`STOP_NOT_CONFIRMED` 不能解释为已停。SIGKILL 无法执行退出处理。

修改固件时按仓库要求跑主机检查与 Debug/Release 构建；构建不能代替实板验证。
权威入口：仓库 `docs/diagnosis.md`、`docs/architecture/overview.md`。
