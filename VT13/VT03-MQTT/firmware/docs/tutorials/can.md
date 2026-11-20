# CAN 与电机协议速查

CAN（Controller Area Network）用 CAN-H/CAN-L 差分线连接控制板和驱动器。同一
总线必须共地、两端正确终端、波特率一致；标准帧 ID 是 11 位，经典 CAN 数据
最多 8 字节。

## 代码路径

```text
应用命令 -> MotorService -> 厂商 adapter -> 协议 codec -> BspCan_Write -> HAL
HAL RX ISR -> CAN_Manager -> TOPIC_CAN_RX -> 厂商 adapter -> MotorSnapshot
```

- `bsp/can/` 只收发标准帧和读诊断，不解释电机协议。
- `modules/can_comm/` 管理通道和 DJI 兼容反馈。
- `modules/motor_protocols/` 是无硬件依赖的字节编解码器。
- `adapters/motor/` 处理厂商使能、限幅、反馈状态和统一控制环。
- 应用层不能直接调用 HAL CAN 或写 8 字节数组。

## DJI

现有配置使用 M3508/C620 和 GM6020。常见反馈为大端：角度 bytes 0~1、速度
2~3、电流/转矩 4~5、温度 byte 6。分组命令把四个有符号 16 位值按大端装入
8 字节；实际 ID、slot 和通道来自 `config/robots/*.c`，不要凭电机编号推算。

当前代码对 M3508/M2006 命令限幅 ±16384，对 GM6020 兼容命令限幅 ±25000。
这些是驱动器命令单位，不应在文档中笼统称为安培。官方资料位于：

- `docs/official-docs/Robomaster_C620_Docs.pdf`
- `docs/official-docs/RM_GM6020_Docs.pdf`

## DM（达妙）MIT 模式

`dm_motor_protocol.c` 实现：

- 位置 16 位；速度、Kp、Kd、力矩各 12 位；
- `FF FF FF FF FF FF FF FC` 使能，末字节 `FD` 失能；
- 反馈解析状态、位置、速度、力矩和两路温度。

P/V/T 映射范围因型号和 DM 工具设置而异，必须逐台写入
`MotorConfig_t.protocol.dm`。代码不会把参考仓库的 ±π、±45、±54 当通用值。

## 本末 BM1505B

当前独立实现支持：

- `0x032`：地址 1~4 分组命令；`0x033`：地址 5~8；命令为大端 int16；
- `0x105`：失能 `0x09`、使能 `0x0A`、当前模式 `0x01`、速度模式 `0x02`；
- `0x106`：每个地址的反馈周期；
- 反馈按速度、电流、角度、故障、模式保留原始单位。

必须配置物理地址、RX ID、模式、反馈周期和命令限幅。不同本末系列的波特率、
量程和反馈 ID 可能不同，BM1505B 实现不能冒充所有本末电机。

## 瓴控

当前只支持由厂商工具预先开启的四电机广播电流模式：

- TX `0x280`，slot 0~3 按小端 int16 排列；
- RX `0x141`~`0x144`；反馈为温度、电流、速度（deg/s）、编码器，小端；
- `command_limit` 和 `encoder_counts_per_rev` 必须按具体系列配置。

单电机 A0/A1/A2 等模式尚未接入统一服务。驱动不会自动改工具参数。

## 接新电机的安全步骤

1. 取得准确型号手册，记录波特率、ID、大小端、单位、限幅、使能/失能和故障。
2. 在 `config/robots/*.c` 增加一个 `MOTOR_TYPE_VENDOR_DEFINED` 实例；不要先改应用。
3. 先运行 `tests/host/run_tests.sh` 和两车型 ARM 构建。
4. 实车先使用 `MOTOR_CONTROL_DISABLED`，只观察反馈和在线时间。
5. 架空轮组、准备急停，用极小开环命令核对方向。
6. 依次调速度、位置、串级环，每步记录反馈和输出限幅。

新增协议应先写纯 codec 和字节测试，再写 adapter；禁止把未知帧发到实车试错。
