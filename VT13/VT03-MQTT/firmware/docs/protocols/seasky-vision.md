# 现有 USB CDC / Seasky 视觉协议

> 本文只描述当前兼容协议。未来 Jetson 传输必须由独立 adapter 解码为
> `VisionTargetMessage` 并发布 `TOPIC_VISION_TARGET`，不能让线协议直接进入
> 应用层。参见[全局架构与调试](../architecture/overview.md)。

## 当前通道和长度

- 物理通道：USB CDC 虚拟串口；
- 协议：Seasky 二进制帧，CRC8 + CRC16；
- 上位机到 MCU：2 个 float，固定 18 字节；
- MCU 到上位机：3 个 float，当前实际发送 22 字节；
- `VISION_SEND_SIZE` 为 36 字节发送缓冲区容量，不是当前有效帧长。

## 帧格式

```text
offset  size  content
0       1     SOF = 0xA5
1       2     data_length，小端
3       1     header CRC8（覆盖前 3 字节）
4       2     command ID，小端
6       2     flags，小端
8       N     IEEE-754 float payload，小端
8+N     2     frame CRC16，小端
```

`data_length = 2-byte flags + float payload`，不包含 4 字节帧头、2 字节命令
ID 和 2 字节 CRC16。因此总帧长为 `data_length + 8`。

## 上位机到 MCU：命令 0x0001

有效载荷为 `pitch`、`yaw` 两个 float：

```c
typedef struct {
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;
    float pitch;
    float yaw;
    uint8_t updated;
} Vision_Recv_s;
```

flags 当前解释：

| 位 | 内容 |
|---|---|
| 0..1 | `NO_FIRE=0`、`AUTO_FIRE=1`、`AUTO_AIM=2` |
| 2..3 | `NO_TARGET=0`、`TARGET_CONVERGING=1`、`READY_TO_FIRE=2` |
| 4..7 | 目标编号：0 无目标，1 英雄，2 工程，3..5 步兵，6 前哨站，7 哨兵，8 基地 |

当前接收路径：

```text
Src/usbd_cdc_if.c
  -> VisionComm_RxCallback()
  -> get_protocol_info() 校验并解析
  -> TOPIC_VISION_DATA / Vision_Recv_s
  -> LegacyVisionBridge
  -> TOPIC_VISION_TARGET / VisionTargetMessage
```

新应用应订阅标准主题 `TOPIC_VISION_TARGET`，不要新增对
`TOPIC_VISION_DATA` 的业务依赖。

## MCU 到上位机：命令 0x0002

线协议字段名沿用 `yaw`、`pitch`、`roll`，但当前运行代码依次填入
`SensorData.g_gz`、`g_gx`、`g_gy`，即原始陀螺仪 Z/X/Y 角速度（rad/s）。
它们不是云台姿态角，不能用作视觉解算的历史姿态或闭环角度反馈。视觉模块订阅
`TOPIC_IMU_UPDATE`，并以 `vision_comm.c` 中固定的 10 ms 间隔尝试发送，因此
标称约 100 Hz；实际频率受 IMU 发布频率、控制任务耗时和 USB busy 状态影响。

当前实现有两个必须注意的兼容问题：

1. 当前载荷把角速度放在了姿态字段，主机必须按实际含义解释；更改其语义
   需要单独确认协议版本和现有接收方。
2. `VisionComm_Send()` 使用固定 flags 表达式 `30 << 8 | 1`；
   `VisionComm_SetFlag()` 保存的 enemy color、work mode 和 bullet speed 目前没有
   被打包进发送帧。
3. `VisionConfig.send_interval_ms` 虽可被修改，但实际发送判断仍使用
   `vision_comm.c` 内固定的 10 ms 宏。

在修复运行路径前，上位机必须按实际固定 flags 解释，不能按旧文档假设所有
`VisionComm_SetFlag()` 参数已经生效。

## CRC 和失败行为

- CRC8 校验帧头前 3 字节；
- CRC16 校验除末尾 CRC16 自身之外的整个帧；
- SOF 或任一 CRC 失败时 `get_protocol_info()` 返回 0；
- 基础 `VisionComm_RxCallback()` 会忽略失败帧，但不会完整更新增强诊断计数。

## 测试脚本

主机端脚本：

```bash
python3 script/test_vision_comm.py --help
```

它可以构造 18 字节的 `0x0001` 帧并解析 MCU 回传。测试时至少确认：

1. 发送端使用小端 float；
2. pitch/yaw 的单位为弧度；
3. 帧长严格为 18；
4. CRC8、CRC16 与固件算法一致；
5. `TOPIC_VISION_TARGET` 最终到达 `CmdController`；
6. 超过应用视觉超时后 `vision_valid` 能清除。

## 实现位置

- `modules/vision_comm/seasky_protocol.c`：组帧、解析；
- `modules/vision_comm/crc8.c`、`crc16.c`：校验；
- `modules/vision_comm/vision_comm.c`：视觉协议状态、消息发布和兼容通信；
- `bsp/usb/bsp_usb.c`：USB CDC 字节发送，向上层隐藏生成代码；
- `adapters/vision/legacy_vision_bridge.c`：旧协议到标准消息；
- `core/contracts/vision_messages.h`：应用层稳定契约；
- `adapters/vision/jetson_vision_port.c`：未知 Jetson 协议的安全占位。

## 相关文档

- [视觉诊断与当前限制](../guides/vision-diagnostics.md)
- [消息中心](message-center.md)
