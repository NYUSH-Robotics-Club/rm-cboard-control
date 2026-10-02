# 视觉诊断与当前限制

本文区分“已经接入运行路径的能力”和“代码存在但尚未接入的能力”。这一区分
对 Jetson 联调非常重要，不能仅看到函数存在就认为固件已经在使用它。

## 当前实际数据流

```text
USB CDC 接收
  -> Src/usbd_cdc_if.c
  -> VisionComm_RxCallback()
  -> TOPIC_VISION_DATA / Vision_Recv_s
  -> LegacyVisionBridge
  -> TOPIC_VISION_TARGET / VisionTargetMessage
  -> CmdController / CommandRouter
  -> GimbalController 双轴位置目标与现有闭环
```

当前接入的是基础回调 `VisionComm_RxCallback()`：

- 接收固定 18 字节的 Seasky 帧；
- 解析命令 `0x0001`；
- 发布旧协议消息；
- 再由适配器转换成传输无关的视觉目标消息。

姿态回传由 IMU 主题触发，`vision_comm.c` 中固定使用 10 ms 间隔，即约 100 Hz。
回传字节现在通过 `bsp/usb` 发送；冻结的 USB 接收回调仍保持不变。

## 双轴目标与控制

路由只接收版本匹配、目标有效且双轴误差有限的消息；每次接收递增内部
`vision_frame`，控制周期复用消息时不递增。沿用20 ms无新消息退出视觉的规则，
未根据未知的相机帧率扩大超时。遥控失联与禁用仍优先停机。

每个新帧将弧度误差转换为8192刻度/圈的位置增量，加到本次处理时的轴位置：
yaw使用连续`target_ticks`，pitch使用绝对编码目标并裁剪到车型配置限位
（步兵1607～2374）。两轴帧间保持目标，不重复叠加误差，不经过摇杆倍率或时间积分，
视觉有效时不叠加手动杆量。退出视觉时从实际位置恢复手动控制。

普通/视觉/小陀螺yaw共用现有位置/速度PID、分段速度上限、阻尼和反向制动；视觉基础上限
仍读取`vision_speed_rpm`。pitch共用现有PID、单一速度阻尼、负载配置和越界锁存停机，
不新增视觉专用PID，不恢复端点制动平滑。pitch切换输入源只清位置环历史，保留承重速度积分。

内部误差正值表示增加对应电机编码目标。旧Seasky适配器仍直接传递yaw/pitch弧度；
相机方向与电机编码方向的对应尚需联调确认，不能根据pitch摇杆的direction猜视觉符号。
现协议没有可用的拍摄时间与历史姿态配对，因此本路径不含视觉延迟补偿；
可选目标速度字段也尚未用作前馈。Jetson新协议仍不支持。

## 尚未接入运行路径的能力

以下实现目前存在于 `modules/vision_comm/`，但当前 USB CDC 回调和启动流程
没有调用它们：

| 能力 | 代码状态 | 当前运行状态 |
|---|---|---|
| 长度/范围/超时检查与 EMA 滤波 | `VisionComm_RxCallback_Enhanced()` 已实现 | 未接入 USB 接收回调 |
| `VisionDiagnostics`/`VisionDataQuality` 统计 | 接口已实现 | 基础回调不会完整更新 |
| 控制饱和诊断 | `VisionComm_UpdateControlDiag()` 已实现 | 云台控制器未调用 |
| 二进制/文本调参命令 | `vision_cmd.c` 已实现 | 未初始化，也未从 USB 分流解析 |
| `send_interval_ms` 运行时设置 | 配置字段和命令已存在 | 发送周期仍使用 `.c` 内固定宏 |
| 配置保存/加载 | 函数存在 | 明确返回 `false`，未实现持久化 |

因此，不应按照旧文档直接发送 `set_filter_window`、`get_diag` 等文本命令并
期待生效；也不能把当前统计结构中的零值当成通信正常。

## 当前可执行的检查

### 1. 检查原始 USB 数据

- 确认 Jetson/上位机发送的帧长为 18 字节；
- SOF、命令 ID、CRC8、CRC16 和大小端必须符合
  [Seasky 协议说明](../protocols/seasky-vision.md)；
- 确认 `Src/usbd_cdc_if.c` 的接收路径实际触发；
- 使用 `TOPIC_VISION_DATA` 订阅点或断点确认基础帧已经发布。

### 2. 检查标准视觉消息

在 `adapters/vision/legacy_vision_bridge.c` 检查：

- 是否收到 `Vision_Recv_s`；
- 是否发布 `VisionTargetMessage`；
- `field_flags` 是否只标记真实存在的字段；
- 角度单位和方向是否与 `core/contracts/vision_messages.h` 一致。

### 3. 检查命令控制

在 `application/cmd/cmd_controller.c` 检查：

- `vision_valid` 是否置位；
- `VISION_CMD_TIMEOUT_MS` 超时后是否清零；
- `target_state` 是否允许当前控制行为；
- 云台命令是否仍被遥控器模式或急停逻辑覆盖。

### 4. 开启日志

将 `modules/logger/logger_config.h` 中 `LOG_ENABLE_VIS` 临时设为 `1`，然后：

```bash
python3 script/logger.py --tags VIS
```

注意：基础接收回调没有输出增强版 `VIS,RX,...` CSV；该日志只有增强回调真正
接入后才会生成。

## Jetson 接入原则

摄像头型号、识别网络和相机标定属于 Jetson 侧。MCU 侧新协议应实现独立
adapter，并转换成 `VisionTargetMessage`，不要让 Jetson 的线协议结构体直接
进入 `CmdController`。

在协议未知时只保留 `adapters/vision/jetson_vision_port.c` 占位，必须确认：

- 物理传输、帧版本、长度、大小端和校验；
- 时间戳来源、序号和超时策略；
- 坐标系、角度方向和单位；
- 距离、速度、置信度等可选字段的有效性。

## 后续接入注意

底层目录当前冻结，不能直接修改 `Src/usbd_cdc_if.c`。如需启用增强回调或命令
分流，应先设计位于上层的兼容入口；如果无法在不修改底层的前提下可靠接入，
必须由项目负责人明确解除相应文件的冻结限制后再实施。
