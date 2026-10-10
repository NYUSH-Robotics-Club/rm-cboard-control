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
仍读取`vision_speed_rpm`。pitch共用现有PID、单一速度阻尼、负载配置和越界停机/稳定100ms后自动恢复保护，
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

### 离线链路仿真

```bash
sh tests/host/run_vision_link_sim.sh
```

该主机检查使用固件现有的 USB 接收函数、Seasky 解码器、旧协议桥接器、
消息中心和命令路由器，模拟有效帧、损坏帧、20 ms 超时、无效角度和遥控失联。
它检查到 `GimbalCmd`，不模拟相机识别、USB 硬件、云台动力学或电机运动。
云台目标逐帧更新和限位另由 `tests/host/test_control_recovery.c` 覆盖。

已导入的 `nyu-vision` 当前 `io::Gimbal` 发出 29 字节 `SP` 帧：
模式字节加 yaw/pitch 角度、速度和加速度。`Aimer::aim()` 输出的是世界系
瞄准角。C 板当前 USB 接收只接受 18 字节 `0xA5` Seasky 帧，其中
pitch/yaw 是相对当前电机位置的弧度误差；C 板回传也采用 Seasky 帧，
与 `nyu-vision` 期望的 `SP` 四元数状态帧不同。离线仿真确认 `SP` 帧
不会产生视觉目标，因此两端尚不能直接运行同一条自瞄链。接入前须确定
双向协议、角度坐标系和符号、时间基准、失联与开火语义，并对照实车验证。

### 合成闭环自动调参

```bash
python3 tools/vision_closed_loop_tuner.py \
  --output /tmp/vision-tuner.json --trace /tmp/vision-tuner.csv
```

该程序在合成目标轨迹和一阶虚拟云台上搜索参数，并用未参与搜索的轨迹
逐场景复核。报告保留基线、候选和是否接受；CSV 可用于绘图。其速度、
加速度与延迟是演示模型的假设，`gain_per_s`、`damping` 和
`speed_limit_deg_s` **不对应** 固件配置字段。仓库目前没有匹配现行
自瞄链的实测轨迹用于标定，因此报告只证明自动评估流程能运行，不能
据此修改 C 板参数或宣称实车跟踪提升。

### 实测数据采集

为把合成仿真推进到实测闭环，需要为每次试验保留同一时间轴上的图像帧号、
拍摄时间、曝光设置、视觉结果及其发送时间，以及 C 板接收时间、控制模式、
双轴目标与实际位置/速度、输出饱和状态。记录时间戳的时钟来源和同步方法，
并将相机采集、识别、通信、控制和云台响应的延迟分别测量；若评估击打，
还需单独测量发射到命中的时间。按采集场次保存原始图像和日志，包含目标丢失、
切换、反向与不同光照条件，避免只用连续相邻帧验证检测器。

这些是后续采集要求，现有演示视频与合成模型均不含上述完整时间链。
相机曝光、标定、坐标方向和每段延迟须以所用硬件实测确定，不能套用
其他赛季或队伍的经验数值。

### 标注数据评估与固件候选升级

[`LabelRoboMaster`](https://github.com/xinyang-go/LabelRoboMaster) 是标注工具；
其每张图片对应的 `.txt` 每行有类别编号和四个归一化角点。拿到经人工检查的
标签，以及模型对同批图片导出的候选框 CSV 后，可以运行：

```bash
git clone --depth 1 https://github.com/xinyang-go/LabelRoboMaster.git /tmp/LabelRoboMaster
python3 tools/vision_extract_label_frames.py nyu-vision/assets/demo/demo.avi \
  --output build/vision-labeling/demo --interval-s 5
python3 tools/vision_prelabel.py --images build/vision-labeling/demo \
  --model /tmp/LabelRoboMaster/resource/model-opt.onnx \
  --output build/vision-labeling/demo-suggestions
```

人工校对标签后，还需从**待优化的 `nyu-vision` 检测器**导出同批图片的
原始候选框，才能调该检测器的阈值：

```bash
python3 -m pip install --target /tmp/vision-onnxruntime --no-deps onnxruntime
PYTHONPATH=/tmp/vision-onnxruntime python3 tools/nyu_vision_export_predictions.py \
  --images /path/to/labelled-images \
  --config nyu-vision/configs/odin.yaml \
  --model nyu-vision/assets/yolov5.onnx \
  --output /tmp/nyu-detector-predictions.csv
python3 tools/labelrobomaster_eval.py /tmp/nyu-detector-predictions.csv \
  --labels /path/to/labelled-images --output /tmp/armor-events.csv
python3 tools/vision_detection_tuner.py /tmp/armor-events.csv \
  --config nyu-vision/configs/odin.yaml \
  --report /tmp/armor-threshold.json \
  --candidate-config /tmp/odin-candidate.yaml
```

`predictions.csv` 必须包含 `split,image,confidence,x1,y1,x2,y2,label_class`，
坐标归一化到 0～1；无检测的图片留一行空置信度。`split` 为 `train` 或
`holdout`，应按采集场次分组。`label_class` 使用标注工具的 0～35 编号，
不能直接套用 `nyu-vision` 内部的颜色/名称枚举；也可改用
`color,name,armor_type` 三列，由转换程序映射。评估程序按类别和框 IoU
配对；候选框应从模型输出、应用当前 `min_confidence` 前导出，否则
已过滤的检测无法恢复。程序再搜索 `min_confidence`；只有留出集 F1 提升且召回基本不下降时
才写独立候选 YAML。模型权重不在此流程中训练。
抽出的演示帧在 `build/vision-labeling/demo/`，初始状态均为未标注；
演示视频只能用于熟悉标注流程，不能作为独立的实车留出集。
预标注只生成 `.suggested.txt` 和空 `split` 的预测 CSV，须人工逐图核对
并另存同名 `.txt` 真值、按采集场次填写划分后才可评估。
`vision_prelabel.py` 使用的是标注工具自带模型；其预测不能用来调
`nyu-vision` 检测器的置信度阈值。
ONNX 主机回放已可导出候选框，但 `odin.yaml` 实车路径使用 TensorRT
并可回退 OpenVINO XML，且还包含传统角点修正；使用结果前须核对模型
版本、后端输出和实车图像域。独立测试集不能来自同一段演示视频。
标注工具对无目标图片会删除同名 `.txt`；只有确认所有图片均已人工复核时，
才可在匹配脚本中加 `--missing-empty` 将缺失标签视为负样本。

`tools/vision_firmware_promote.py` 只接受步兵车型、当前配置哈希匹配、
有安全边界的 `vision_speed_rpm` 候选，以及至少两个场景、每场景三个
成对实测回合的基线/候选试验。默认仅输出检查报告；加 `--apply` 才修改
上层车型配置，并运行主机回归和步兵 ARM 构建，失败则恢复源文件。
它不烧录板卡。合成仿真报告和旧固件日志不能作为这些实测证据。
升级提案 JSON 需含 `source="measured_board_ab"`、
`robot_type="infantry_standard"`、`config_sha256`、`candidate_rpm`、
`bounds_rpm`、`trials_csv`、`trials_sha256`；试验 CSV 列为
`variant,scenario,session,yaw_mae_deg,yaw_peak_deg,saturation_fraction,remote_loss_safe,elf_sha256`。
先运行 `python3 tools/vision_firmware_promote.py proposal.json` 查看结果，
通过后可用 `--apply` 升级配置。

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
