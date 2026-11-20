# Quest 经官方 MQTT 与 VT03/VT13 遥控底盘

## 数据路径与范围

```text
Quest / FrankaBot → USB / ADB reverse → 已有校准与两轴位移映射
→ MQTT 3.1.1 / CustomControl / Protobuf
→ 官方 RoboMasterClient 12.0.0.213
→ VT13 USB → 无线图传 → VT03 UART / 0x0311
→ C 板 UART1（USART6 / RX PG9 / 921600 8N1）
→ 原 VRD2 校验与 RC 输入适配 → TOPIC_RC_UPDATE → 原底盘控制
```

SSH 和 ST-Link 仅承担部署与只读观测，不在驾驶指令路径中。不向 Jetson 的
网络或 C 板 RAM 写入驾驶轴量，不使用 mailbox、虚拟 COM、改版选手端或自建
MQTT 服务器替代图传。Windows 同机测试连接官方选手端已监听的
`127.0.0.1:3333`；独立设备按官方组网手册使用 `192.168.12.1:3333`。
这不表示已经验证独立设备组网，也不需要修改现有外网/裁判局域网的地址。

控制对象是底盘前后、左右平移，不是机械臂，不改变底盘、云台、PID、电机 ID、
CAN 或发射机构代码。原健康物理 DBUS 优先、200 ms 失联及松扳机重新接管机制保留。

## MQTT 与线协议

- Topic 为大小写精确的 `CustomControl`，载荷直接是 protobuf
  `message CustomControl { bytes data = 1; }`，最多 30 字节。
- `data` 是原 VRD2 的 30 字节业务数据，不包括 A5、串口命令码或串口 CRC。
  `pc/mqtt/custom_control.proto` 为对应最小 schema，运行时使用 protobuf 的标准
  descriptor/message API 编解码，避免依赖本机 protoc 版本。
- 当前现场 R1 的 MQTT Client ID 是字符串 `1`；不是选手端服务器内部登录编号
  `41`。更换机器人时须按实际机器人 ID 调整。原版 broker 拒绝错误 Client ID。
- 发布 20 Hz、QoS 0、retain=false、clean session；不进行应用层离线缓存或重放。
  协议最高允许 75 Hz/QoS 1，这里不需要达到上限。
- VT03 在 UART 上封装为 `A5 | len=30 | seq | CRC8 | cmd=0x0311 | data | CRC16`，
  共 39 字节。C 接收器兼容旧 `0x0302`，CRC、魔数、范围、保留位及序号检查保留。
- MQTT 发布成功和 socket 完成不等于 C 板接收或电机执行 ACK。状态窗口分别显示
  上位机发布数与人工刷新的 SWD 接收/发布尝试计数。没有应用层往返 ACK。

## Windows 部署与入口

1. 准备官方原版 2026 选手端、依赖及 DJI 驱动，正常连接 VT13。VT03/VT13 应先
   完成对频，VT03 绿灯每秒闪一次。主控 App `12.0.0.10`、电管 `12.0.0.4`、
   VT03 App `12.0.0.14`/核心 `12.0.0.6`、VT13 `12.0.0.3` 为官方配套最低版本。
2. 在 `tools/vr` 环境安装 `requirements.txt`。配置忽略提交的 `config.local.json`，
   在原配置之外增加：

   ```json
   {
     "python_path": "C:/path/to/.venv/Scripts/python.exe",
     "mqtt_host": "127.0.0.1",
     "mqtt_client_id": "1",
     "jetson_vr_repository": "/home/nyu/rm-cboard-control-vr",
     "jetson_mqtt_elf": "/home/nyu/rm-cboard-control-vr/build/mqtt-vt03/infantry_standard/NYUSH_Infantry.elf"
   }
   ```

   保留原 `adb_path`、`quest_serial`、`diagnostic_ssh_host` 等字段。
   若继续使用桌面上已有的校准入口，可把 `calibration_path` 设置为该入口实际保存的
   `quest_lab_current.json`；状态窗口显示实际加载文件的哈希，不依赖重复拷贝。
3. Quest 允许 USB 调试，执行 `python tools/vr/scripts/prepare_quest.py --launch`。
   只启动已有 FrankaBot，不安装 APK、不修改应用偏好。没有设备时监听仍保留；
   USB 准备在独立线程重试，不阻塞控制循环的失效处理。
4. 执行 `tools/vr/pc/mqtt/start.ps1`。它启动原样式手柄状态窗口和单实例 MQTT
   监听。`-NeutralOnly` 可显示真实输入但强制输出零。校准继续使用
   `quest_runtime/repository_calibrations/quest_lab_current.json`。
5. B 先 Low 后 High，松开食指扳机确定中立位置，再按住扳机移动。中心死区
   3 cm，额外位移 12 cm 达到 100 RC 刻度；这不是 m/s。松开扳机只归零，
   程序继续监听。停止程序执行 `pc/mqtt/stop.ps1`。
6. 重校准使用原入口 `pc/start-existing-quest-calibration.ps1`，先停止 MQTT
   监听，避免争用 Quest 端口。停止后无需退出官方选手端。
   方向采集每段水平位移至少 10 cm，向右与向前应接近垂直；适配器拒绝两段
   近乎共线的采集（夹角不足约 32° 或超过约 148°），即使原页面显示已保存。
   原 HTML 与求基算法保持原样，该检查只阻止含糊校准进入底盘控制。

运行时状态、日志和本机路径位于 `tools/vr/runtime/`、`config.local.json`，不提交。
桌面快捷方式属于本机部署配置，不是硬编码的源码依赖。

## Jetson 与 C 板

Jetson 部署在独立 VR 工作树，保留原活动工作树与其中未提交改动。
接收适配改动只在 `bsp/remote/`、`modules/remote/remote_control.c` 和顶层 CMake。
不修改 CubeMX/启动/驱动基线。CMake 参数 `VR_VTM_UART_BAUD` 默认 115200，
保留旧 VT02 构建行为；VT03 构建显式传入 `-DVR_VTM_UART_BAUD=921600`。
VT03 使用空闲的 DMA2 Stream1/channel5 与 1024 字节循环缓冲，DMA 完成中断
计数加当前位置共同跟踪写入进度；轮询检测缓冲覆盖。UART6 不再逐字节中断，
并在 BSP 中处理 UART 溢出标志，避免高速连续流导致接收停止。旧 VT02 构建保留
原单字节中断路径；其它 UART/DMA 用户不变。

```sh
cmake -S . -B build/mqtt-vt03/infantry_standard -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DROBOT_TYPE=infantry_standard -DVR_VTM_UART_BAUD=921600 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/mqtt-vt03/infantry_standard
CC=gcc sh tests/host/run_tests.sh
python3 tools/vr/tests/test_c_receiver.py
```

按本机工具配置选择 CMake/Ninja/GCC；新版本 xPack OpenOCD 的原生 ST-Link 驱动
可能需 `swd` 而非 `hla_swd`，不可混用运输模式。本现场已验证系统 OpenOCD 0.11
搭配 `interface/stlink.cfg` / `hla_swd`。烧录前保存原板完整 flash，写入后
`verify_image`，按 `docs/quickstart.md` 的部署边界管理复位与运行。

`scripts/read_board_status.py --elf <实际烧录的ELF> [--verify]` 只读解析 ELF 符号
并用 SWD 读取计数。`--verify` 另外校验 flash 与 ELF。脚本校验 MQTT 固件标记，
不沿用旧镜像固定 RAM 地址；默认不主动暂停/复位目标，不改 RAM。
`vr_uart_publish_count` 仍是入队前的发布尝试，不是电机执行确认。
`vr_uart_mqtt_count` 专门统计通过帧/内容校验的 `0x0311`，诊断帧也计入。
额外提供 UART 字节、错误、环形缓冲溢出、实际波特率与最后序号观测。

## 验证与限制

离线 Python 测试：`python -m unittest discover -s tools/vr/tests -v`。
真实 C 接收器测试覆盖旧/新命令、VT03 原生 RC 字节夹杂、CRC、分包、序号、
超时重新接管和 DBUS 优先；完整主机回归与两车型构建独立执行。

纯诊断使用 `pc/mqtt/bridge.py --diagnostic 50`。诊断帧 enable=0、轴量=0，
C 板在计数后返回，不发布运动输入。对比发送前后的 `0x0311` 计数与序号，
可以验证 MQTT → 官方选手端 → 无线 → C 板 UART。之后真实手柄四方向、松扳机
和断流恢复要分别检查，不把离线测试、网络连接或诊断计数等同于车辆运动验收。

随包的 `tools/vr/pc/mqtt/verification.json` 记录本机验收：50/50 诊断帧、末帧序号
对应、UART 无错误/覆盖；真实手柄运行时记录了四方向非零 C 板轴量，24 组非零
快照可在相邻两帧范围内与上位机轴量对应。操作员确认车体运动正常，松开后
轴量回零。SWD 逐项读取并非原子快照，不据此计算精确延迟；断流重连的长期稳定性、
停止距离和带载耐久性未做本轮量化验收。

源码保留原云台闭环，VR 不给 yaw/pitch 新指令并不意味着云台被禁用。
CRC/魔数不是身份认证，链路不保证每帧必达；SSH/SWD 只读快照也不是原子快照。
官方协议支持范围和赛场准入须按对应比赛规定确认，本方案验证的是实验室配置。

## 参考

- [官方 2026 通信协议：CustomControl / 0x0311](https://bbs-web-static.robomaster.com/ef4f084944e34393aa70378a4a405c681774586313231/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V1.3.0%EF%BC%8820260327%EF%BC%89.pdf)
- [官方 2026 学生版软件与组网文档](https://bbs.robomaster.com/wiki/20204847/804784)
- [ClearWei/RM26_Client：MQTT 3.1.1 与 protobuf schema](https://github.com/ClearWei/RM26_Client)
- [Eclipse Paho MQTT Python](https://github.com/eclipse-paho/paho.mqtt.python)

实现参考官方协议与开源接口，未复制第三方 Qt 应用代码或厂商二进制。
