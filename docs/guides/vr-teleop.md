# VR 经裁判系统图传遥控底盘

本文描述保留的 VT02/VT12 虚拟串口方案。2026 官方选手端与 VT03/VT13 的
MQTT 部署、接收适配和验证边界见 [MQTT 链路](vr-mqtt.md)。

## 用途与适用范围

本方案把 Quest 的右手柄位移变成步兵底盘的前后、左右输入，用于实验室开发、
架空底盘检查、原版选手端串口接入和图传链路联调。当前使用 VT02/VT12，
控制对象是车辆底盘，不是机械臂；不包含 IK、yaw 遥控、抓取或自主导航。

```text
Quest / FrankaBot
  → USB / ADB reverse：8125 位姿，8100 B 状态
  → Python 解析、原有校准、扳机与位移映射
  → COM21 → VSPE Pair → COM22
  → 原版 RoboMasterClient 11.0.1.128 串口助手
  → 真实 VT12 USB COM11 → 无线图传 → VT02 UART
  → C 板 UART1（USART6，RX=PG9，115200 / 8N1）
  → VRD2 解码 → TOPIC_RC_UPDATE → 原底盘控制链
```

COM11 是本次实机端口，不保证其他电脑相同。Jetson、SSH 和 ST-Link 仅用于构建、
已有烧录及只读诊断，不负责转发 VR 命令，也不写入测试轴量伪造接收。
当前路径不使用早期本地文件 mailbox，不使用 MQTT，也不修改官方选手端二进制。

## 当前软件与硬件

| 项目 | 本次使用值 |
| --- | --- |
| 主机 | Windows 11 x64，安全启动、内存完整性保持开启 |
| Quest 应用 | 已安装的 `com.Xigbee.FrankaBot` |
| 头显枚举 | 本次 ADB 报告 Quest_3S；用户称 Quest 3 |
| 原版选手端 | RoboMasterClient 11.0.1.128（2025 版本） |
| 虚拟串口 | VSPE 1.5.8.489 x64，COM21/COM22 Pair |
| 图传 | VT12 电脑端、VT02 车端 |
| 固件基线 | `without-shoot-motors` 的 a816dd5，加 UART 接收与 VR 输入适配 |
| 车型 | Infantry；当前配置不含拨弹与摩擦轮电机，保留四轮和 yaw/pitch |

`VR` 分支同时保留原有视觉工具。活动固件的 ELF/BIN/HEX/MAP 放在
`tools/vr/firmware/`，与重新构建产物分开保存。重新构建会因路径/调试信息等产生
不同 ELF 哈希；判断板上版本不能只看分支名。

## 新电脑部署

1. 安装带 tkinter 的 Python 3（本次现场为 3.14）和 Android Platform Tools。
   在 `tools/vr` 中执行 `scripts/bootstrap.ps1`。脚本创建局部 `.venv`，
   安装固定版本 numpy/pyzmq/pyserial，并生成 Git 忽略的 `config.local.json`。
2. 编辑该配置：原版选手端根目录、ADB 路径、可选 Quest 序列号、虚拟串口两端和
   只读诊断 SSH 别名。序列号为空时，ADB 必须能唯一选择设备。配置不含登录密码。
3. 独立安装原版选手端及厂商依赖。`pc/virtual-serial/stock-hashes-before.json`
   保存本次六个关键 EXE/DLL 的相对路径和哈希，启动器拒绝不匹配版本。
   不要为了通过检查直接改掉期望哈希；先核对新的官方安装包和兼容性。
4. 执行 `scripts/download-vspe.ps1` 下载厂商安装包并检查数字签名。
   用管理员 PowerShell 执行 `pc/virtual-serial/setup-virtual-serial.ps1 -Step Install`。
   在 VSPE 界面手动创建 Pair，端口 COM21、COM22，并保持运行。
5. 本版原版 `DeviceInfo.dll` 不是枚举任意 COM 口，而是筛选硬件标识含 `FTDI`
   的设备。用管理员 PowerShell 执行
   `pc/virtual-serial/setup-virtual-serial.ps1 -Step Alias -ClientPort 22`，创建
   `VR Virtual Serial Discovery (COM22)` 兼容识别节点。它的硬件标识是
   `VRFTDI\CUSTOMCONTROLLER`，仅提供识别元数据，不传输字节、不加载 FTDI 驱动。
   真正的字节传输由 VSPE 完成。它可能在设备管理器显示为无功能驱动节点。
6. 执行 `.venv/Scripts/python.exe pc/virtual-serial/discovery.py --require-controller 22`。
   应返回 custom_controller=22；确认旧图传实际 COM 号。
7. 原版选手端关闭时，可执行 `.venv/Scripts/python.exe pc/virtual-serial/test_pair.py`，
   验证虚拟口字节一致。该检查会占用两端，不能与选手端同时执行。
8. 执行 `pc/virtual-serial/start-stock-client.ps1`。在界面按 P 选择老图传，
   使用与主控一致的机器人 ID 正常登录，确认自定义控制器允许转发。
   原版 `isCCValid` 检查保留；串口绿灯不等于控制权限已开启。
9. 连接 Quest，允许 USB 调试。执行
   `.venv/Scripts/python.exe scripts/prepare_quest.py --launch`，显式建立 ADB reverse；
   只在应用尚未运行时启动 FrankaBot，不修改头显应用配置，不安装 APK。
10. 执行 `pc/start-frankabot-vehicle-live.ps1`。状态窗口自动打开。
    `pc/virtual-serial/show-vr-status.ps1` 可以单独打开只读状态窗口；
    `pc/virtual-serial/stop-virtual-vr.ps1` 只停止本适配器，保留选手端和 VSPE。

如果已有现场程序运行，先退出它再从此目录启动，不要让两个程序争用同一组 Quest
端口或 COM21。本包没有自动修改当前桌面快捷方式，也没有自动烧录固件。

## 校准与键位

继续使用本次保存的 `quest_lab_current.json`：SHA256
`0351fa415d72f769a8a4368dc01d35e5db3f8f27a5f90a4dff53af2c0b6faa61`。
原版校准 HTML 与坐标计算源码在 vendor，Windows 启动适配在 quest_runtime。
需要重新校准时，先停止 VR 控制，再运行 `pc/start-existing-quest-calibration.ps1`；
完成并关闭校准接收后再启动控制。不会自动重新标定或覆盖旧校准。

- B 状态先 Low，再 High，才允许本次输入；启动时已经 High 不直接接管。
- 松开右手食指扳机记录手柄中立位置，再按住扳机并平移手柄。
- 前后、左右两轴连续输出；松开扳机发零指令，进程继续监听。
- 中心死区 3 cm；超出死区再移动 12 cm 达到幅值 100 RC 刻度。
  这里不是 m/s。两轴采用径向限幅，发送约 20 Hz。
- 位姿超过 120 ms、B 状态超过 200 ms 或追踪无效时撤销运动输入；
  恢复后需要重新松开扳机再接管。不要通过放宽失效时间掩盖追踪问题。

状态窗口显示追踪/B/扳机、最终指令、相对中立位移和实际串口写入计数。
“刷新 C 板计数”通过独立 SSH/ST-Link 只读检查；不是驱动指令路径。

## 线协议与下位机映射

串口帧：`A5 | length(u16 LE)=30 | header_seq(u8) | CRC8 | cmd(u16 LE)=0x0302 |
payload(30) | CRC16(u16 LE)`，共 39 字节。CRC8 初值 FF、多项式 8C；
CRC16 初值 FFFF、多项式 8408，均为反射移位算法。

| payload 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0..3 | 4 bytes | ASCII `VRD2` |
| 4..5 | u16 LE | 控制序号，模 65536 |
| 6 | u8 | enable，0/1 |
| 7 | u8 | diagnostic，0/1；诊断帧必须 enable=0 且轴量为零 |
| 8..9 | i16 LE | forward，范围 -100..100 |
| 10..11 | i16 LE | right，范围 -100..100 |
| 12..29 | 18 bytes | 保留，必须全零 |

接收代码在 `bsp/remote/bsp_rc.c/.h` 和 `modules/remote/remote_control.c`。
它用 USART6 单字节中断与 256 字节环形队列接收，再在消息中心派发钩子中解析。
VR forward→`rc.ch[3]`，right→`rc.ch[2]`，两拨杆 DOWN，其余输入为零。
原路由产生 `vx=ch3/660`、`vy=-ch2/660`。物理轮子方向仍须实车验证。

有效物理 DBUS 在 200 ms 内优先。VR 断流超过 200 ms 后需先收到禁用零帧重新接管；
拒绝重复/旧序号，保留原控制器失联、回中、CAN 健康和输出解锁机制。

## 验证边界与局限性

用户确认本次配置与代码可用，归档时硬件已全部拔除。以下区分用户反馈、已保存的测量证据和后续适用边界；归档检查不重新连接或驱动硬件。

- 本地虚拟串口 10 帧/390 字节完全一致；正常登录后两批诊断帧 10+50，
  C 板有效帧计数分别增加到 10、60，VR 发布计数保持 0。见 `verification.json`。
- 真实 Quest 位姿、B、扳机已到上位机；上位机日志出现过非零指令，例如
  forward=52/right=85。真实输入接通后的 C 板接收和发布尝试计数增长。
  尚未同步记录四方向非零轴量在 C 板的逐帧对应，也未完成原版虚拟串口路径的
  实际电机方向/速度/停止距离验收，不把计数增长描述为车辆已经运动。
- 追踪过程中出现过大量无效位姿和短时丢帧。状态窗口会明确显示，控制撤销后
  重新松开扳机接管；不能把当前结果视为稳定、低延迟、带载遥控已验收。
- `vr_uart_valid_count` 是通过帧和内容校验的次数，包含后续因重复序号/优先级被拒绝的帧；
  `vr_uart_publish_count` 在入队前递增，表示发布尝试，不是消息交付或电机执行 ACK。
  环形队列满会丢字节，当前没有专门的 UART6 溢出统计。
- 当前 VR 只给 yaw/pitch 零输入，并没有删除或禁用原云台闭环；云台可能保持原目标。
  此车参数和无发射电机配置不应直接套到其他车型。
- VSPE x64 评估版可手动创建设备，但配置文件加载、命令行创建设备和长期授权存在
  厂商限制。重启后的恢复需要重新检查；没有购买或绕过许可证。
- 虚拟设备兼容识别节点改变了 Windows 设备配置。官方文件哈希未变不代表比赛批准；
  正式操作间是否允许安装驱动、运行上位机、使用 VR 无线手柄等须由对应赛季规则及
  组委会明确确认。本方案只声明本机实验结果。
- 本次没有验证 12.x/2026、VT03/VT13 或 MQTT `CustomControl/0x0311`。
  本包的接收器仅处理这里约定的 `0x0302`，升级版本不能直接承诺兼容。
- CRC 与魔数不是认证；当前协议无应用层接收 ACK、重传确认或独立急停认证。
- SSH 私钥、登录信息、头显应用偏好、运行日志、原始轨迹、RAM dump 和个人配置均不提交。
  厂商 APK/EXE/DLL/驱动不包含在源码包中；运行环境仍需要独立准备。

## RS-232 实体替代方式

若不使用虚拟驱动，可选 FTDI FT232R 芯片、DB9、115200/8N1 的 USB→RS-232
成品转接线，确保包含 RS-232 电平转换，不能用 USB-TTL 线直接替代。
同一台电脑测试需两根线，以 Null Modem 交叉线连接，VR 写 A、原版选手端读 B。
典型 DTE DB9 接法 2↔3、3↔2、5↔5，但必须核对实际线材手册和接口公母。
比赛已有 RS-232 输入时通常只需自己计算设备一侧的转接线与匹配连接线。
RS-232 不直接接 C 板 TTL UART。购买转接线不能保证新赛季整条链路兼容。

## 回退与验证命令

关闭选手端后，以管理员运行 `setup-virtual-serial.ps1 -Step RemoveAlias`，
只删除记录中的 VR 根设备。VSPE Pair 在 VSPE 中单独删除；不触碰真实 FTDI/DJI 设备。

Python 离线测试：`python -m unittest discover -s tools/vr/tests -v`。
实际 C 解码互操作：`python tools/vr/tests/test_c_receiver.py`（需要主机 C 编译器）。
完整主机回归：`CC=gcc sh tests/host/run_tests.sh`。
固件：按 `docs/quickstart.md` 配置本机工具后分别构建 infantry_standard、sentry_swerve。
构建/测试不会自动烧录、重启开发板或驱动电机。

## 外部资料

- [VSPE 下载与当前许可](https://eterlogic.com/Products.VSPE_Download.html)、[许可限制](https://eterlogic.com/Products.VSPE_License.html)
- [FTDI US232R 数据手册](https://ftdichip.com/wp-content/uploads/2023/07/DS_US232R-10_R-100-500.pdf)
- [原版 2025 赛事引擎版本来源](https://www.robomaster.com/zh-CN/products/components/detail/6145)
- [2026 官方通信协议](https://bbs-web-static.robomaster.com/ef4f084944e34393aa70378a4a405c681774586313231/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V1.3.0%EF%BC%8820260327%EF%BC%89.pdf)
