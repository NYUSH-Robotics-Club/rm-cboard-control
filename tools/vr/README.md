# Quest 手柄经原版选手端遥控步兵底盘

当前方案：FrankaBot → USB/ADB → Python 上位机 → COM21/COM22 虚拟串口对 →
**原版 RoboMasterClient 11.0.1.128** → VT12 → 无线 → VT02 → C 板 UART1。

适用于现有 VT02/VT12 的本地开发、架空底盘联调和链路验证。不是机械臂 IK、
MQTT 自定义客户端、自动驾驶或已获赛事批准的部署方案。

- [应用场景、完整部署、协议和局限性](../../docs/guides/vr-teleop.md)
- `pc/`：当前车辆映射、虚拟串口、原版程序识别、只读状态窗口及启动脚本。
- `quest_runtime/`：Windows Quest 接收适配器、原版校准页面启动适配和本次校准文件。
- `vendor/quest-trajectory-recorder/`：原版校准 HTML、解析器、坐标计算等上游源码。
- `scripts/`：新电脑环境准备、驱动下载、显式 USB 准备工具。
- `tests/`：离线映射及 Python→实际 C 接收器互操作验证。
- `firmware/`：当前已烧录镜像的 ELF/BIN/HEX/MAP 与哈希；完整 C 源码仍在仓库原目录。
- `verification.json`：两轮诊断帧接收记录及验证边界。
- `source-snapshot.json`：从工作现场收集的上位机原文件哈希。打包仅调整路径、配置和运行时输出目录。

首次使用：在 Windows PowerShell 执行 `scripts/bootstrap.ps1`，编辑
`config.local.json`，再按部署文档建立虚拟串口。VSPE 64 位评估版需要手动创建 Pair；
没有购买许可证，也没有禁用 Windows 安全启动或内存完整性。

已经配置好的电脑：先启动原版选手端并正常登录，再执行
`pc/start-frankabot-vehicle-live.ps1`。校准继续使用
`quest_runtime/repository_calibrations/quest_lab_current.json`；它属于本次场地，
换跟踪空间或面向后须由操作员确认是否仍有效。

原版选手端、FrankaBot APK、ADB、VSPE 和厂商驱动由各自官方/授权来源安装，
本目录不重新分发它们，也不包含早期修改过的 DJI DLL、EXE 或安装包。
