# 已验证配置与来源

这些信息是本次 J3507/C 板工作形成的可复用基准，不是其他电机或下次接线的默认值。
接入时重新读取实际参数；不硬编码端口、PID、运行状态或正在使用的控制模式。

## 本机位置

| 用途 | 当前路径/标识 |
| --- | --- |
| C 板仓库 | `/Users/pengyue/Codespace/rm-cboard-control`，`Diagnosis` 分支 |
| RTT 接入及低速工具 | 该仓库 `tools/diagnosis/` |
| 电脑直连工程 | `/Users/pengyue/Codespace/damiao-3507` |
| 直连 Python 环境 | 上述目录 `.venv`；SDK 在 `vendor/DM_CAN.py` |
| C 板当前架构起点 | 提交 38cf20c 引入双向 RTT；e8181d5 引入受监测的低速工具 |
| 原固件备份 | 仓库 `logs/board-backup/before-diagnosis-20261117.bin`，1 MiB，忽略目录 |
| RTT 最终只读实测 | 仓库 `logs/board-backup/can1-rtt-final-20261117.json` |
| 电脑直连上下文 | Codex 聊天 `01a1e9b4-23b3-71b3-9d96-e3d89c01ed54` |

文件可能不随 clone 存在。先检查当前路径，不存在时定位用户实际工程；不要凭本页
重建不存在的现场数据。Windows/Linux 可用各自的 Python/驱动/串口名执行同样流程，
本次实际验证的是 Apple Silicon Mac，不宣称所有操作系统与所有转接器都已实测。

## 实际 J3507 参数

| 项目 | 本次读取/观察 |
| --- | --- |
| 型号 | DM-J3507-2EC，主机 SDK 类型 DM3507 |
| 电机 ID / 反馈 ID | 1 / 0 |
| 经典 CAN 位速率 | 1 Mbps，can_br 寄存器读值 4 |
| 控制模式 | 只读验证时读值 1（MIT）；运行工具临时切为 3（速度），未保存参数 |
| P_MAX / V_MAX / T_MAX | 约 12.566 rad / 50 rad/s / 5 N·m，均为协议映射范围 |
| 硬件/固件版本（电脑直连读取） | 300V / 5717 |
| 已测试低速目标 | +0.1 rad/s，约 0.955 rpm；角度约每 2 s 增加 0.20 rad |

低速瞬时速度有量化/波动，连续角度增量比单帧速度更能说明平均转速。
这不证明带载性能、机械限位、急停时延或断线自动停机已经验证。低速脚本后续曾因
RTT 回复超时退出，零速/失能未确认；因此持续运行与停止都必须看当次新证据。

## 两条链路的实测结论

- **电脑直连**：原生 dm-device-sdk 加载成功但未枚举设备；实际 USB 为 2e88:4603
  HDSC CDC，使用 921600 串口与官方 u2can Python 库后读到参数/状态，并成功低速旋转。
  该路径跳过达妙助手。后续一次停止只确认命令已发送，未收到回复，不能作为已停证据。
- **C 板 RTT**：ST-Link V2 可实际打开 F407，匹配 ELF 的 RTT 控制块可读写；内部
  CAN1 静默回环有 TX_OK/RX。起初板子未完全上电，外部 CAN 读请求失败；完整上电后
  同一路径读通。最终独立一轮 8 次 TX_OK、8 次 RX/回调、错误/超时/TEC/REC 全为 0。
  读取到匹配参数及失能状态，随后经 RTT 的显式低速程序也观察到使能状态与连续角度增加。
- **架构选择**：用户明确选择 RTT 双向终端。曾尝试主机扫描自定义 RAM 状态，
  读取耗时约 3 ms 与状态刷新窗口冲突；该外部接口已移除。不要把它作为当前接入步骤，
  也不要把 RTT 的调试器缓冲搬运频率解释成电机轮询周期。

## 寄存器与兼容性

本台实际读通 RID 7=MST_ID、8=ESC_ID、10=CTRL_MODE、21/22/23=P/V/T、35=can_br。
J3507 英文 V1.0 手册附录 ESC_ID 一行十六进制/十进制栏有冲突；先前只读探测默认
选用明确的 RID 7，后以 SDK 与实机回复确认 RID 8，不仅凭那一行猜写参数。
不同固件的命令 ID/模式可能不同；其他型号必须拿到实际资料再支持。

协议来源：本机官方 SDK 副本与本次原始收发；
[J3507 V1.0 厂商手册镜像](https://f.foxtech.com/manual/DM-J3507-2EC-V1.0.pdf)、
[SEGGER RTT](https://kb.segger.com/RTT)、
[OpenOCD RTT 命令](https://openocd.org/doc/html/General-Commands.html#Real-Time-Transfer-_0028RTT_0029)。
