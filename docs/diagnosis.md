# C 板 ↔ 达妙 CAN 诊断

电脑用 ST-Link/SWD 连接 C 板，C 板通过自身 CAN 收发器连接电机。
电脑和 C 板之间没有额外 USB CDC/串口要求。ST-Link 本身不提供 CAN 接口。

## 接线与第一次连接

1. ST-Link 的 SWDIO、SWCLK、GND、目标电压参考按 C 板丝印接到调试口。
   C 板与电机各自使用适配的电源；不要依赖调试器给电机供电。
2. C 板所选 CAN 口的 H 接电机 CAN_H，L 接 CAN_L，信号地共地。
   使用板上的 CAN 插座，不把 MCU 的 PD0/PD1 或 PB5/PB6 逻辑引脚直接接总线。
   插头线序以两端丝印/手册为准，不按线色猜测。电机调试串口不是 CAN 口。
3. 核对总线两端终端电阻；不要给每个中间节点都增加终端。断电检查 H/L 电阻可
   帮助确认终端数量，具体是否已有板载终端须核对实物。
4. 首次上板先选内部静默回环发测试帧，验证内部路径。再将通道切到正常收发。
   只有两个节点时，只听模式不会 ACK 电机，不能据此判定线路坏了。

[DM-J3507-2EC 厂商手册 V1.0（2024-11-19，厂商文档镜像）](https://f.foxtech.com/manual/DM-J3507-2EC-V1.0.pdf)
给出 24 V 额定电源和默认 1 Mbps。手册第 7、10–11 页说明经典 CAN 参数读取与
波特率模式；电机改为超过 1 Mbps 后会用 CAN FD 回复，本 C 板无法接收。
因此未经达妙助手配置的电机可以先直接用 C 板按 1 Mbps 正常模式读取，实际设置
仍须由回复验证。若已处于 CAN FD，先用兼容设备恢复经典 CAN；工具不会自动改写它。

## 构建

需要 Python 3.10+、CMake 3.22+、Ninja、Arm GNU 工具链；实机需 OpenOCD 0.12+。
`arm-none-eabi-gcc`、`arm-none-eabi-nm`、`cmake`、`ninja`、`openocd` 均应在 PATH。
主机界面没有 pip 依赖，不需要旧分支的 Python 环境。

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
sh tests/host/run_tests.sh
```

也可用 `just build`、`just build Release`、`just test`。
构建产物在 `build/Debug/` 或 `build/Release/`，文件名为 `CBoard_Diagnosis.elf/.bin`。
旧 `ROBOT_TYPE` 不再选择整车；本分支不构建步兵或哨兵配置。

## 明确烧录，再连接网页

下面第一条命令会写板、校验并启动固件。它只在你明确执行时运行；连接网页不会
自动烧录。工具先检查目标 F405/407 家族 ID=0x413、Flash=1024 KiB，再复位并复核，
随后写入/校验，任何步骤失败即停止。确认所选物理目标确为你的 C 板。

```sh
python3 tools/diagnosis/manage.py flash --serial 实际STLink序列号
```

只有一个可识别的 ST-Link 时可省略 `--serial`；多个探针必须指定。
选择 Release 烧录时加 `--mode Release`，界面也必须使用匹配的 Release ELF。
可以用 `--openocd /完整路径/openocd` 指定程序；不要同时开着 CubeIDE 的调试会话。

烧录完成后，在两个终端分别执行：

```sh
# 终端 A：保持运行，独占探针；此命令不复位、不暂停、不烧录
python3 tools/diagnosis/manage.py connect --serial 实际STLink序列号
```

```sh
# 终端 B：保持运行，读取对应 ELF 的 SRAM 符号
python3 tools/diagnosis/interface.py --elf build/Debug/CBoard_Diagnosis.elf
```

浏览器打开 [本地诊断界面](http://127.0.0.1:8765)。可分别用 `just connect` 和
`just interface` 代替。目标若已暂停，需要操作者在调试器中明确恢复；工具不代为恢复。
自定义 TCP 端口时，两侧 `--tcl-port` 必须相同；网页端口可用 `--port` 更改。
主机启动时 ABI 不匹配会拒绝读取/写入。断连后重启连接和网页进程，不自动重试命令。

先不接硬件看界面：`python3 tools/diagnosis/interface.py --demo`，或 `just demo`。
演示所有操作都是模拟，不证明 ST-Link 或 CAN 正常。

## 排查顺序

1. **电脑 → C 板**：网页显示真实固件心跳持续变化。停住时先检查供电、SWD、
   固件是否运行、是否被 IDE 暂停。蓝灯约每 500 ms 翻转也可辅助判断。
2. **C 板内部**：选择通道、1 Mbps、内部静默回环，应用配置。发送例如标准 ID
   `0x123`、数据 `01 02 03 04`，应看到 TX_QUEUED、TX_OK 和同内容 RX。
   静默回环不向外部总线发出这条测试帧，也不验证收发器或线缆。
3. **C 板 → 电机**：将通道改为正常收发。点击「探测 ID 1–16」逐个发送参数读取，
   默认读取 RID 7（MST_ID/反馈 ID）。只扫描这 16 个 ID，不扫描所有地址；若无回复，
   还可手工读取 ID 0 或其他已知范围，或逐个试 500/250/200/125 kbps。
   选择不同波特率只改变 C 板，不改变电机配置。
4. **确认电机身份**：候选回复保留返回 CAN ID、载荷中的电机 ID、RID 和 uint32/
   float32 两种解释；应核对它们确实对应刚发出的读取。RID 7 可确认反馈 ID，
   RID 0x0A 可读控制模式，0x15/0x16/0x17 可读 P/V/T 映射范围。
   数据类型和 RID 以匹配的电机固件手册为准。J3507 V1.0 附录的 ESC_ID 一行十六进制
   与十进制栏存在冲突，因此本工具探测不依赖该行，也不会自动据此修改电机 ID。
5. **单次发帧**：核对完整命令 ID 与实际控制模式后填写原始报文，勾选确认再发送。
   失能按钮发七个 FF 加 FD，使用所填 ID；型号/固件兼容性和实际失能仍需回复及实物验证。
   不预设零点、使能后目标、力矩范围或控制增益。
6. **反馈解码**：填写已确认的反馈 CAN ID、Motor ID 和实际 P_MAX/V_MAX/T_MAX，
   再勾选解码。使用位置 16 位、速度/力矩 12 位的对称范围映射；不支持不同反馈格式，
   不自动把八字节参数回复当作电机反馈。反馈时间戳可检查是否已陈旧。

任意原始帧均可能运动或修改电机参数；没有自动循环控制或失联停机协议。
单次失能消息可能因线路/波特率/模式不匹配而失败，不能替代物理断电。
关闭网页或断开 ST-Link 不会保证电机停止。正常模式的 TX_OK 仅说明有节点 ACK，
不是特定电机已经理解/执行命令。

## 诊断指标

| 现象 | 可定位的范围 |
| --- | --- |
| ABI 不匹配 / 无心跳 | ELF、板上固件、暂停状态或 SWD 链路 |
| TX_QUEUED 无 TX_OK | 只有邮箱接收成功，继续看 TX_ERROR/TX_TIMEOUT |
| TX_ERROR，LEC=3 | ACK 错误；检查电机供电、线路、波特率、正常模式 |
| TX_OK，无电机回复 | 已有节点 ACK；核对电机 ID/模式、参数协议和反馈格式 |
| TX_TIMEOUT 后 BUSY | 取消尚未被硬件确认，禁止追加或重新配置；先检查硬件状态 |
| Bus-off / Error passive | TEC/REC/ESR 异常；排除根因后显式重新应用配置 |
| rx_dropped 或 MCU→主机遗漏增加 | FIFO/软件队列或 SWD 记录窗口跟不上，不能声称没有收到报文 |

ESR 的 LEC 为最近错误码，不是累计错误；`last_tx_tsr` 锁存最后发送结果，
TSR 当前值与 RF0R 也可查看。`rx_callbacks` 是接收回调次数，不等于帧数。
只听状态下没有帧不证明电机不存在：电机可能只在收到请求后回复。

网页最多保留最近 1000 条记录，MCU 环形记录 128 条。导出 JSON 包括这一窗口及
计数/寄存器；不是连续无损录波器。需要精确位时序、线电平或完整高负载抓包时，
还需外部 CAN 分析仪或示波器。

## 验证边界

主机测试包括服务请求校验、一次执行、环覆盖、HAL 替身下的正常/失败/超时/取消
竞争/溢出，以及网页鉴权、只读报文编码和不连接硬件的 OpenOCD 烧录失败路径。
构建、主机模拟、网页演示均不代表已烧录或实机通过。实板接线、CAN 电气层、
J3507 回复及运动行为须单独验证。
