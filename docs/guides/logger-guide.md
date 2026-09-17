# Logger 使用指南

本文说明固件日志接口、主机端串口工具和 RTT 仪表盘入口。
串口实现以 `modules/logger/` 和 `script/logger.py` 为准，
`just logger` 启动的是 `scripts/dashboard/rtt_ws_bridge.py` 网页仪表盘。

## RTT 仪表盘的 Python 环境

在仓库根目录安装依赖，并在每个新终端激活项目环境：

```bash
uv pip install --python .venv/bin/python -r requirements.txt
source tools/activate.sh
python -m pyocd pack install stm32f407ighx
just logger --help
just logger
```

`just logger` 沿用启动 `just` 时 PATH 中的 `python3`，因此只给其他 Python
安装依赖不能解决 `ModuleNotFoundError: No module named 'websockets'`。
根目录 `requirements.in` 已包含 `scripts/requirement.txt` 的仪表盘依赖，
`requirements.txt` 保存解析后的版本；网页入口依赖 `websockets`，探针连接依赖
`pyocd`，终端入口 `just logger-cli` 可使用 `rich`。

桥接使用新版 asyncio 服务接口，所以 `websockets` 下限为 14.0，见
[官方迁移说明](https://websockets.readthedocs.io/en/stable/howto/upgrade.html)。
`--help` 只验证启动和参数，不连接板卡；实际曲线仍要求匹配的固件 RTT 输出及可用探针。
默认网页地址为 `http://127.0.0.1:8080/`。

本机 ST-Link 使用 pyOCD 后端；`just logger --backend pyocd` 可显式选择。
STM32F4 支持包与 Python 包分开安装，见
[pyOCD 芯片支持说明](https://pyocd.io/docs/target_support.html)。脚本将默认短型号
`STM32F407IG` 映射到 `.ioc` 中 H 封装对应的 `stm32f407ighx`；未知型号仍需显式指定。
自动模式只在检测到 J-Link 时使用它，ST-Link 不需要安装 J-Link 动态库。

默认 `--connect normal` 对 pyOCD 表示 `attach`，保持 CPU 原来的运行状态；
目标已暂停时退出提示，不自动恢复。显式 `halt` / `under-reset` 会干预执行后恢复运行，
不用于日常监控。RTT 消费数据会更新上行缓冲区的读指针，不是完全无写入的内存观察。

若连接后报 `Control block not found`，先确认板上固件包含并初始化了 RTT。
09-16 只读比对确认本机板上仍是没有 RTT 的旧镜像；缺失的 SEGGER 源码已补齐，
新构建包含 `_SEGGER_RTT` 和 `Dashboard_Step`。更新板上程序后再开仪表盘：

```bash
just flash
just logger
```

`just flash` 会先构建并校验，烧录成功后按本机配置复位运行；不要同时运行其他占用探针的工具。

## 曲线和状态的含义（遥测 v8）

更新固件后关闭旧 logger，执行 `just flash`、`just logger`，再刷新网页。
仅重启网页无法补齐旧固件未发送的数据；网页会提示旧版数据缺少有效性标记。
桥接会告诉网页实际 WebSocket 端口，支持默认端口、单端口和自定义端口。

- `IMU & Gimbal` 显示 yaw/pitch 角度（度）和速度（度/秒）；yaw 使用连续角度，
  pitch 使用单圈编码角度。速度目标来自当前内环；spin 下 yaw 实际速度也是内环使用的 IMU 反馈。
- `speed_loop_only=true` 时 yaw 位置目标没有参与控制，因此位置目标曲线留空，
  观察速度目标/实际曲线。停用控制或反馈过期时相应曲线留空，不画假零值。
- `Motors` 按实际软件电机编号标注，与固件速度数组同序，速度为转子 RPM；
  `Chassis Cmd` 是归一化命令，不是米/秒。目标是控制器请求，不等于已发送电流。
- `CAN1 RX` / `CAN2 RX` 的 ON 表示该总线配置的电机在最近 100 ms 有反馈，
  OFF 表示没有满足条件的反馈；它不是 CAN bus-off 标志，也不表示遥控已解锁。
- 遥控消息超过 200 ms、IMU 消息/电机反馈/云台快照超过 100 ms 标为过期。
  IMU 状态仅证明应用消息近期更新，不代表独立验证传感器健康。
- 无效/未接入浮点数在日志中为 `nan`，在图上断线；裁判、视觉、射击等尚未接入，
  页面显示 N/A。v8 尾部附带能力位、轴状态、底盘电机 ID、云台使能和启动状态，
  主机日志保存这些字段；旧版解析保留兼容。
- 网页超过 1 秒没有新帧或连接断开会提示数据未更新；历史曲线仍保留。
  样本间隔超过 250 ms 不连线，避免把传输空窗画成连续运动。

## 当前能力

- 12 个标签：`SYS`、`CMD`、`CHA`、`GIM`、`SHO`、`SEN`、`MOT`、
  `IMU`、`CAN`、`VIS`、`RC`、`DEBUG`。
- 5 个级别：`ERROR`、`WARN`、`INFO`、`DEBUG`、`CSV`。
- 每个标签可编译期开关，并可设置独立的运行时限流间隔。
- 文本格式为 `[TAG][LEVEL] message`，CSV 格式为
  `TAG,timestamp_ms,fields...`。

## 固件端

包含头文件：

```c
#include "logger.h"
```

常用接口：

```c
LOG_ERROR(LOG_TAG_CAN, "CAN start failed: %d", status);
LOG_WARN(LOG_TAG_VIS, "Vision timeout: %lu ms", elapsed_ms);
LOG_INFO(LOG_TAG_SYS, "System ready");
LOG_DEBUG(LOG_TAG_MOT, "motor=%u speed=%.1f", id, speed);
LOG_CSV(LOG_TAG_GIM, "YAW,%.2f,%.2f", target, feedback);
```

编译期开关位于 `modules/logger/logger_config.h`。当前默认只启用
`LOG_ENABLE_GIM`，其余标签为关闭状态。需要某个标签时将对应宏改为 `1`；
大量日志会占用 USB CDC 带宽，调试结束后应关闭无关标签。

初始化和默认限流由 `Src/main.c` 完成。上层模块如需调整，可调用：

```c
Logger_SetRate(LOG_TAG_GIM, 50);  // 最短间隔 50 ms，即最高约 20 Hz
```

间隔为 `0` 表示不进行 Logger 限流，不代表底层传输一定能无损承载全部输出。

## 主机端串口工具

当前输出链路是 `Logger -> Debug_SendString -> BspUsb_Write -> USB CDC`，
需要 C 板自己的 USB 数据口。ST-Link 的 USB 负责 SWD，其自带 VCP 是另一条 UART，
不会自动转发此固件的 USB 日志。2026-09-09 本机仅枚举到 ST-Link VCP
`/dev/ttyACM0`，未枚举到 C 板 CDC，尚不能据此读取板上日志。

先按 [环境指南](../quickstart.md) 安装最新工具和 Python 依赖，再列出串口、
结合设备描述与插拔前后变化确认 C 板端口。不要依赖脚本自动选口：它可能选中 ST-Link。

仓库中的有效脚本名称是 `script/logger.py`，不是旧文档中的
`script/smart_logger.py`。

```bash
source tools/activate.sh
python -m serial.tools.list_ports -v

# 将路径替换为确认属于 C 板 USB CDC 的真实端口
python script/logger.py /dev/serial/by-id/实际C板设备 --tags GIM
python script/logger.py /dev/serial/by-id/实际C板设备 --tags GIM --save gimbal_session.csv

# 查看标签
python script/logger.py --list-tags
```

脚本支持 `--save` 和 `--auto-save`，不支持旧说明中的 `--no-plot`。脚本本身
只负责终端显示和保存，不提供曲线绘图。

`--tags all` 只取消主机过滤，不能开启固件中关闭的日志。当前仅 GIM 编译开启；
补偿路径有 `COMPENSATION` CSV，若干 YAW/ENCODER 日志调用仍被注释。
有日志不等于有所有电机反馈，没有日志也不能单独证明某个回调未执行。

## 查看消息回调和变量（OpenOCD + GDB）

当前是裸机 `MsgCenter_Dispatch()` 调用订阅回调，不是 FreeRTOS 任务。
USB logger 只接收主动输出的串口日志，不能直接订阅 MCU 内部消息中心；
RTT 仪表盘使用上方独立入口。需要观察 `on_gimbal_cmd`、`on_imu_update` 的参数、
调用栈或变量时，可以用工具链自带的 `arm-none-eabi-gdb`。

**断点会暂停控制循环，可能留下电机最后一次输出。先隔离动力输出再调试。**
下面是手动调试示例，不会由 `just doctor` 启动。不要与烧录工具同时占用探针。
先确保目标已完整烧录并校验了与所选 Debug ELF 完全一致的固件；GDB 加载符号
不会自动烧录，也不能证明板上镜像一致。

```bash
source tools/activate.sh
# 替换实际探针序列号；仅监听本机，关闭额外服务
openocd -f tools/openocd/stm32f407-stlink.cfg \
  -c 'fw_select_serial 实际探针序列号' \
  -c 'bindto 127.0.0.1; tcl port disabled; telnet port disabled'
```

另一个激活环境的终端运行 `arm-none-eabi-gdb /本次构建产物的完整路径/固件.elf`，
ELF 路径以 `just build` 输出为准。在 GDB 中：

```text
target extended-remote localhost:3333
monitor halt
break gimbal_controller.c:on_gimbal_cmd
continue
# 命中断点后再执行以下命令
bt
print *ev
```

`continue` 会运行目标；只有消息实际到达，断点才会命中。此流程尚未实机验证，
本轮仅验证了 GDB 能启动及 OpenOCD 的 SWD 身份读取；未暂停、复位或运行目标。
高频断点会改变时序，不适合据此测量正常控制周期。

## 常见问题

1. 没有输出：先确认固件已完整校验并在运行、端口是 C 板 CDC，再检查对应
   `LOG_ENABLE_*` 和调用路径。默认 `just flash` 校验后暂停，不会开始日志输出。
2. 找不到串口：显式传入 `/dev/ttyACM*`、`/dev/cu.usbmodem*` 或 Windows COM 口。
3. 乱码：确认主机端波特率与实际串口配置一致；USB CDC 虚拟串口通常不依赖
   物理 UART 波特率，但工具仍要求一个参数。
4. 丢行或控制变慢：增大 `Logger_SetRate()` 间隔，并关闭无关标签。
5. CSV 没有预期字段：检查调用处的 `LOG_CSV` 格式；脚本只对部分已知标签
   提供表头，其余数据仍会原样记录。

## 历史脚本

`script/deprecated/` 保存旧绘图脚本，仅供参考。它们没有随当前消息契约和目录
结构持续维护，不能作为现行调试入口。
