# 构建、配置与烧录

本仓库的统一入口是 `justfile`，后端为 CMake、Ninja 和 Arm GNU Toolchain。
支持 `infantry_standard`、`sentry_swerve`。配置保存在 Git 忽略的
`.firmware.local.json`；不同电脑需要各自配置，不能从旧记录推断本机工具路径。

## 首次配置

先准备 Python 3.12+ 和项目虚拟环境，并安装 Python 依赖。Linux/macOS 示例：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python tools/bootstrap.py
.venv/bin/python tools/firmware.py configure infantry_standard --mode Debug --run-after yes --allow-single yes
source tools/activate.sh
just doctor
just build
just build sentry_swerve
```

Windows 可用 `py -3 -m venv .venv`，再用 `.venv/Scripts/python.exe`
运行安装、bootstrap 和 configure。macOS/Linux 也可以用已有的 uv 环境；
工具路径和探针可通过
`tools/firmware.py configure --help` 指定。Python 依赖从 `requirements.txt`
安装；直接依赖见 `requirements.in`。首次安装和升级工具时核对实际版本，
不要把旧机器的版本表当成当前要求。

## 日常命令

| 命令 | 行为 |
|---|---|
| `just doctor` | 检查本机工具、配置和探针枚举；不连接 MCU |
| `just build` | 构建本机保存的车型 |
| `just build sentry_swerve` | 本次构建哨兵，不改本机默认车型 |
| `CC=gcc sh tests/host/run_tests.sh` | 执行硬件无关的 C 测试 |
| `just flash-plan` | 显示烧录计划；不连接 MCU |
| `just flash` | 构建、检查目标、擦写并校验固件 |
| `just monitor` | 经 SWD 读取云台快照；不烧录 |
| `just logger` / `just logger-cli` | RTT 网页/终端遥测 |

默认烧录后端是 OpenOCD + ST-Link/SWD。烧录前先确认车型、目标板和
本机 `run_after` 配置：`yes` 会在成功校验后复位运行，`no` 会保持暂停。
不要根据旧日志中的默认值推断当前配置。多探针时应显式配置序列号。

`just flash` 是硬件写入：先确认 STM32F407 的设备身份和容量，
再擦写与 `verify_image`；只有校验成功才按 `run_after` 处理运行状态。
`just doctor`、`just flash-plan` 和主机测试不能证明目标板当前镜像、
接线、电机方向或整车行为。固件改动后分别记录源码提交、构建产物、
实际烧录结果和实车观测。

## 排查

- 找不到 `just`：激活 `tools/activate.sh` 或检查本机工具路径。
- 找不到 OpenOCD 配套脚本：检查发行包目录和本机配置。
- USB 可见但 SWD 连接失败：核对探针占用、目标供电、VREF/GND、
  SWDIO/SWCLK；`doctor` 枚举成功不代表目标连接成功。
- 车型或 ELF 不匹配：核对构建目录、CMake cache 和 manifest。
- 日志与遥测接入见 [Logger 指南](guides/logger-guide.md)和
  [云台监控](gimbal-monitor.md)。
