# 开发环境、构建与烧录

**本项目首选 OpenOCD + ST-Link/SWD。CubeProgrammer 不推荐用于日常开发，不是安装依赖，
统一命令也不会调用它。** pyOCD 是可用的替代工具，但本仓库不维护第二套 pyOCD 烧录入口。

这是针对 Linux ARM64、ST-Link 和 STM32F407 的选择：OpenOCD 原生支持目标芯片，
同一套配置可用于烧录、校验和 GDB 调试。pyOCD 同样支持 ST-Link，适合需要 Python API
或 CMSIS-Pack 的工作流；Linux 并不要求只能用其中一个。另一个组织仓库的工具选择
不是本项目的强制约束。

参考：[OpenOCD ST-Link 支持](https://openocd.org/doc-release/html/Debug-Adapter-Configuration.html)、
[OpenOCD Flash 命令](https://openocd.org/doc-release/html/Flash-Commands.html)、
[pyOCD 探针支持](https://pyocd.io/docs/debug_probes.html)。

当前源码保持裸机启动，先不调用 RTOS。冻结底层不因配置环境而改动。

yaw/pitch实时电流、位置、速度及控制回调监控见[云台监控说明](gimbal-monitor.md)，
烧录含监控的新固件后运行`just monitor`。

CAN Bus-Off 的受控恢复及上电回中解锁见 [CAN 故障恢复](can-recovery.md)。
新固件上电或恢复后需要两只拨杆下位、摇杆回中，恢复通信不会直接恢复旧电机指令。

## 版本策略：安装最新，记录实际版本

按用户要求，bootstrap 查询发布渠道的最新正式发行，不再要求旧文档中的固定版本。
日常 doctor/build/flash 不联网升级。工具并排安装在用户目录，旧版本、旧构建目录保留。
本地配置与构建 manifest 记录实际工具版本，方便复现和排错。

| 工具 | 本轮安装/选择 | 来源 |
|---|---|---|
| Python | 3.14.7，项目独立 .venv | uv 管理的 CPython / python-build-standalone |
| just | 1.58.0 | [维护者 Releases](https://github.com/casey/just/releases) |
| CMake | 4.4.3 | [Kitware Releases](https://github.com/Kitware/CMake/releases) |
| Ninja | 1.13.2 | [Ninja Releases](https://github.com/ninja-build/ninja/releases) |
| Arm GNU Toolchain | 15.3.Rel1，GCC 15.3.1，目标 arm-none-eabi | [Arm GitLab](https://gitlab.arm.com/tooling/gnu-toolchains-for-arm) |
| OpenOCD | xPack 0.12.0-7，原生 Linux ARM64 包 | [xPack Releases](https://github.com/xpack-dev-tools/openocd-xpack/releases) |
| Git | 复用系统已有 Git | 本机 2.34.1 |

上表是本轮快照，不是版本上限。xPack 是 OpenOCD 的独立二进制发行项目；其最新发布包
包含上游开发分支代码，本轮 banner 为
0.12.0+dev-02228-ge5888bda3-dirty (2025-10-04-22:42)。
这里的 dirty 是发行包自带 banner，不表示本轮修改了 OpenOCD 源码。不能把这个包
表述成未经修改的上游稳定版 0.12.0。bootstrap 记录包版本、URL、SHA-256。

bootstrap 校验 GitHub 资产 digest 或发行方 SHA-256 文件后才解包；Arm 使用官方
.sha256asc 摘要，但没有额外建立 GPG 信任链。查询失败、缺少目标主机包或校验失败
时停止，不猜下载地址或自动降级。OpenOCD 必须与同一发行包的 scripts 一起使用。
固件工具只保留必要兼容性下限和 MCU/ABI 检查，不锁定上述版本。

## Linux ARM64 / Jetson 首次配置

Ubuntu 22.04 系统 Python 保留；使用用户目录 Python 与项目虚拟环境。
bootstrap 支持 Linux aarch64/x86_64；本轮实机为 Ubuntu 22.04.5 aarch64。

~~~sh
python3 -m pip install --user --upgrade uv
"$HOME/.local/bin/uv" python install
# 将下方解释器路径替换为上一条安装结果中的最新稳定 CPython 路径
"$HOME/.local/bin/uv" venv --python /最新稳定CPython的完整路径/bin/python3 .venv
"$HOME/.local/bin/uv" pip compile --upgrade --python .venv/bin/python requirements.in --output-file requirements.txt
"$HOME/.local/bin/uv" pip install --python .venv/bin/python -r requirements.txt
.venv/bin/python tools/bootstrap.py
.venv/bin/python tools/firmware.py configure infantry_standard --mode Debug --run-after yes --allow-single yes
source tools/activate.sh
just doctor
just build
just build sentry_swerve
just flash-plan
~~~

不固定 Python minor；按 [uv 官方安装说明](https://docs.astral.sh/uv/guides/install-python/)
先更新 uv，再安装最新稳定 Python，并显式选择刚安装的解释器，避免误用旧环境。
已有 .venv 时先保留备份，
再切换解释器，不让环境管理器无提示删除原环境。本轮旧环境保留为
.venv-python313-backup/。requirements.in 只声明直接依赖，requirements.txt 是解析出的
可复现版本快照；以后需要更新 Python 包时重复 compile --upgrade 和 pip install。

source tools/activate.sh 只改变当前 Bash 终端的 PATH 和虚拟环境，不写全局 shell profile。
工具安装在 ~/.local/opt/rm-firmware/。没有桌面的机器无需安装 VS Code GUI 才能构建。
本机已有 ST-Link USB 权限规则，本轮不重复添加规则或修改用户组。

“最新”适用于新装的开发工具和主机 Python 依赖，不表示自动升级 Ubuntu、系统 Python、
Jetson/ROS、探针固件或仓库冻结的 HAL/FreeRTOS。Git 当前复用系统版本，并未升级为
上游最新；新机器安装 Git/编辑器/驱动时选各自官方渠道最新兼容发行。

## 现在能烧录、能看反馈吗？

2026-09-09 本机已用 OpenOCD 建立 SWD 连接，读到 Cortex-M4、ID 0x413、
1024 KiB Flash、约 3.23 V。两车型 Debug 构建通过；随后经用户授权执行
`just flash`，步兵 Debug 真实擦写与 verify_image 校验通过，退出码 0。
校验后保持暂停，尚未验证程序运行、USB 日志或机器人功能；哨兵尚未实机烧录。

确认车型及电机安全状态后，在仓库终端运行 `source tools/activate.sh`、`just flash`。
该命令会真正擦写；当前默认步兵 Debug，校验后保持暂停，**不会自动运行，也不会输出日志**。
需要运行时须另行明确选择运行操作，不能复位运行失败或部分写入的镜像。

本机当前 `/dev/ttyACM0` 属于 ST-Link 自带 UART，并非 C 板固件日志 USB CDC；
尚未枚举到 C 板自身 USB。日志需另接 C 板 USB 数据口，并运行有效固件。
消息回调并非自动对电脑开放的接口；读取现有日志或用 GDB 查看回调，见
[日志与回调调试](guides/logger-guide.md)。

## Windows / macOS

两平台沿用同一 bootstrap、configure 和 just 入口，新增 OpenOCD 路径后需要重新 configure。
本轮 OpenOCD 流程未在 Windows/macOS 实机验证，不沿用历史 CubeProgrammer 烧录验证结论。

Windows 使用原生 PowerShell、当前稳定 Python 和 Git：

~~~powershell
py -3 tools/bootstrap.py
py -3 tools/firmware.py configure infantry_standard --mode Debug --run-after yes --allow-single yes
~~~

bootstrap 使用 Windows x64 发行包，安装在 %LOCALAPPDATA%/Programs/FirmwareTools。
无需为这些命令安装 MSYS2。Windows ARM64 主机包支持未确认，脚本拒绝运行。
ST-Link USB 驱动若缺失，使用官方 [STSW-LINK009](https://www.st.com/en/development-tools/stsw-link009.html)
单独驱动包；无需安装 CubeProgrammer。驱动安装如需管理员，由系统提示处理。

macOS 使用原生架构 Python/Git：

~~~sh
python3 tools/bootstrap.py
python3 tools/firmware.py configure infantry_standard --mode Debug --run-after yes --allow-single yes
~~~

bootstrap 按主机架构选择包；如果最新 Arm 发行不提供 Intel Mac 包，会明确失败，
不再静默锁定历史 14.2。不要关闭 Gatekeeper 或靠架构模拟掩盖缺少的原生支持。

所有平台都可以显式指定已有工具：

~~~sh
python3 tools/firmware.py configure infantry_standard --tool 'openocd=/实际安装目录/bin/openocd'
~~~

Windows 可使用 --tool "openocd=D:\开发 工具\OpenOCD\bin\openocd.exe"。
工具目录包含空格时保留引号，OpenOCD 的配套 scripts 目录应保持发行包原布局。

## 本地配置与日常命令

.firmware.local.json 保存工具路径、车型、Debug/Release、OpenOCD 后端、SWD 和探针选择。
configure 会把历史 Cube 后端迁移为 OpenOCD，并从活动 tools 中移除旧 cube 路径；
不会卸载用户机器上的其他工具。

多探针请保存 --serial 实际序列号 --allow-single no。默认 --run-after yes，
烧录校验成功后自动 reset run。调试时可显式设置 --run-after no 保持暂停。
再次 configure 不给 serial 会清除旧序列号；已有本地配置沿用其保存的 run_after 值。

| 命令 | 行为 |
|---|---|
| just doctor | 版本与 OpenOCD 配置解析检查，读取 OS USB 序列号；不连接 MCU |
| just build | 构建保存的车型，检查 ELF/cache，生成 bin/hex/map/manifest |
| just build sentry_swerve | 仅本次构建哨兵，不修改默认车型 |
| just flash-plan | 显示 OpenOCD 计划和命令模板，不启动目标连接、复位或擦写 |
| just flash | 构建与检查后，使用 OpenOCD 执行芯片检查、复位暂停、写入与校验，默认随后复位运行 |
| just configure infantry_standard | 保存步兵/Debug/OpenOCD/单探针允许/校验后自动复位运行 |
| just configure infantry_standard Debug no | 调试用：校验成功后保持暂停 |

VS Code 的 Firmware: Doctor / Build / Flash 和 flash: openocd (stlink) 均复用 just。
Ctrl+Shift+B 默认只构建。项目本地 settings 合并生成终端 PATH 和编译器路径；
打开新终端生效。JSONC 无法解析时停止，不覆盖旧设置。旧 DFU 任务仅是手动备用入口，
不属于当前默认流程，未验证且不得把任意 USB DFU 设备当作 C 板。

## OpenOCD 烧录的实际行为

tools/openocd/stm32f407-stlink.cfg 加载发行包中的 ST-Link 和 STM32F4 配置：
新版使用原生 ST-Link/SWD，旧 HLA 发行兼容选择 hla_swd。reset_config none
表示不假设接有 NRST；并不禁止软件复位。SWD 速度采用该发行包的 F4 配置。

doctor 用 noinit 解析配置并退出，同时关闭 GDB/Tcl/Telnet 服务端口。
探针枚举只读取 Linux sysfs、Windows PnP 或 macOS system_profiler 的 USB 元数据。
缺失、重复或无法可靠解析的物理序列号会拒绝选择；USB 可见不等于 SWD 连线已验证。

实际 flash 在构建与 ELF SHA-256 检查通过后，以唯一选定序列号打开一个 OpenOCD 会话：

1. init，然后读 DBGMCU_IDCODE 和 Flash 容量，严格检查设备 ID 0x413、1024 KiB。
2. 匹配后执行 reset halt；再次检查身份和容量，随后才允许擦写。
3. flash write_image erase 写入本次 ELF，再运行 verify_image。
4. 校验成功后默认执行 reset run；显式 run_after=false 时保持暂停。任何前置步骤失败均不执行复位运行。

init 会由 OpenOCD 目标脚本配置调试相关寄存器，不能称为完全只读的连接。
这里使用 reset halt，不调用会运行额外时钟配置钩子的 reset init。目标脚本可能执行
正常调试初始化；它不修改仓库冻结底层文件。身份 ID 是 F405/407 家族共享值，仍需
按板卡手册/芯片丝印确认 F407；脚本不能仅凭 ID 分辨封装。

写入、校验或连接失败均停止，不自动重试、不使用 under-reset、不解除读写保护，
不修改 Option Bytes、不升级探针固件。失败时 Flash 可能已擦除或部分写入，
不能运行未完整校验的镜像。日志保存到当前构建目录的 openocd-flash.log。
退出码零且同时有 FIRMWARE_VERIFY_OK / FIRMWARE_FLASH_OK 才报告成功。

CubeProgrammer 已从推荐流程、工具发现、默认任务和烧录实现中移除。
历史 Windows 的 CubeProgrammer 记录仅用于追溯，不是继续安装它的要求。

## 验证与排查

~~~sh
python tools/test_firmware.py -v
python tools/test_bootstrap.py -v
python tools/test_openocd.py -v
CC=gcc sh tests/host/run_tests.sh
python script/logger.py --help
~~~

OpenOCD 测试使用真实 Tcl 解释器，但目标操作全部由测试替身替代，
且不加载接口配置；这些测试不能代替真实芯片连接或烧录验证。
平台和内存结果见 [环境验证记录](environment-validation.md)。

| 问题 | 处理 |
|---|---|
| 找不到 just | source tools/activate.sh，或用本地配置中的完整路径；VS Code 新建终端 |
| 缺少 OpenOCD/interface/target scripts | 重跑 bootstrap 或配置完整发行包中的 openocd 路径 |
| 下载/校验失败 | 核对发布渠道，不执行未校验的归档，不自动使用旧版兜底 |
| 没有 ST-Link 序列号 | 检查探针 USB 数据线、lsusb/设备管理器和 USB 权限 |
| 多探针或序列号重复 | 保存实际 serial；重复序列号需断开有歧义的探针 |
| USB 可见但连接失败 | 检查占用会话、目标电源/VTref/GND/SWDIO/SWCLK；不要猜硬件参数 |
| 芯片 ID/容量不匹配 | 停止并核实板卡，不能解除保护强行尝试 |
| .firmware.lock 已存在 | 先确认旧进程已退出，再人工处理遗留锁 |
| 编译缓存不匹配 | 保留旧目录，使用对应工具/路径的新构建目录 |

成功编译只证明链接和静态检查通过，不证明机器人功能、控制周期或电机安全已验证。
