# 从零配置、构建与烧录

日常入口是 `just flash`。它使用**首次明确保存的车型**，先构建、检查，再识别探针/芯片、下载并校验。默认 SWD；**下载前以 NORMAL/SWrst 软件复位并暂停，校验后默认不复位运行**。只有用户执行 flash 时才进入复位/擦写阶段；doctor 和 flash-plan 不复位。

编译不需要 ST-Link；默认 SWD 烧录需要 ST-Link。当前源码保持裸机启动，**先不调用 RTOS**。构建 FreeRTOS 静态库不等于运行调度器。平台实测、工具路径与产物见 [验证记录](environment-validation.md)。

## 版本、来源与权限

下表适用于 Windows/macOS，不替换其他工程的依赖。CMake 4.2.3 / Ninja 1.13.1 是 Windows 已有且双车型实测通过的组合。Linux ARM64 使用后文单独列出的固定版本。编译脚本严格核对各主机版本；Python/编辑器不影响目标 ABI，已有可用版本可复用。

| 工具 | 固定版本/主机包 | 官方来源与安装 | 权限 | 验证命令 |
|---|---|---|---|---|
| Python | 本次 3.13.7；bootstrap 要求 3.12+ | [Python 3.13.7](https://www.python.org/downloads/release/python-3137/)：Windows x64 installer；Mac universal2 pkg | Windows 用户安装；Mac pkg 可能需管理员 | Windows `py -3 --version`；Mac `python3 --version` |
| Git | 本次 Windows 2.51.0；可复用现有 Git | [Git for Windows 2.51.0](https://github.com/git-for-windows/git/releases/tag/v2.51.0.windows.1)；Mac 可使用 Apple Command Line Tools 提供的 Git | Windows 可选用户安装；Apple 安装可能需管理员 | `git --version` |
| just | 1.46.0；Windows x86_64；Mac aarch64 / x86_64 | [维护者发行包](https://github.com/casey/just/releases/tag/1.46.0)，bootstrap 自动下载校验 | 用户级 | `just --version` |
| CMake | 4.2.3；Windows x86_64 / Mac universal | [Kitware 发行包](https://github.com/Kitware/CMake/releases/tag/v4.2.3)，bootstrap 解压安装 | 用户级 | `cmake --version` |
| Ninja | 1.13.1；ninja-win.zip / ninja-mac.zip | [Ninja 发行包](https://github.com/ninja-build/ninja/releases/tag/v1.13.1)，bootstrap 解压安装 | 用户级 | `ninja --version` |
| Arm GNU Toolchain | Windows x64、Apple Silicon：14.3.Rel1，GCC 14.3.1；Intel Mac：14.2.Rel1，GCC 14.2.1 | [Arm 14.3 官方清单](https://gitlab.arm.com/tooling/gnu-toolchains-for-arm/-/blob/releases/14.3.rel1/README.md)、[14.2 清单](https://gitlab.arm.com/tooling/gnu-toolchains-for-arm/-/blob/releases/14.2.rel1/README.md)，bootstrap 自动选择 `arm-none-eabi` 目标包 | 用户级解压 | `arm-none-eabi-gcc -dumpmachine`，应为 `arm-none-eabi` |
| STM32CubeProgrammer | 2.21.0；Windows x64 / Mac 对应架构 | [ST 产品页/历史版本](https://www.st.com/en/development-tools/stm32cubeprog.html)，选择 2.21.0，运行官方安装程序 | 可能需要 ST 登录/接受许可；安装到用户可写目录；驱动需管理员 | `STM32_Programmer_CLI --version` |
| ST-Link USB 驱动 | CubeProgrammer 附带 STSW-LINK009；本机 2.2.0.0 | [STSW-LINK009](https://www.st.com/en/development-tools/stsw-link009.html)；Windows 运行安装包内 `Drivers/stsw-link009_v3/dpinst_amd64.exe` | 管理员；只在缺失时安装 | `pnputil /enum-drivers`，检查 STMicroelectronics / stlink_dbg_winusb.inf |
| VS Code | 本次 1.129.1；Windows x64、Mac 按架构 | [官方历史版本 URL 说明](https://code.visualstudio.com/docs/supporting/faq#_previous-release-versions)；下载 `https://update.code.visualstudio.com/1.129.1/win32-x64-user/stable` 或相应 `darwin-arm64` / `darwin` 包 | 用户级；Mac 可拖到 `~/Applications` | `code --version` |

Windows 范围为原生 Windows 10/11 x64，实际验证是 Windows 11 26200。Arm 清单中的 `mingw-w64-x86_64` 是原生 Windows 发行包，**无需安装 MSYS2、Git Bash 或 WSL**。本机已有的 Windows Arm 编译器虽装在 Program Files (x86)，仍以实测可执行文件为准。

Mac 两个架构分别提供安装选择，目标系统取 ST 手册列出的 macOS 14 Sonoma / 15 Sequoia。[UM2237 系统要求](https://www.st.com/resource/en/user_manual/dm00403500-stm32cubeprogrammer-stmicroelectronics.pdf)明确区分系统版本和架构；Python 3.13 universal2 自身最低 10.13，不能据此推断整套工具支持旧 Mac。Arm 迁移后的版本清单未列完整最低 macOS 小版本，本次无法在 Mac 核验动态库/系统兼容性，**Mac 整套实现待实机验证**；旧于 14 或新于文档列出的系统应先核对该版安装包内要求，不盲装最新版或关闭 Gatekeeper。Intel Mac 使用 14.2.Rel1 是因为官方 14.3 清单不再提供 Intel Mac 包。

**Windows ARM64 暂时阻塞**：所选 Arm 主机包及 ST-Link USB 驱动未确认支持。脚本会停止；不能因 Python/just 有 ARM64 包就认为整套链路可用。

## Windows 首次操作（原生 PowerShell）

1. 先检查基础工具，缺少才安装：

   ```powershell
   Get-Command git,py,code -ErrorAction SilentlyContinue
   py -3 --version
   git --version
   code --version
   ```

   Python 使用上表官方 x64 安装器，选当前用户安装并保留 Python launcher。VS Code 使用 User Installer。Git 只需命令行功能；不用打开 Git Bash。这里不依赖尚未安装的 Python 或 just 来安装 Python。安装后新开 PowerShell；`py` 不存在时可先使用 Python 安装器显示的 `python.exe` 完整路径。

2. 获取代码，新目录完整克隆（默认 main）：

   ```powershell
   git clone --recurse-submodules https://github.com/NYUSH-Robotics-Club/rm_C_board_control_cleanup.git rm_C_board_control_cleanup
   Set-Location rm_C_board_control_cleanup
   py -3 tools/bootstrap.py
   ```

   bootstrap 先探测已有固定版本，缺失才从官方发行渠道下载到 `%LOCALAPPDATA%/Programs/FirmwareTools`。所有新下载归档校验官方 SHA-256 后才解压；没有校验和就停止。不会修改机器 PATH。若拿到的是上游尚未包含工具脚本的提交，请使用本次交付的完整工作目录，或先应用随交付保留的工具层改动。

3. 安装/复用 CubeProgrammer 2.21.0 和 ST-Link 驱动。已有工具无需重复安装。`STM32 ST-LINK Utility` 是另一款旧工具，不能替代这里的 CubeProgrammer CLI。ST 包内已附 JRE，无需额外安装 Java 或完整 STM32 IDE。驱动安装出现 UAC 时由用户完成管理员操作；即使暂时没有驱动/探针，仍可继续构建。

4. 明确保存车型、构建类型及烧录行为：

   ```powershell
   py -3 tools/firmware.py configure infantry_standard --mode Debug --run-after no --allow-single yes
   ```

   若安装在非标准目录，参数值整体加引号：

   ```powershell
   py -3 tools/firmware.py configure infantry_standard --tool "gcc=D:\开发 工具\Arm\bin\arm-none-eabi-gcc.exe" --tool "cube=D:\开发 工具\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
   ```

   有多个探针时追加 `--serial 实际序列号 --allow-single no`。`--run-after yes` 才允许校验成功后复位并运行；默认 no。配置保存到被 Git 忽略的 `.firmware.local.json`，不包含在共享提交中。直接再次 configure 会按输入更新烧录设置，未给序列号会清除旧序列号。

5. 从桌面启动 VS Code，打开项目文件夹，选择信任这个已确认的工作目录（如果出现工作区信任提示），**新建终端**。首次配置合并生成 `.vscode/settings.json`，设置本项目 PowerShell、终端 PATH 及任务工具路径；不更改用户其他项目的全局终端配置。

   ```powershell
   just doctor
   just build
   just build sentry_swerve
   just flash-plan
   ```

   当前外部 PowerShell 如暂时找不到 just，可直接使用配置里的完整路径：

   ```powershell
   $fwConfig = Get-Content .firmware.local.json -Raw | ConvertFrom-Json
   & $fwConfig.tools.just doctor
   ```

   若也希望外部终端全局可用，只向用户 PATH 添加该 just 所在目录一次，随后彻底退出并重开 VS Code/新终端。集成终端已由项目设置提供路径，不依赖桌面进程继承到的新 PATH。

## macOS 首次操作（Apple Silicon / Intel）

1. 查看系统和架构：`sw_vers`、`uname -m`。Apple Silicon 使用原生 arm64 终端和 Python；Intel 为 x86_64。先运行 `git --version`、`python3 --version`、`code --version`；缺 Git 时使用 Apple `xcode-select --install` 安装 Command Line Tools，缺 Python 时运行上表 Python universal2 官方 pkg。不要求 Homebrew，不需要安装完整 Xcode。VS Code 下载对应平台包，拖到 Applications 或 `~/Applications`，在其命令面板运行 “Shell Command: Install 'code' command in PATH”。

2. 获取仓库和安装固定工具：

   ```sh
   git clone --recurse-submodules https://github.com/NYUSH-Robotics-Club/rm_C_board_control_cleanup.git rm_C_board_control_cleanup
   cd rm_C_board_control_cleanup
   python3 tools/bootstrap.py
   ```

   bootstrap 安装到 `~/.local/opt/rm-firmware/`，不写 shell profile；Apple Silicon 自动选择 14.3.Rel1 darwin-arm64，Intel 自动选择 14.2.Rel1 darwin-x86_64。两者目标都为 `arm-none-eabi`，不能使用 `aarch64-none-elf` 或 Linux 压缩包。

3. 从 ST 官方历史下载页取得 **2.21.0** 的对应 Mac 架构包，按官方安装器完成安装。先核对包内系统要求；ST 可能要求登录或接受许可，由用户完成。Apple Silicon 优先原生 ARM 包；只有选择 x86_64 包时才按官方要求安装 Rosetta。无需 Windows 的 dpinst 驱动。保留 macOS 安全保护；禁止用关闭 Gatekeeper 或删除隔离属性绕过不兼容安装器。

4. 首次配置（非标准安装位置显式指定）：

   ```sh
   python3 tools/firmware.py configure infantry_standard --mode Debug --run-after no --allow-single yes
   # 如自动探测不到 CubeProgrammer，使用安装器实际生成的路径：
   python3 tools/firmware.py configure infantry_standard --tool 'cube=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI'
   ```

   打开 VS Code 项目并新建终端后，生成的本地设置将提供工具 PATH。从普通 shell 临时使用 just，可运行：

   ```sh
   fw_just="$(python3 -c 'import json; print(json.load(open(".firmware.local.json"))["tools"]["just"])')"
   "$fw_just" doctor
   "$fw_just" build
   "$fw_just" build sentry_swerve
   "$fw_just" flash-plan
   ```

   VS Code 新终端直接执行 `just doctor`、`just build`、`just flash-plan`。有多探针、需要复位运行时，使用与 Windows 相同的 `--serial`、`--allow-single`、`--run-after` 参数。以上 Mac 实现/安装步骤均**待 Mac 实机验证**。

## Linux ARM64 构建

2026-09-13 已实测本机 aarch64 的双车型 Debug 构建。固定工具为 CMake 4.4.3、
Ninja 1.13.2、Arm GNU Toolchain 15.3.Rel1（GCC 15.3.1，目标 `arm-none-eabi`）、
just 1.58.0；复用 `.firmware.local.json` 中已安装的工具路径。
Linux x86_64 尚未纳入此入口，Windows/macOS 版本要求保持不变。

已有本机配置时直接运行 `just build`。如果外部终端提示 `just: command not found`，
可用不依赖 just 的等价入口：

```sh
python3 tools/firmware.py build
python3 tools/firmware.py build sentry_swerve
```

也可从本机配置中读取 just 的完整路径执行；VS Code 已有本地工具 PATH 设置时，
新建终端即可使用 `just build`。首次配置使用
`python3 tools/firmware.py configure infantry_standard --mode Debug --run-after no`，
必要时通过 `--tool 名称=完整路径` 指定上述版本。
配置现在正确生成 `terminal.integrated.env.linux`。

Linux 产物写入 `build/verified/linux-aarch64-<路径标识>/<车型>-debug/`，保留原来
`build/linux-aarch64-<路径标识>/` 中用于实机诊断的旧镜像。
构建仍执行缓存、ARM ELF、内存、向量与未解析符号检查并生成 BIN/HEX/manifest。

`bootstrap.py` 尚无 Linux 自动安装流程，会明确停止，避免选择 macOS 包。
Linux 烧录现在使用已配置的 OpenOCD 0.12.0；Windows/macOS 默认仍使用 CubeProgrammer。
`flash.backend` 保存后端，Linux 缺省为 `openocd`，其他主机缺省为 `cubeprogrammer`。
需要显式配置时使用 `configure --backend openocd --tool openocd=完整可执行文件路径`。

```sh
just doctor      # 读取 Linux sysfs 中的探针信息，不打开 SWD
just flash-plan  # 显示当前车型、复位运行设置及完整脚本模板，不连接设备
just flash       # 构建、备份、下载、校验；按保存的 run_after 决定是否复位运行
```

OpenOCD 流程先选择唯一序列号，再在同一连接内核对设备 ID `0x413` 和 1024 KiB
容量，完整备份 1 MiB Flash，然后以 SYSRESETREQ 软件复位并暂停，写入本次构建的
ELF 副本并执行 `verify_image`。任意步骤失败都不会继续到复位运行；未请求运行时
校验后保持 halted。每次操作单独保存 `flash-*/firmware.elf`、`before-flash.bin`、
`program.tcl`、`openocd.log` 和 `flash-manifest.json`，成功时记录备份 SHA-256。
多探针或重复序列号不会自动选择第一支。旧 ST-Link/V2 的 12 字节 USB 序列号会
按 OpenOCD 的格式转为 24 位十六进制。

当前本机保存的是 `run_after=true`，执行 `just flash` 校验成功后会复位运行。
源码与现有板上固件有版本差异，详见 [实机记录](chassis-can1-check.md)；工具修复
已完成只读连接和模拟写入/失败验证，尚未实际替换板上固件。若 Linux 报 USB
`open failed`，检查设备权限及其他调试器是否占用；脚本不会自行 sudo、安装驱动
或放宽设备权限。调试环境沙箱外的只读连接已通过。

OpenOCD 参数依据：[适配器配置](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)、
[写入与校验](https://openocd.org/doc/html/Flash-Commands.html)。
下文 CubeProgrammer 下载命令适用于 `cubeprogrammer` 后端。

## 日常命令与硬件边界

| 命令 | 行为 |
|---|---|
| `just doctor` | 显示工具路径/版本；Linux OpenOCD 后端读 sysfs，CubeProgrammer 后端用 `-l stlink-only` 枚举探针；不连接芯片 |
| `just build` | 构建保存的车型、检查 ELF/map/cache、生成本次 bin/hex 和 manifest |
| `just build sentry_swerve` | 仅本次选择哨兵，不改变默认车型 |
| `just flash-plan` | 不连接设备，不下载/擦除/复位；显示下一次烧录的流程与命令模板，不把已有 ELF 当作新构建证据 |
| `just flash` | 配置检查 → 构建 → ELF 检查 → 探针选择 → 芯片识别 → 下载并校验 → 可选复位运行 |
| `just flash sentry_swerve` | 显式选择哨兵执行同一流程 |
| `just configure infantry_standard Debug no` | 保存默认车型/类型/不运行；单探针允许 |

**只有决定实际烧录时才执行 `just flash`。** SWDIO、SWCLK、GND、VTref/参考电压及可选 NRST 的接线、电源电压和供电方式依据实际 C 板与 ST-Link 手册确认，不把 VTref 当成通用供电输出。连接失败时不自动使用 under-reset、不解除读保护、不修改 Option Bytes，不自动升级探针固件。USB DFU 只有板卡支持并进入正确 Boot 模式时才可另行配置；当前统一脚本只实现 ST-Link/SWD。

无探针停止；多探针必须有明确序列号。写前显示车型、ELF 完整路径及 SHA-256、探针序列号、芯片 ID/名称/容量、复位行为。CubeProgrammer 使用 `mode=HOTPLUG` 识别，避免身份检查时复位；通过检查后，写入阶段使用 `mode=NORMAL reset=SWrst -halt -d ELF -v`，在下载前软件复位并暂停，清理原程序/先前失败留下的核心状态。该步骤与“校验后是否复位运行”是两个独立阶段；run_after=false 仅关闭后者。不会自动改用 UR 或解除保护。

下载命令使用 `-q -vb 3 -log`，去掉乱码进度条并将详细日志写到当前车型构建目录的
`cube-flash.log`。只有进程返回零且输出明确报告下载校验成功才算成功。
`failed to download Sector[0]`、99% 或任意下载失败均不能表示固件完整；Flash 可能
已被擦除或部分写入，脚本返回非零、不自动重试、不复位运行。先检查日志再重新完整烧录。
ST 对 NORMAL/HOTPLUG 的定义见 [官方命令说明](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html)；本机 2.21.0 `--help` 同样列出这两个模式和 SWrst。

STM32F405/407/415/417 共享设备 ID `0x413`；CLI 不能仅凭 ID 分辨封装和所有具体型号。脚本同时检查 ST 返回的 F40x 名称和 **1024 KBytes Flash**，也接受 CubeProgrammer 实际返回的等价写法 **1 MBytes**；换算后仍严格检查容量，不接受其他家族/容量或未知单位。首次接板还需人工按芯片丝印、板卡手册确认是项目指定的 STM32F407。识别字段缺失或无法解析时停止写入，并在错误中显示实际返回值。

VS Code 任务面板使用 **Firmware: Doctor / Build / Flash**。这些任务复用 just，不重复构建；Ctrl+Shift+B 默认只构建。历史 ST-Link/OpenOCD 烧录任务现改为 **flash: stlink (CubeProgrammer)**，同样调用 `just flash`，沿用车型、序列号、芯片检查和不复位设置。历史 DFU 任务保留，但不是默认 Build，也未纳入当前检查。不需要新增扩展；本机已有 C/C++ 和 CMake Tools，任务本身使用 VS Code 内置能力；无需 GDB 配置。

## ST-Link 无法识别：RM_Ecat 对照与分层排查

已核对 RM_Ecat 提交 `89a86d88d4ee9f44213c83e7d3b31dab7580c91c` 的
`EcatV2_AX58100_H750_Universal/stm32h750b-disco.cfg`：它加载 OpenOCD 的
`interface/stlink.cfg`、`target/stm32h7x.cfg`，并设置 `reset_config none`。
这是调试适配器/目标芯片配置，不是 Windows USB 驱动安装包。本工程是 F407，
不能照搬 H7 目标。适配文件保存在 [tools/openocd/stm32f407-stlink.cfg](../tools/openocd/stm32f407-stlink.cfg)，
使用 F4 目标并保留 `reset_config none`；保留源文件 GPL-2.0-or-later 标记。
该文件可供独立 OpenOCD 调用；Linux 的 `just flash` 生成带身份检查和备份的独立脚本，
不直接读取此 `.cfg`。Windows/macOS 默认使用 CubeProgrammer。
`reset_config none` 也不等于所有 OpenOCD 命令都不会软件复位。

先运行 `just doctor`，它显示 CubeProgrammer 原始枚举输出；枚举失败时，Windows
还会列出当前 USB 设备、服务和错误码，区分以下情况，全程不连接目标芯片：

| 标记 | 含义与下一步 |
|---|---|
| `NO_STLINK_USB` | Windows 未识别到 ST-Link USB。确认插的是探针的 USB 接口，换已知能传数据的线，直接接电脑 USB 口，再运行 doctor。控制板 USB 和探针 USB 不是一回事；亮灯也不能证明数据连接正常 |
| `USB_DEVICE_ERROR` | 存在身份不明的异常 USB 设备。根据设备管理器错误码检查线缆/接口/设备，不能直接断定它就是 ST-Link |
| `USB_DRIVER_ERROR` | 已识别 ST-Link USB，但 Windows 报错。查看设备管理器错误码；例如代码 28 时检查官方 STSW-LINK009 驱动安装情况 |
| `USB_PRESENT_CLI_UNAVAILABLE` | Windows 可见 ST-Link，但 CLI 未返回可用序列号。关闭占用探针的 IDE/调试会话，检查实际绑定驱动和原始 CLI 错误；不自动替换驱动 |
| `STLINK_LOADER` | 探针出现在 USB loader 模式；在 ST 官方工具中核查模式，脚本不自动升级固件 |
| `QUERY_FAILED` | Windows USB 诊断本身失败，不能据此声称没插探针 |

只有 doctor 显示探针序列号后，才进入板卡供电、VTref、GND、SWDIO、SWCLK
接线和芯片识别排查。MCU/SWD 配置错误不会让电脑 USB 设备列表里的探针消失。
本机 Driver Store 已有官方 2.2.0.0 驱动，不需要反复安装；其他电脑缺失时按上表
STSW-LINK009 官方入口安装。不要把设备列表里的摄像头 `Camera DFU Device` 当作 STM32 DFU。

## 产物、路径与恢复

构建目录为 `build/<平台-路径标识>/<车型缩写>-<debug|release>/`，Windows 缩写 w64、inf/sen。目录与 CMake cache 共同标识车型，不能靠都叫 `NYUSH_Infantry.elf` 来分辨。移动仓库/换工具路径会使用不同目录，旧目录保留。Windows 为带空格/中文的仓库或工具建立临时 `subst` 盘符（P:–Z: 中空闲项），退出时移除映射；不复制或删除原文件，也不需要管理员权限。盘符占用变化可能产生新构建目录。

每次成功构建输出 ELF、map、bin、hex、`.inspection.txt` 和 `firmware-manifest.json`。manifest 包含车型、类型、主机、工具版本、Git 提交/dirty 状态、ELF 完整路径、SHA-256、内存与 RTOS 符号检查。Flash/RAM 静态占用含已分配段和链接脚本 heap/stack 预留，不代表运行时最大栈或 heap 用量已测量。

| 报错 | 修复 |
|---|---|
| 找不到 just/py/python3 | 先完成基础安装；从桌面新开 VS Code 终端；外部终端用 `.firmware.local.json` 中完整路径 |
| 工具版本冲突 | `doctor` 看实际路径；`bootstrap.py` 复用固定版本或并排安装，再用 `configure --tool NAME=PATH` 固定。不要替换其他项目依赖 |
| 没有本机配置 | `just configure 车型 Debug no`；不允许 flash 偷选车型 |
| VS Code 打开为 Restricted Mode | 在 VS Code 中由用户确认信任这个仓库，再重开终端；不关闭全局工作区信任机制 |
| settings.json 含 JSONC 注释导致 configure 拒绝 | 先备份/人工合并设置为合法 JSON 后重试；脚本不会整体覆盖无法解析的旧配置 |
| 缓存属于另一个工程/编译器 | 保留旧目录，使用新 checkout/工具路径；不要通过修改目标 ABI、芯片或链接脚本规避 |
| `.firmware.lock` 已存在 | 检查是否还有构建/烧录进行；只有确认异常进程已停止后才手动删除遗留锁 |
| Windows 映射盘符耗尽 | 释放 P:–Z: 中一个非任务使用的盘符后重试；不要解除别人的映射 |
| no ST-Link / serial 不匹配 | 检查 USB 数据线、官方驱动、已保存序列号；不要选别的板尝试 |
| 芯片/容量不匹配，或读保护导致失败 | 核实板卡与程序是否对应；停止操作，不自动解锁或修改 Option Bytes |
| 下载/校验失败 | 返回非零且不复位运行；查电源、SWD 接线和工具日志，修复后重新完整执行 |
| 下载归档 SHA256/网络错误 | 重新访问官方源；不执行校验失败文件。ST 登录/安装交互由用户完成 |

bootstrap 通过 HTTPS 读取官方发布的 SHA-256 / GitHub 资产 digest，校验下载字节；Arm `.sha256asc` 的摘要会比对，但脚本不额外建立 GPG 信任链。既有工具本次复用且未重新下载安装包，不能追溯校验原归档；本机 CubeProgrammer 的 Authenticode 签名为 Valid。ST 安装包如果没有独立官方校验和，应检查系统签名与 ST 来源并记录该边界。

工具层失败测试：Windows `py -3 tools/test_firmware.py -v`，Mac `python3 tools/test_firmware.py -v`。这些测试模拟探针/芯片及 CLI 返回，绝不写设备，也不能代替 ARM 链接、实际烧录或实车验证。
