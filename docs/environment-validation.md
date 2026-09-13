# 环境验证记录

## 2026-09-13：Linux OpenOCD 烧录入口修复

此前 `just flash` 被 Linux 未实现保护直接拒绝。现已接入本机 OpenOCD 0.12.0，
支持 Linux sysfs 探针选择、只读计划、F407 身份/容量检查、完整 Flash 备份、
构建产物副本、写入/校验及保存的运行偏好。

- 24 项原工具测试及 9 项新增测试通过，新增测试在 Tcl 模拟目标上实际执行生成的
  脚本，覆盖备份、写入、校验失败、保持暂停及校验成功后运行顺序和特殊字符路径。
- `just build`、`just doctor`、`just flash-plan` 和 Python 语法检查通过。
  本机后端为 openocd，run_after=true，计划对应当前步兵源码。
- 新脚本经 SWD 实机只读检查通过，约 3.33 V，ID 0x413，Flash 1024 KiB。
  未执行脚本的写入分支，不能把模拟验证或身份读取称为烧录成功。
- 当前源码与板上版本不同，待明确替换意图后才能完成实际写入/校验；备份和日志
  将保存在本次车型构建目录下的独立 `flash-*` 子目录。源码与冻结底层未改动。

## 2026-09-13：Linux ARM64 的 just build 修复

复现原入口报错 `This workflow supports Windows x64 and macOS`；主机检查在编译前
拒绝 Linux。本机已有工具为 CMake 4.4.3、Ninja 1.13.2、GCC 15.3.1、just 1.58.0，
与原 Windows/macOS 固定版本不同。已新增独立 Linux ARM64 版本配置，保持版本校验，
并修正 CMake 编译器缓存文件的版本目录和 configure 的 Linux 终端 PATH。

- 24 项 Python 工具测试通过，包括主机/版本、缓存编译器不匹配、旧产物隔离和
  Linux 不支持的下载/烧录入口拒绝检查。
- `just build` 完整及重复构建、`just build sentry_swerve` 完整构建通过。
  步兵 FLASH/RAM 为 127040/47800 B，哨兵为 129408/47808 B，CCMRAM 均为 0。
- 双车型 ARM EABI5 hard-float、向量与地址范围、无未解析符号检查通过；
  ELF/map/BIN/HEX/inspection/manifest 生成在 `build/verified/linux-aarch64-306202d6/`。
  RTOS 启动/调度器符号均未链接。
- 原 `build/linux-aarch64-306202d6/inf-debug/NYUSH_Infantry.bin` 的 SHA-256
  仍为 `8e5a1434bcc1a51cb120ca5193b4ca42cf857abf28a0b35d2037c958817a75e6`，未覆盖。
- 此构建修复没有安装工具、烧录或访问硬件；当时 Linux 自动安装及统一 flash 尚未实现，
  后续 OpenOCD 接入见上节。
  板上运行状态与遥控检查见 [CAN1 检查](chassis-can1-check.md)，下方为历史记录。

## 2026-09-07：Windows 环境与下载诊断

当时最新状态：用户后续实际下载报 Sector[0] 失败；只读检查确认 Flash 已部分写入，
CPU 处于 locked up 状态。脚本已将下载阶段由 HOTPLUG 改为 NORMAL/SWrst（复位后暂停），
身份检查仍为 HOTPLUG。18 项模拟测试/flash-plan 通过；修订后的真实重烧仍待验证，
不能宣称已修复硬件下载或把 99% 当成功。详情见下方失败诊断。

此前状态：用户确认使用 DJI 官方 C 板。当前 ST-Link 已被 Windows/CubeProgrammer
识别；HOTPLUG 读取到 3.21 V、设备 ID 0x413、F405/407/415/417 系列、`1 MBytes`。
已修复脚本只接受 `1024 KBytes` 的容量单位误判，真实输出回放校验与 18 项工具
测试通过。下面无探针记录是早先检查状态；仍未执行下载、擦除、复位或运行。

源码来自 `NYUSH-Robotics-Club/rm_C_board_control_cleanup`，main 提交
`6c2925f6e72461f17292be268c0c1b296b051838`。本次使用新目录完整 Git clone，
非浅克隆，包含两次提交；仓库没有额外子模块。工具层改动保留在工作区，未推送远端。

## 主机与依赖

- Windows 11 家庭版中文版 10.0.26200，x64；本次原生 PowerShell 7.6.5。
- Python 3.13.7；VS Code 1.129.1 x64；Git 2.51.0.windows.1。
- CMake 4.2.3；Ninja 1.13.1；Arm GNU Toolchain 14.3.Rel1，GCC 14.3.1，target `arm-none-eabi`。
- STM32CubeProgrammer 2.21.0 在标准 ST 安装目录，CLI 能运行，Authenticode 为 Valid。
- Driver Store 已有 ST-Link 调试/VCP/bridge 驱动 2.2.0.0；未重复安装、未移除旧版本。
- 缺少的 just 1.46.0 已用户级安装，官方 x86_64 ZIP 的 SHA-256 比对通过：
  `f0acf3f8ccbcf360b481baae9cae4c921774c89d5d932012481d3e0bda78ab39`。
- 其他所需工具和 C/C++、CMake Tools 扩展已有，直接复用。未安装 ROS、CUDA、完整嵌入式 IDE 或额外 GDB 环境。

发现本机 Python 3.10、3.13、WindowsApps alias 并存，用户 PATH 还有 MSYS/MinGW
相关工具；VS Code 全局默认终端原为 MSYS2。项目本地配置选择原生 PowerShell，
并为任务/新终端提供选定工具目录。构建子进程只优先使用配置中的 ARM 编译器，
不继承其他项目的 CC/CXX/CFLAGS/CXXFLAGS/ASMFLAGS/LDFLAGS。

具体可执行文件完整路径、CPU 信息和已有 PATH 记录于交付目录外层的
`environment-report.local.json`；用户名、路径和探针配置没有写进共享项目配置。

## 当前 ARM 产物（Debug）

| 车型 | FLASH / 1 MiB | RAM / 128 KiB | CCMRAM / 64 KiB | Reset entry |
|---|---:|---:|---:|---|
| infantry_standard | 127312 B (12.14%) | 47800 B (36.47%) | 0 B | 0x0800c6d9 |
| sentry_swerve | 129672 B (12.37%) | 47808 B (36.47%) | 0 B | 0x0800c6e9 |

两车型均完成 113 个 ARM 编译/归档/链接步骤。ELF 都是 little-endian ELF32 ARM
EABI5、Cortex-M4/v7E-M、Thumb、VFPv4-D16 hard-float；向量表位于 0x08000000，
初始 MSP 为 0x20020000。所有分配段和加载段位于冻结链接脚本的合法内存区域；
无未解析符号，SVC/PendSV/SysTick 向量绑定当前强符号处理器。

每个目录均生成 ELF、map、bin、hex、inspection 和 manifest；根据 CMake 缓存确认
车型及 Debug 构建。最新完整路径、SHA-256、Git dirty 和工具链 banner 以各自
`build/w64-*/inf-debug/firmware-manifest.json`、
`build/w64-*/sen-debug/firmware-manifest.json` 为准。
历史目录保留，不将旧产物作为构建失败时的回退。Debug 路径/工具映射会影响 ELF
的调试信息和 SHA，容量相同并不表示 ELF 每个字节相同。

## RTOS 核查与明确边界

当前 main 在初始化后进入裸机循环，没有调用 `RobotRtos_Start()`。
`Src/stm32f4xx_it.c` 的 SVC/PendSV 函数为空，SysTick 只调用 `HAL_IncTick()`。
两车型 ELF 中 `RobotRtos_Start`、`vTaskStartScheduler`、`xPortPendSVHandler`、
`vPortSVCHandler`、`xPortSysTickHandler` 均未链接。

运行时代码和 FreeRTOS 静态库存在，但没有进入有效启动链。原 PROJECT_MEMO、
README 和 setup-guide 的部分“已启用 RTOS”表述与当前源码不符，已在本次文档
中纠正。用户明确要求先不调用 RTOS，本次没有添加任何调用或中断转接。

## 工具层验证

- `just doctor`：编译环境就绪；实际 `-l stlink-only` 枚举结果为无 ST-Link。
  用户回复已连接后再次检查，Windows 仍未出现 ST-Link/VID_0483；只读诊断返回
  `NO_STLINK_USB`，13 个当前 USB 设备错误码均为 0。摄像头的 DFU 接口不是 STM32。
  本机已有 ST 官方驱动，尚不能证明线缆/探针硬件正常，未重复安装驱动。
- 参考 RM_Ecat 89a86d88 新增可选 F407 OpenOCD 配置，未安装或实测 OpenOCD。
  日常入口继续使用 CubeProgrammer。修订后标准步兵再次构建、ELF 检查通过，
  SHA-256 为 `7549f026c60f7736b2cd4ea15ac04120f75d1a9d6a1c10381fc6835eecdccb11`，
  RTOS 仍未链接；任务 JSON 可解析，冻结底层 diff 为空。
- `just flash-plan`：生成只读计划；未调用设备连接、下载、擦除、复位或运行命令。
- 17 项 Python 测试：无配置不选车型、显式车型覆盖不改默认、无探针/多探针/
  指定探针缺失、芯片或容量错误、编译失败、外部进程非零、下载失败、校验失败、
  仅明确选择后复位、计划不启动外部进程、错误 ELF、含空格/中文参数均通过。
  涉及硬件的返回值均为测试替身，不能标为真实烧录通过。
  ST-Link 修订新增 USB 状态分层判断、CLI 枚举失败不进入连接的测试。
- 将仓库复制到包含“中文 空格工程”的路径，并将 just 放在“工具 空格”路径，
  在仓库以外目录通过 `just --justfile ... build infantry_standard` 完整构建通过。
- 初始 Windows 过长构建目录暴露 MinGW 路径限制，已用较短且可区分的构建目录
  修复；中文路径的 DSP 库链接失败已用临时 subst 映射修复，未改链接脚本或库。
- 首次配置读写成功；本机路径/复位选项保存后可重读；bootstrap 重跑复用全部固定版本。
- 新增 Firmware: Doctor/Build/Flash。默认 Build 改为只构建，旧 DFU 任务不再占用
  默认构建快捷键。ST-Link 修订将旧 OpenOCD 烧录任务替换为统一 CubeProgrammer
  入口，标签为 flash: stlink (CubeProgrammer)，其余历史任务保留；没有自动烧录任务。
- VS Code 真实终端/任务验证状态将在完成工作区信任后补记；此前测试窗口因
  Restricted Mode 忽略终端配置，不能把该次尝试声明为通过。
- 冻结范围以及 BSP、应用、电机驱动、协议、控制参数没有修改；顶层 CMake、
  原 toolchain、CMakePresets 保留。

## 仍未验证

已执行一次 HOTPLUG SWD 身份读取，没有执行烧录、擦除、复位、运行，没有机器人
功能或实车安全验证。探针已枚举，目标参考电压读数为 3.21 V；用户确认 DJI 官方 C 板，
尚未独立核查芯片丝印/完整接线。早先断开探针与当前探针序列号不同，不能据此宣称
早先红灯闪烁的具体根因已查明。
Release 分支提供配置入口，本次容量记录为 Debug。
主机 Unix shell 测试未作为 Windows 依赖运行，不代替 ARM 检查。

macOS Apple Silicon / Intel 仅提供安装和统一脚本实现，均待对应系统实机验证。
Intel Mac 固定 Arm 14.2.Rel1；Windows ARM64 因 Arm host/USB 驱动支持未确认而阻塞。

## Sector[0] 下载失败诊断

用户提供日志显示擦除扇区 [0,4] 后下载失败，属于实际写入尝试，不能用早先环境配置
“未烧录”的记录描述该次操作。agent 本轮仅执行 HOTPLUG 读取，未重复擦写或复位。

- RDP=0xAA（无读保护），WRP0..11 全部不保护；目标电压 3.21 V。
- FLASH_SR=0，CPU 状态 locked up；PC=0x20000000，MSP=0x20000500，
  RAM 首条指令包含 BKPT。CFSR=0x00018200，HFSR=0x40000000，表明存在故障状态；
  尚不能仅凭这些寄存器证明最初故障触发原因。
- 读取 0x08000000 起 127312 B，与本次 bin 比较：121586 字节不同，首个差异
  0x08001390，末个差异 0x0801F14F，确认镜像不完整。
- 读回 SHA-256：`f4da4c40e4d663cf376f6c8d6702373b0bc73f97915cc8e487f16db0b5250170`；
  期望 bin：`9b3b9cc98973e3ccdd6f0f9cd9cb587f97715b182dd192227c768d148fcd21cc`。
- 本机证据保存在被忽略的 build/flash-failure-readonly.log、flash-failure-registers.log、
  flash-failure-readback.bin；修订后的计划在 build/flash-recovery-plan.log。
- 18 项工具测试验证身份检查不复位、写入前 NORMAL/SWrst、失败不自动重试/运行，
  以及原有芯片/容量/车型/校验检查。真实重烧尚未执行，RTOS 和冻结源码未修改。
