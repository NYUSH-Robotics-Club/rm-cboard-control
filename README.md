# C Board CAN Diagnosis

`Diagnosis` 分支将 RoboMaster C 板改为独立 CAN 排障工具：

```text
电脑终端 / 网页 → RTT TCP 流 → OpenOCD → ST-Link / SWD
                                         ↓
                              C 板 CAN1 / CAN2 ↔ 达妙电机
```

提供原始收发记录、单次测试报文、达妙只读 ID 探测/参数读取、内部静默回环、
CAN 错误计数和 JSON 导出。启动默认两路 1 Mbps 只听，不自动发送电机命令。
主机工具只用 Python 标准库；ST-Link 通过 SEGGER RTT 通道 0 双向收发文本命令和事件，不需要额外串口。
状态仅在命令请求时返回，主机不轮询整块诊断 RAM。

原整车控制、视觉、遥控器、旧监控工具和相应依赖已删除。冻结的 CubeMX/HAL
基线文件保留原样；其中旧 `Src/main.c` 等不参与此分支的固件构建。

先看 [接线、构建与使用说明](docs/diagnosis.md)。本项目不自动烧录或驱动电机。

```sh
# 需要 CMake >= 3.22、Ninja、Arm GNU 工具链和 Python 3.10+
cmake --preset Debug
cmake --build --preset Debug
sh tests/host/run_tests.sh

# 无硬件演示（不会连接 ST-Link）
python3 tools/diagnosis/interface.py --demo
# 浏览器打开 http://127.0.0.1:8765
```

产物为 `build/Debug/CBoard_Diagnosis.elf` 和 `.bin`。
实机接入与明确烧录命令见使用说明。
