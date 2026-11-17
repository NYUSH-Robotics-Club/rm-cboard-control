# 电脑直连达妙 USB-CAN

## 先辨别后端

```text
Mac / Windows / Linux → USB-CAN 转接器 → 电机 CAN_H / CAN_L
独立电源 ─────────────────────────────→ 电机电源端
```

不经过 C 板，不需要达妙助手。程序通过匹配 SDK/串口协议把 CAN ID、DLC、载荷交给
转接器。电脑 USB 口不能直接接 CAN。已验证的 Mac 设备为 DM Tools USB2CANFD
REV V1.1 外观，但实际固件呈现 HDSC CDC 串口，并非原生 dm-device-sdk USB 接口。

| 当前检测证据 | 选择 |
| --- | --- |
| `dmcan` 的 DmCanContext 找到并打开设备 | 使用匹配平台/架构的 dm-device-sdk |
| VID:PID `2e88:4603`、HDSC CDC、系统串口可见 | 使用 PySerial + 官方 `u2can/DM_CAN.py` |
| 只有硬件外观/产品标签 | 还不足以决定 SDK；读设备标识与固件表现 |

原生 SDK 返回 0 台，不等于同一外观的 CDC 转接器不可用。也不要把 PySerial 的
921600（USB 串口配置）当成 CAN 的 1 Mbps（总线位速率）。SDK/串口适配器的配置
方式不同，不能向未知 USB 设备发送猜测的串口封包。

## 环境定位

本机已有 `/Users/pengyue/Codespace/damiao-3507`，包括 `.venv`、`check.command`、
`check_connection.py`、`vendor/DM_CAN.py`、平台动态库与 `requirements.txt`。
先读该目录当前 README/requirements；版本是已验证环境信息，不是所有电脑的固定要求。

```sh
cd /实际路径/damiao-3507
.venv/bin/python -m serial.tools.list_ports -v
.venv/bin/python check_connection.py
```

Windows 使用 `.venv\Scripts\python.exe`。端口按当前枚举选择：Mac 常见 `/dev/cu.*`，
Linux 常见 `/dev/ttyACM*`，Windows 为 COM 名称。不要复用历史端口和序列号。
检测脚本会打开设备/串口，但不发送电机运动命令；打开成功不是 CAN 应答证明。

在新电脑重建时，安装适合实际转接器的官方 SDK 与依赖；CDC 分支需要 pyserial、numpy。
原生 SDK 分支还需要对应 OS/CPU 架构后端。Mac 上可用 libusb；本机原生 SDK 的 dylib
链接与重建步骤见项目 README，不复制 Apple Silicon 动态库到 Windows/Intel 机器。
不能保证任意 USB-CAN 都适配达妙 SDK；其他厂商用其公开后端或 python-can 接口。

## CDC 分支的只读验证示例

确认只有一个进程占用串口后，在该项目虚拟环境里运行。先替换实际端口；ID=1、
反馈 ID=0 和 DM3507 仅适用于已确认的这台 J3507。例子不会使能或写电机参数。

```python
import math
import os
import time
import serial
from vendor.DM_CAN import Motor, MotorControl, DM_Motor_Type

class ObservedMotor(Motor):
    def __init__(self):
        super().__init__(DM_Motor_Type.DM3507, 1, 0)
        self.received = 0
    def recv_data(self, q, dq, tau, err):
        super().recv_data(q, dq, tau, err)
        self.received += 1

port = '/dev/cu.替换为当前实际端口'  # Windows 可使用 'COM3' 等实际值
options = {'exclusive': True} if os.name == 'posix' else {}
with serial.Serial(port, 921600, timeout=.1, write_timeout=1, **options) as connection:
    bus = MotorControl(connection)
    motor = ObservedMotor()
    bus.addMotor(motor)
    values = {}
    for rid in (8, 7, 10, 21, 22, 23, 35):
        motor.temp_param_dict.pop(rid, None)  # 清本地缓存，不写电机
        value = bus.read_motor_param(motor, rid)
        if value is None:
            raise RuntimeError(f'RID {rid} 没有确认的新回复')
        values[rid] = value
    assert values[8] == 1 and values[7] == 0, values
    limits = [values[r] for r in (21, 22, 23)]
    if not all(math.isfinite(v) and v > 0 for v in limits):
        raise RuntimeError('实际反馈范围无效')
    bus.change_limit_param(DM_Motor_Type.DM3507, *limits)  # 只改变主机解码范围
    before = motor.received
    bus.refresh_motor_status(motor)
    deadline = time.monotonic() + .5
    while motor.received == before and time.monotonic() < deadline:
        time.sleep(.01)
        bus.recv()
    if motor.received == before:
        raise RuntimeError('没有新电机反馈；缓存零值不算回复')
    print(values)
    print(motor.getPosition(), motor.getVelocity(), motor.getTorque(), motor.getError())
```

当前 SDK 的 `getPosition/getVelocity/getError` 都读缓存。构造 Motor 后默认零，或
以前收到过一帧后一直重复零，都不能证明当前在线。参数字典也应在新查询前清对应项。
没有新帧时应保留未确认结论，而非把默认状态 0 宣称成“已失能”。

## 运动与停止

已授权时可复用该目录的 `slow_rotate.py`：先读模式和实际范围，失能，临时切速度模式，
读回确认，零速，然后使能并持续 +0.1 rad/s。该脚本是 CDC 分支、固定 ID 的本地工具，
不是所有型号的通用控制器。它监测新反馈超过 1 s 则退出，退出尝试零速/失能。

原 USB 脚本“停止指令已发送”的输出没有单独确认失能回复；应继续做上面的新鲜度检查。
曾发生发送停止后无任何回复的情况，正确结论是未确认，而不是已通过反馈确认停止。
不要复制旧 PID。先核对当前运行命令，再 SIGINT，等待进程退出后才用第二个客户端查询。

现代 J3507 已验证的线协议（不是所有达妙固件的无条件规则）：

- 参数读取：标准 ID 0x7FF、DLC=8、`ID_L ID_H 33 RID 00 00 00 00`。
- 参数写入：标准 ID 0x7FF、`ID_L ID_H 55 RID VALUE_LE32`；切换速度模式为 RID 10、值 3。
- 使能/失能：基础电机 ID，七个 FF 加 FC/FD。旧固件可能要求不同命令 ID，先核对资料。
- 速度模式：ID `0x200 + MotorID`，float32 小端 rad/s，当前 SDK 以八字节发送，末四字节 0。
- 刷新状态：ID 0x7FF，`ID_L ID_H CC 00 00 00 00 00`；它是查询，不是使能。
- 参数保存与设零是独立写操作，不要作为连接检查的默认步骤。

官方实现和资料：
[dm-device-sdk](https://github.com/dmBots/dm-device-sdk)、
[USB2CANFD](https://github.com/dmBots/usb2canfd)、
[串口电机 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)。
