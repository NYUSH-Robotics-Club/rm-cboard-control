# 双手柄 WebXR 接收扩展

在原 VRD2 平移接收器上增加 VRD3，原 VRD2 仍然逐字节兼容。只改 `modules/remote/remote_control.c` 的接收适配：允许 30 字节 VRD3 的 12/13 字节放置 little-endian i16 底盘旋转，范围 ±100 RC 刻度；14..29 字节必须为零。disabled/diagnostic 必须三轴为零。原 CRC、命令码、序号、DBUS 优先和超时重接管规则保留。

前后进入 RC ch3，左右进入 ch2，旋转进入 ch4（原拨轮）。ch0 始终零，不把旋转映射给云台 yaw。符号以底盘 +wz 逆时针为约定，实际车辆方向须匹配接线实测。路由器原有拨轮滞回/斜坡保留，松扳机发布零输入不等于物理瞬时制动。

主机入口：`python3 tools/vr/tests/test_c_receiver.py`（旧路径 34 项）和 `python3 tools/vr/tests/test_dual_receiver.py`（14 项），以及完整 `tests/host/run_tests.sh`。两车型需构建。诊断 `vr_dual_firmware_tag=0x56524433` 和 `vr_uart_last_rotation` 仅用于只读核对；入队尝试计数不是电机执行 ACK。

上位机新建在独立 `dual-controller-ui-20261120/studio`，无须更换或改动 FrankaBot。采用官方 CustomControl / 0x0311 下发，不经 Jetson 网络旁路写入指令。VRD3 不被旧固件接受；部署前须保留完整 Flash、原 ELF、校验结果和回滚入口。
