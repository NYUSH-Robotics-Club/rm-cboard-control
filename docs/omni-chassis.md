# 步兵四全向轮底盘

`config/robots/infantry_standard.c` 选择 `CHASSIS_TYPE_OMNI`，使用
`adapters/chassis/omni_chassis_strategy.c` 的参数化逆运动学。
当前配置为 X 形 ±45°、前后及左右轮中心距 0.54 m、轮半径 0.0775 m、
M3508 + P19（减速比 3591/187）。以下均为**源码配置**，并非当前板上
镜像或实车方向的证明。

## 轮位和 CAN 映射

俯视、车头朝上；运动学数组按左前、右前、右后、左后排序：

| 轮位 | CAN1 电调 ID | 反馈 ID | 0x200 控制帧字节 |
|---|---:|---|---|
| 左前 | 2 | 0x202 | 2、3 |
| 右前 | 1 | 0x201 | 0、1 |
| 右后 | 4 | 0x204 | 6、7 |
| 左后 | 3 | 0x203 | 4、5 |

CAN 报文字节仍按硬件 ID 的槽位排列，不按运动学数组排序。
`MotorConfig.direction` 中 ID1/2/3/4 当前为 -1/+1/+1/-1；
这些符号需要架空核对机械正向。

## 运动学与上限

车体速度约定为 `vx` 向前、`vy` 向左，单位 m/s；
`wz` 俯视逆时针为正，单位 rad/s。各轮速度由轮中心坐标
`(x_i,y_i)`、驱动单位向量 `(tx_i,ty_i)`、半径 `r_i` 和减速比
`G_i` 投影得出：

```text
v_i = tx_i*vx + ty_i*vy + (x_i*ty_i - y_i*tx_i)*wz
motor_rpm_i = v_i / r_i * 60/(2*pi) * G_i
```

控制输入先限制为归一化平移及 [-1,1] 的旋转量，再乘配置的
`max_translation_mps=0.90`、`max_rotation_radps=1.20`。
四轮目标超过公共 RPM 上限时整体缩放。速度 PID 输出 C620
电流命令原始刻度，不是 RPM 命令。C620 报文定义见
[厂商手册](official-docs/Robomaster_C620_Docs.pdf)。

## 停机与验证

几何、映射或反馈无效时，底盘不应输出非零电流；任一底盘轮
反馈超过 100 ms 未更新时四轮停机。零电流不代表车辆立即停止。
先运行 `tests/host/test_chassis_strategies.c` 和
`tests/host/test_omni_controller.c`，再以匹配固件架空逐轮检查
ID、方向、单轴平移和旋转。实车测试结果必须单独记录。
