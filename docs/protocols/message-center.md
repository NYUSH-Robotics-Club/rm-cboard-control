# Message Center（发布/订阅）

消息中心位于 `modules/message_center/`，是仿 ROS2 的定长、无动态内存事件总线。
它只负责排队、订阅和派发，不包含电机、底盘或视觉业务逻辑。

## 当前主题

主题定义在 `modules/message_center/message_center.h`：

| 主题 | 典型载荷 | 用途 |
|---|---|---|
| `TOPIC_RC_UPDATE` | 遥控器数据 | 输入更新 |
| `TOPIC_IMU_UPDATE` | `SensorData` | IMU 更新 |
| `TOPIC_CAN_RX` | 原始 CAN 数据 | 底层 CAN 事件 |
| `TOPIC_MOTOR_FEEDBACK` | `MotorFeedbackEvent` | DJI M3508 等反馈 |
| `TOPIC_GM6020_FEEDBACK` | `MotorFeedbackEvent` | GM6020 反馈 |
| `TOPIC_CHASSIS_CMD` | `ChassisCmd` | 底盘业务命令 |
| `TOPIC_SHOOT_CMD` | `ShootCmd` | 发射业务命令 |
| `TOPIC_GIMBAL_CMD` | `GimbalCmd` | 云台业务命令 |
| `TOPIC_VISION_DATA` | `Vision_Recv_s` | 旧 Seasky 传输数据 |
| `TOPIC_VISION_TARGET` | `VisionTargetMessage` | 传输无关的视觉目标 |
| `TOPIC_ROBOT_STATE` | `VisionRobotStateMessage` | 为上位机回传预留 |

业务消息结构优先定义在 `core/contracts/`。协议或驱动私有结构不能直接成为
新的长期业务契约。

## 初始化

消息中心使用调用者提供的静态环形队列：

```c
static MsgEvent queue[128];
MsgCenter_Init(queue, 128);
```

当前 `Src/main.c` 使用 128 个事件槽。环形队列会保留一个空槽区分满和
空，因此最大待处理事件数是 `length - 1`。

## 发布

```c
SensorData sensor_data;
/* fill sensor_data */

int status = MsgCenter_Publish(
    TOPIC_IMU_UPDATE,
    &sensor_data,
    sizeof(sensor_data));
```

返回值：

- `0`：成功；
- `-1`：未初始化或队列无效；
- `-2`：主题无效或载荷超过 128 字节。

队列已满时会覆盖最旧事件；`MsgCenter_GetDiagnostics()` 提供总覆盖数、
按主题覆盖数、合并数和批次上限命中数。STM32 构建中发布过程通过
`bsp/critical` 做短临界区保护，消息中心不再直接操作 MCU 中断寄存器。
当前中断路径可以发布；不要在中断中等待订阅者执行，因为回调
只会在后续派发时运行。

## 订阅

```c
static void on_chassis_cmd(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    if (event->size != sizeof(ChassisCmd)) {
        return;
    }

    ChassisCmd command;
    memcpy(&command, event->data, sizeof(command));
    /* Store or process the command without blocking. */
}

void Example_Init(void)
{
    (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD,
                              on_chassis_cmd,
                              NULL);
}
```

每个主题最多 8 个订阅者。订阅失败返回负值，初始化代码不应忽略容量错误。

## 派发

当前控制任务调用（上电初始化的等待路径也会派发）：

```c
MsgCenter_Dispatch();
```

一次派发最多处理 64 条普通事件，并在最后运行通用
after-dispatch hooks。当前最多注册 4 个 hook，电机服务用它刷新各厂商 adapter
缓存的发送帧。消息中心本身不知道 hook 属于哪个业务域。

回调同步运行在派发调用者上下文，不运行在发布中断中。回调不应长时间阻塞，否则会
延长控制周期并增加消息覆盖风险。

## 固定限制

| 项目 | 当前值 |
|---|---:|
| 单条载荷 | 128 字节 |
| 每主题订阅者 | 8 |
| after-dispatch hooks | 4 |
| 主程序队列槽数 | 128 |
| 满队列策略 | 覆盖最旧事件 |

`MsgEvent.data` 按 4 字节对齐，避免把 float/uint32 消息从队列取出时产生未对齐
访问；主机契约测试会检查全部公共载荷不超过 128 字节。

当前没有取消订阅、优先级、阻塞等待、背压、每主题队列或丢包统计。修改容量前
必须评估静态 RAM 和最坏派发时间；不要把业务副作用写进消息中心实现。

## 相关文档

- [全局架构与调试](../architecture/overview.md)
- [现有 USB CDC/Seasky 协议](seasky-vision.md)
