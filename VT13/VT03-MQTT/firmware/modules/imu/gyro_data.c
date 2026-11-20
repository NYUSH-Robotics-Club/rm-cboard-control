/*
 * 读取 IMU、计算姿态并发布统一传感器数据。
 * 角度和角速度的方向约定必须与控制器保持一致。
 */
#include "gyro_data.h"
#include "bmi088driver.h"
#include "wt61c.h"
#include "printing.h"
#include "logger.h"
#include "message_center.h"
#include "QuaternionEKF.h"
#include "buzzer.h"
#include "bsp_time.h"
#include <math.h>

static float gyro[3];
static float accel[3];
static float temp;

// 姿态估计变量（静态保持状态）- QuaternionEKF会管理这些
static uint8_t initialized = 0;     // 初始化标志

// 陀螺仪零偏（rad/s）- 开机静止校准获得
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static uint8_t calibrated = 0;      // 校准完成标志
static float accel_scale = 1.0f;    // 加速度计缩放系数
static float g_norm = 9.81f;        // 重力加速度范数

// 时间测量
static uint32_t last_update_tick = 0;  // 上次更新时间戳（ms）
static float dt = 0.005f;               // 动态计算的采样周期（s）

// 校准期间的回调函数（用于保持云台位置等）
static GyroCalibCallback_t gyro_calib_callback = NULL;

#define RAD_TO_DEG (57.295779513f)
#define DEG_TO_RAD (0.017453292f)
#define GRAVITY_ACCEL (9.80665f)  // 标准重力加速度 m/s²

/**
 * @brief 使用加速度计初始化四元数（仅roll和pitch）
 * @param ax, ay, az: 加速度计读数（单位：m/s²）
 */
static void init_attitude_from_accel(float ax, float ay, float az)
{
    // 注意：BMI088_read返回的accel单位是m/s²，需要转换为g用于角度计算
    ax = ax / GRAVITY_ACCEL;
    ay = ay / GRAVITY_ACCEL;
    az = az / GRAVITY_ACCEL;

    // 计算roll和pitch（假设静止状态，加速度计测量重力方向）
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;  // yaw无法从加速度计获取，初始化为0

    // 从欧拉角转换到四元数
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    float init_quaternion[4];
    init_quaternion[0] = cr * cp * cy + sr * sp * sy;  // q0 (w)
    init_quaternion[1] = sr * cp * cy - cr * sp * sy;  // q1 (x)
    init_quaternion[2] = cr * sp * cy + sr * cp * sy;  // q2 (y)
    init_quaternion[3] = cr * cp * sy - sr * sp * cy;  // q3 (z)

    // 初始化QuaternionEKF
    // 参数: init_quaternion, process_noise1, process_noise2, measure_noise, lambda, lpf
    // process_noise1: 四元数过程噪声 (10)
    // process_noise2: 陀螺仪零偏过程噪声 (0.001)
    // measure_noise: 加速度计量测噪声 (1000000)
    // lambda: 渐消因子 (1 = 无渐消，与 basic_framework 一致)
    // lpf: 低通滤波系数 (0 = 不使用低通滤波)
    IMU_QuaternionEKF_Init(init_quaternion, 10.0f, 0.001f, 1000000.0f, 1.0f, 0.0f);

    // 注意：不初始化 GyroBias，让 EKF 从 0 开始估计
    // 因为我们会在传给 EKF 之前先减去校准零偏（特别是 Z 轴）
    // EKF 会将 GyroBias[2] 强制设为 0（无法观测 Yaw 漂移）

    initialized = 1;

    USB_CDC_Printf("[QuaternionEKF] Initialized with q=[%.3f,%.3f,%.3f,%.3f], roll=%.2f°, pitch=%.2f°\r\n",
                   init_quaternion[0], init_quaternion[1], init_quaternion[2], init_quaternion[3],
                   roll * RAD_TO_DEG, pitch * RAD_TO_DEG);
}

/**
 * @brief 设置陀螺仪校准期间的回调函数
 * @param callback 回调函数指针，在每次采样间隔时调用（约1ms间隔）
 */
void gyro_calibrate_set_callback(GyroCalibCallback_t callback)
{
    gyro_calib_callback = callback;
}

/**
 * @brief IMU校准（与basic_framework的Calibrate_MPU_Offset相似）
 * @attention 调用此函数时，IMU必须静止不动
 * @note 采集2000个样本（约2秒），计算陀螺仪零偏和加速度计缩放
 */
void gyro_calibrate(void)
{
    #define CALIB_SAMPLES 2000    // 2000样本（约2秒）
    #define CALIB_TIMEOUT_MS 15000  // 15秒超时
    #define MAX_RETRY 3             // 最多重试3次

    uint32_t start_time = BspTime_NowMs();
    uint16_t cali_count = 0;

    float gyro_max[3], gyro_min[3];
    float gyro_diff[3];
    float g_norm_temp = 0.0f, g_norm_max = 0.0f, g_norm_min = 0.0f, g_norm_diff = 0.0f;

    // 验证BMI088是否初始化成功
    USB_CDC_Printf("[BMI088] Verifying BMI088 initialization...\r\n");
    BMI088_read(gyro, accel, &temp);
    USB_CDC_Printf("[BMI088] Initial reading: accel=[%.3f,%.3f,%.3f]m/s², gyro=[%.3f,%.3f,%.3f]rad/s, temp=%.1f°C\r\n",
                   accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp);

    if (accel[0] == 0.0f && accel[1] == 0.0f && accel[2] == 0.0f) {
        USB_CDC_Printf("[BMI088] *** ERROR: Accelerometer reading is ALL ZERO! ***\r\n");
        USB_CDC_Printf("[BMI088] *** BMI088 accelerometer initialization may have failed! ***\r\n");
        USB_CDC_Printf("[BMI088] *** Check SPI connection and BMI088_init() return value ***\r\n");
    }

    if (temp < -50.0f || temp > 100.0f) {
        USB_CDC_Printf("[BMI088] *** WARNING: Temperature reading abnormal (%.1f°C) ***\r\n", temp);
    }

    USB_CDC_Printf("[BMI088] Starting calibration, keep IMU still for 2 seconds...\r\n");
    Buzzer_PlayBeep();

    // do-while循环：重试直到校准成功或超时
    do {
        // 检查超时或重试次数
        if (BspTime_NowMs() - start_time > CALIB_TIMEOUT_MS || cali_count >= MAX_RETRY) {
            if (cali_count >= MAX_RETRY) {
                USB_CDC_Printf("[BMI088] Max retry reached! Using last attempt values.\r\n");
                // 使用最后一次尝试的值（总比默认值好）
                accel_scale = GRAVITY_ACCEL / g_norm;
                calibrated = 1;
            } else {
                USB_CDC_Printf("[BMI088] Calibration timeout! Using default values.\r\n");
                gyro_offset[0] = 0.0f;
                gyro_offset[1] = 0.0f;
                gyro_offset[2] = 0.0f;
                g_norm = GRAVITY_ACCEL;  // 使用标准重力加速度
                accel_scale = 1.0f;
                temp = 40.0f;
                calibrated = 0;
            }
            break;
        }

        if (cali_count > 0) {
            BspTime_DelayMs(100);  // 重试前延时100ms
        }

        // 重置累积变量
        g_norm = 0.0f;
        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;

        USB_CDC_Printf("[BMI088] Attempt %d: Collecting %d samples...\r\n", cali_count + 1, CALIB_SAMPLES);

        // 采集2000个样本
        for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
            BMI088_read(gyro, accel, &temp);

            // 累积加速度计范数（重力）
            // 注意：BMI088_read返回的accel单位是m/s²
            g_norm_temp = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
            g_norm += g_norm_temp;

            // 首次采样时打印调试信息
            if (i == 0 && cali_count == 0) {
                USB_CDC_Printf("[BMI088] First sample: accel=[%.3f,%.3f,%.3f]m/s², gyro=[%.3f,%.3f,%.3f]rad/s, temp=%.1f°C\r\n",
                               accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp);
            }

            // 累积陀螺仪零偏
            gyro_offset[0] += gyro[0];
            gyro_offset[1] += gyro[1];
            gyro_offset[2] += gyro[2];

            // 记录最大最小值
            if (i == 0) {
                g_norm_max = g_norm_temp;
                g_norm_min = g_norm_temp;
                gyro_max[0] = gyro[0];
                gyro_max[1] = gyro[1];
                gyro_max[2] = gyro[2];
                gyro_min[0] = gyro[0];
                gyro_min[1] = gyro[1];
                gyro_min[2] = gyro[2];
            } else {
                if (g_norm_temp > g_norm_max) g_norm_max = g_norm_temp;
                if (g_norm_temp < g_norm_min) g_norm_min = g_norm_temp;

                for (uint8_t j = 0; j < 3; j++) {
                    if (gyro[j] > gyro_max[j]) gyro_max[j] = gyro[j];
                    if (gyro[j] < gyro_min[j]) gyro_min[j] = gyro[j];
                }
            }

            // 每隔500个样本检查一次运动（减少检查频率，提高效率）
            if (i % 500 == 499) {
                g_norm_diff = g_norm_max - g_norm_min;
                gyro_diff[0] = gyro_max[0] - gyro_min[0];
                gyro_diff[1] = gyro_max[1] - gyro_min[1];
                gyro_diff[2] = gyro_max[2] - gyro_min[2];

                if (g_norm_diff > 0.5f ||  // 重力范数变化 < 0.5 m/s²
                    gyro_diff[0] > 0.15f ||
                    gyro_diff[1] > 0.15f ||
                    gyro_diff[2] > 0.15f) {
                    USB_CDC_Printf("[BMI088] Movement detected (gNorm_diff=%.3f m/s², gyro_diff=[%.3f,%.3f,%.3f]rad/s), retry...\r\n",
                                   g_norm_diff, gyro_diff[0], gyro_diff[1], gyro_diff[2]);
                    break;  // 跳出采样循环，重新开始
                }
            }

            // 调用回调函数（如果设置了），用于保持云台位置等
            if (gyro_calib_callback != NULL) {
                gyro_calib_callback();
            }

            BspTime_DelayMs(1);  // 1ms间隔
        }

        // 计算平均值和最终差值
        g_norm /= (float)CALIB_SAMPLES;
        gyro_offset[0] /= (float)CALIB_SAMPLES;
        gyro_offset[1] /= (float)CALIB_SAMPLES;
        gyro_offset[2] /= (float)CALIB_SAMPLES;

        // 计算最终差值用于检查
        g_norm_diff = g_norm_max - g_norm_min;
        gyro_diff[0] = gyro_max[0] - gyro_min[0];
        gyro_diff[1] = gyro_max[1] - gyro_min[1];
        gyro_diff[2] = gyro_max[2] - gyro_min[2];

        USB_CDC_Printf("[BMI088] Sample complete: gNorm=%.3f m/s² (%.2fg), offset=[%.6f,%.6f,%.6f]rad/s\r\n",
                       g_norm, g_norm / GRAVITY_ACCEL, gyro_offset[0], gyro_offset[1], gyro_offset[2]);

        cali_count++;

    } while (g_norm_diff > 0.5f ||                                    // 重力范数变化 < 0.5 m/s²
             fabsf(g_norm - GRAVITY_ACCEL) > 0.5f ||              // gNorm应该接近9.8 m/s²
             gyro_diff[0] > 0.15f ||
             gyro_diff[1] > 0.15f ||
             gyro_diff[2] > 0.15f ||
             fabsf(gyro_offset[0]) > 0.01f ||  // 零偏阈值收紧到0.01（与basic_framework一致）
             fabsf(gyro_offset[1]) > 0.01f ||
             fabsf(gyro_offset[2]) > 0.01f);

    // 如果成功退出循环（不是因为超时/重试），设置校准成功标志
    if (!calibrated && (BspTime_NowMs() - start_time <= CALIB_TIMEOUT_MS && cali_count < MAX_RETRY)) {
        calibrated = 1;
    }

    // 计算加速度计缩放系数（无论是否校准成功都要设置，避免使用未初始化的值）
    if (calibrated) {
        // gNorm是m/s²，accel_scale用于校准（理想情况下接近1.0）
        accel_scale = GRAVITY_ACCEL / g_norm;  // 应该接近1.0
        USB_CDC_Printf("[BMI088] ===== Calibration SUCCESS (attempts: %d) =====\r\n", cali_count);
        USB_CDC_Printf("Gyro offset: gx=%.6f, gy=%.6f, gz=%.6f rad/s\r\n",
                       gyro_offset[0], gyro_offset[1], gyro_offset[2]);
        USB_CDC_Printf("Accel: gNorm=%.3f m/s² (%.2fg)\r\n", g_norm, g_norm / GRAVITY_ACCEL);
        USB_CDC_Printf("Temp when cali: %.2f °C\r\n", temp);
        Buzzer_PlayBeep();
    }
}

void gyro_data_init(void)
{
    BMI088_init();
    initialized = 0;  // 标记为未初始化，等待第一次数据
    last_update_tick = BspTime_NowMs();  // 初始化时间戳
}

void gyro_data_update(SensorData *sensor_data)
{
    // === 调试：确认函数被调用 ===
    static uint32_t debug_counter = 0;
    if (debug_counter < 5) {
        USB_CDC_Printf("[DEBUG] gyro_data_update called, count=%lu, initialized=%d\r\n", debug_counter, initialized);
        debug_counter++;
    }

    // === 动态计算采样周期 ===
    uint32_t current_tick = BspTime_NowMs();
    dt = (current_tick - last_update_tick) * 0.001f;  // ms -> s
    last_update_tick = current_tick;

    // 限制 dt 范围，防止异常值（0.5ms ~ 20ms）
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.02f) dt = 0.02f;

    BMI088_read(gyro, accel, &temp);

    // 保存原始IMU数据（gyro: rad/s, accel: m/s²）
    sensor_data->g_gx = gyro[0];
    sensor_data->g_gy = gyro[1];
    sensor_data->g_gz = gyro[2];
    sensor_data->g_ax = accel[0];
    sensor_data->g_ay = accel[1];
    sensor_data->g_az = accel[2];

    // === 如果未初始化，使用加速度计初始化四元数 ===
    if (!initialized) {
        init_attitude_from_accel(accel[0], accel[1], accel[2]);
        return;
    }

    // === QuaternionEKF姿态更新 ===
    // QuaternionEKF需要的输入：
    // - 陀螺仪：rad/s（需要先减去校准零偏，特别是 Z 轴！）
    // - 加速度计：m/s²（需要应用缩放系数校准）
    // - 采样周期：s
    //
    // 关键：EKF 无法观测 Yaw 轴零偏（GyroBias[2] 总是被设为 0），
    //       所以必须在这里先减去校准零偏，再传给 EKF
    float gyro_calibrated[3], accel_calibrated[3];
    gyro_calibrated[0] = gyro[0] - gyro_offset[0];
    gyro_calibrated[1] = gyro[1] - gyro_offset[1];
    gyro_calibrated[2] = gyro[2] - gyro_offset[2];  // Z 轴最重要！
    accel_calibrated[0] = accel[0] * accel_scale;
    accel_calibrated[1] = accel[1] * accel_scale;
    accel_calibrated[2] = accel[2] * accel_scale;

    // EKF 会在内部再减去 GyroBias（X/Y 轴动态估计，Z 轴恒为 0）
    IMU_QuaternionEKF_Update(gyro_calibrated[0], gyro_calibrated[1], gyro_calibrated[2],
                              accel_calibrated[0], accel_calibrated[1], accel_calibrated[2],
                              dt);  // 使用动态计算的 dt，而不是固定值

    // 从QuaternionEKF获取姿态角度（度）
    sensor_data->yaw = QEKF_INS.Yaw;
    sensor_data->pitch = QEKF_INS.Pitch;
    sensor_data->roll = QEKF_INS.Roll;

    // 获取多圈累积角度（QuaternionEKF已经计算好了）
    sensor_data->yaw_total_angle = QEKF_INS.YawTotalAngle;
    sensor_data->yaw_round_count = QEKF_INS.YawRoundCount;

    // 兼容旧代码
    sensor_data->absolute_angle = sensor_data->yaw_total_angle;

    // === Log IMU data via logger (10Hz rate limited in main.c) ===
    // Format: IMU,timestamp_ms,gimbal_yaw,gimbal_pitch,gimbal_roll,gimbal_yaw_total,
    //         yaw_round_count,g_gx,g_gy,g_gz,chassis_yaw,chassis_pitch,chassis_roll,
    //         c_gx,c_gy,c_gz,c_ax,c_ay,c_az
    // Gimbal (BMI088): yaw, pitch, roll, yaw_total, round_count, gx, gy, gz (rad/s)
    // Chassis (WT61C): yaw, pitch, roll, gx, gy, gz (rad/s), ax, ay, az (m/s²)
    LOG_CSV(LOG_TAG_IMU, "%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
            sensor_data->yaw,
            sensor_data->pitch,
            sensor_data->roll,
            sensor_data->yaw_total_angle,
            sensor_data->yaw_round_count,
            sensor_data->g_gx,
            sensor_data->g_gy,
            sensor_data->g_gz,
            sensor_data->c_yaw,
            sensor_data->c_pitch,
            sensor_data->c_roll,
            sensor_data->c_gx,
            sensor_data->c_gy,
            sensor_data->c_gz,
            sensor_data->c_ax,
            sensor_data->c_ay,
            sensor_data->c_az);

    WT61C_Data const* d = WT61C_GetData();

    // 姿态角：直接使用度数（float）
    sensor_data->c_roll = d->roll;
    sensor_data->c_pitch = d->pitch;
    sensor_data->c_yaw = d->yaw;

    // 陀螺仪：deg/s转换为rad/s（统一单位）
    sensor_data->c_gx = d->gx * DEG_TO_RAD;
    sensor_data->c_gy = d->gy * DEG_TO_RAD;
    sensor_data->c_gz = d->gz * DEG_TO_RAD;

    // 加速度计：已经是m/s²，直接使用
    sensor_data->c_ax = d->ax;
    sensor_data->c_ay = d->ay;
    sensor_data->c_az = d->az;

    (void)MsgCenter_Publish(TOPIC_IMU_UPDATE, sensor_data, sizeof(*sensor_data));
}

void WT61C_OnNewData(const WT61C_Data *d)
{
  (void)d;
}
