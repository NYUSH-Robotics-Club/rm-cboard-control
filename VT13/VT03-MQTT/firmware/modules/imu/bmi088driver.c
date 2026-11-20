/*
 * 初始化并读取板载 BMI088 加速度计和陀螺仪。
 * 寄存器通信通过独立 BSP 完成。
 */
#include "bmi088driver.h"
#include "bmi088reg.h"
#include "bsp_bmi088.h"
#include "bsp_time.h"
#include "logger.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_write_byte(0x55);              \
        uint8_t __i;                               \
        for (__i = 0; __i < (len); __i++) {        \
            (data)[__i] = BMI088_read_write_byte(0x55); \
        }                                          \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)

#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    BMI088_GPIO_init();
    BMI088_com_init();
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    return error;
}

/**
 * @brief Initialize BMI088 with comprehensive diagnostics and CS pin testing
 * @return uint8_t Error code (0 = success, non-zero = error)
 *
 * This function performs:
 * - CS pin functionality tests for both accelerometer and gyroscope
 * - BMI088 sensor initialization
 * - Detailed error reporting and diagnostics via USB CDC
 */
uint8_t BMI088_InitWithDiagnostics(void)
{
    LOG_INFO(LOG_TAG_IMU, "");
    LOG_INFO(LOG_TAG_IMU, "=== BMI088 Initialization ===");

    // Test CS pins
    LOG_INFO(LOG_TAG_IMU, "Testing CS pins...\r\n");
    LOG_INFO(LOG_TAG_IMU, "Testing ACCEL CS (PA4)...\r\n");
    BMI088_ACCEL_NS_H();
    BspTime_DelayMs(10);
    uint8_t accel_cs_state = BMI088_ACCEL_NS_READ();
    LOG_INFO(LOG_TAG_IMU, "ACCEL CS HIGH: %s\r\n", accel_cs_state ? "OK" : "FAIL");

    BMI088_ACCEL_NS_L();
    BspTime_DelayMs(10);
    accel_cs_state = BMI088_ACCEL_NS_READ();
    LOG_INFO(LOG_TAG_IMU, "ACCEL CS LOW: %s\r\n", !accel_cs_state ? "OK" : "FAIL");
    BMI088_ACCEL_NS_H();

    LOG_INFO(LOG_TAG_IMU, "Testing GYRO CS (PB0)...\r\n");
    BMI088_GYRO_NS_H();
    BspTime_DelayMs(10);
    uint8_t gyro_cs_state = BMI088_GYRO_NS_READ();
    LOG_INFO(LOG_TAG_IMU, "GYRO CS HIGH: %s\r\n", gyro_cs_state ? "OK" : "FAIL");

    BMI088_GYRO_NS_L();
    BspTime_DelayMs(10);
    gyro_cs_state = BMI088_GYRO_NS_READ();
    LOG_INFO(LOG_TAG_IMU, "GYRO CS LOW: %s\r\n", !gyro_cs_state ? "OK" : "FAIL");
    BMI088_GYRO_NS_H();

    // Initialize BMI088
    uint8_t bmi088_error = BMI088_init();
    LOG_INFO(LOG_TAG_IMU, "BMI088_init() returned: 0x%02X\r\n", bmi088_error);

    // Detailed error reporting
    if (bmi088_error != 0) {
        LOG_INFO(LOG_TAG_IMU, "*** ERROR: BMI088 initialization failed! Error code: 0x%02X ***\r\n", bmi088_error);
        LOG_INFO(LOG_TAG_IMU, "Error details:\r\n");
        if (bmi088_error & 0x01) LOG_INFO(LOG_TAG_IMU, "  BMI088_ACC_PWR_CTRL_ERROR\r\n");
        if (bmi088_error & 0x02) LOG_INFO(LOG_TAG_IMU, "  BMI088_ACC_PWR_CONF_ERROR\r\n");
        if (bmi088_error & 0x04) LOG_INFO(LOG_TAG_IMU, "  BMI088_ACC_CONF_ERROR\r\n");
        if (bmi088_error & 0x08) LOG_INFO(LOG_TAG_IMU, "  BMI088_ACC_SELF_TEST_ERROR\r\n");
        if (bmi088_error & 0x10) LOG_INFO(LOG_TAG_IMU, "  BMI088_ACC_RANGE_ERROR\r\n");
        if (bmi088_error & 0x20) LOG_INFO(LOG_TAG_IMU, "  BMI088_INT1_IO_CTRL_ERROR\r\n");
        if (bmi088_error & 0x40) LOG_INFO(LOG_TAG_IMU, "  BMI088_INT_MAP_DATA_ERROR\r\n");
        if (bmi088_error & 0x80) LOG_INFO(LOG_TAG_IMU, "  GYRO initialization errors\r\n");
    } else {
        LOG_INFO(LOG_TAG_IMU, "BMI088 initialization SUCCESS!\r\n");
    }

    LOG_INFO(LOG_TAG_IMU, "=== BMI088 Init Complete ===");
    LOG_INFO(LOG_TAG_IMU, "");

    return bmi088_error;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t error = BMI088_NO_ERROR;

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // Debug: print CHIP_ID
    LOG_DEBUG(LOG_TAG_IMU, "accel_init] CHIP_ID after reset: 0x%02X (expected 0x%02X)\r\n",
                   res, BMI088_ACC_CHIP_ID_VALUE);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // Configure registers and accumulate errors (don't return on first failure)
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);

        // PWR_CTRL and PWR_CONF need longer delay per BMI088 datasheet
        if (write_reg_num == 0 || write_reg_num == 1) {
            BMI088_delay_ms(5);  // 5ms for power management registers
        } else {
            BMI088_delay_ms(1);
        }

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // Debug: print register verification
        LOG_DEBUG(LOG_TAG_IMU, "accel_init] Reg 0x%02X: wrote 0x%02X, read 0x%02X %s\r\n",
                       write_BMI088_accel_reg_data_error[write_reg_num][0],
                       write_BMI088_accel_reg_data_error[write_reg_num][1],
                       res,
                       (res == write_BMI088_accel_reg_data_error[write_reg_num][1]) ? "OK" : "FAIL");

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return error;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;
    uint8_t error = BMI088_NO_ERROR;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // Configure registers and accumulate errors (don't return on first failure)
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_ms(1);  // Increase delay for consistency
        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            error |= write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }
    return error;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    // Debug: print raw buffer (first 3 times)
    static uint8_t debug_count = 0;
    if (debug_count < 3) {
        LOG_DEBUG(LOG_TAG_IMU, "read] Accel raw: %02X %02X %02X %02X %02X %02X\r\n",
                       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        debug_count++;
    }

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
