/*
 * 封装 BMI088 使用的 SPI、片选和延时操作。
 * 它只连接驱动与 HAL，不包含姿态计算。
 */
#include "bsp_bmi088.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }


}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

uint8_t BMI088_ACCEL_NS_READ(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET ? 1U : 0U;
}

uint8_t BMI088_GYRO_NS_READ(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET ? 1U : 0U;
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

