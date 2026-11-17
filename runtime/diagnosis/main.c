#include "main.h"
#include "gpio.h"
#include "can.h"
#include "diagnosis.h"
#include "diagnosis_terminal.h"

/* Same 72 MHz SYSCLK / 36 MHz APB1 tree as the frozen CubeMX main. Keeping
 * this composition root separate avoids changing the generated baseline.
 */
static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clock = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 6;
    osc.PLL.PLLN = 72;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

int main(void)
{
    HAL_Init();
    clock_init();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    Diagnosis_Init();
    DiagnosisTerminal_Init();
    uint32_t last_led = 0;
    for (;;) {
        uint32_t now = HAL_GetTick();
        DiagnosisTerminal_Step(now);
        if ((uint32_t)(now - last_led) >= 500U) {
            last_led = now;
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
        }
        /* CAN reception is interrupt driven; RTT input/output is nonblocking. */
        HAL_Delay(1);
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
void CAN2_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan2); }
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }
void Error_Handler(void) { __disable_irq(); for (;;) { } }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
