/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088driver.h"
#include "buzzer.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "remote_control.h"
#include "bsp_rc.h"
#include "chassis_controller.h"
#include "shooter_controller.h"
#include "gimbal_controller.h"
#include "motor_driver.h"
#include "can_manager.h"
#include "motor_registry.h"
#include "robot_config.h"
#include <stdarg.h>
#include <math.h>
#include "printing.h"
#include "wt61c.h"
#include "gyro_data.h"
#include "message_center.h"
#include "app_subscriptions.h"
#include "cmd_controller.h"
#include "vision_comm.h"
#include "logger.h"
#include "robot_rtos.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Minimized delays for maximum response speed
#define WAIT_ESC_BOOT_MS                (200U)  // Reduced from 500ms → 200ms
// RC loss timeout for health gating
#define RC_LOSS_TIMEOUT_MS              (200U)
// USART6 hello message send interval
#define USART6_SEND_INTERVAL_MS         (1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// CAN managers
CAN_Manager_t can1_manager;
CAN_Manager_t can2_manager;

// Motor registry storage (one per CAN channel)
static MotorRegistry_t can1_registry;
static MotorRegistry_t can2_registry;

float gyro[3], accel[3], temp;

// WT61C-TTL IMU sensor on USART1
#define WT61C_UART_HANDLE  huart1
#define RX_DMA_BUF_SZ 256
static uint8_t wt61c_rxbuf[RX_DMA_BUF_SZ];

// Message center buffer
#define MSG_CENTER_QUEUE_LEN 128
static MsgEvent g_msg_queue[MSG_CENTER_QUEUE_LEN];

// RC logging
static uint32_t s_rc_frame_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);
static void Gimbal_HoldPosition_Callback(void);
static void on_rc_update(const MsgEvent *ev, void *user);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
CAN receive callback - delegate to CAN manager
这个函数是stm官方自带的软函数，作用就是如果有更新可以中断主循环
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /*这个函数查看can manager.c的定义，*/
  CAN_Manager_GlobalCallback(hcan);
}

/**
 * @brief RC update callback for logging (runs in dispatch context, not ISR)
 */
static void on_rc_update(const MsgEvent *ev, void *user)
{
  (void)user;
  if (ev->size == sizeof(RC_ctrl_t)) {
    const RC_ctrl_t *rc = (const RC_ctrl_t *)ev->data;
    s_rc_frame_counter++;

    
  /*这个只是日志打印，具体查看remote_control的定义*/
    // Log RC data from the dispatch caller.
    LOG_CSV(LOG_TAG_RC, "%lu,%d,%d,%d,%d,%d,%u,%u",
            (unsigned long)s_rc_frame_counter,
            (int)rc->rc.ch[0], (int)rc->rc.ch[1],
            (int)rc->rc.ch[2], (int)rc->rc.ch[3],
            (int)rc->rc.ch[4],
            (unsigned int)rc->rc.s[0], (unsigned int)rc->rc.s[1]);
  }
}

/**
 * @brief Callback function to hold gimbal position during calibration
 * @note This function is called every ~1ms during gyro calibration to keep
 *       the gimbal motors actively holding their position.
 * 初始化时防止因重力低头的程序，每5ms一次，个人认为应该优化，至少不应该放在这里
 */
static void Gimbal_HoldPosition_Callback(void)
{
  static uint32_t call_count = 0;
  call_count++;

  // Only send command and dispatch every 5ms to reduce overhead
  if (call_count % 5 == 0) {
    GimbalCmd cmd = {
      .enabled = true,
      .pitch_rate = 0.0f,
      .yaw_rate = 0.0f,
      .yaw_rate_memo = 0.0f,
      .yaw_target_memo = 0.0f,
      .vision_valid = false,
      .vision_yaw_err_rad = 0.0f,
      .vision_pitch_err_rad = 0.0f,
      .vision_ts_ms = 0
    };

    MsgCenter_Publish(TOPIC_GIMBAL_CMD, &cmd, sizeof(cmd));
    MsgCenter_Dispatch();  // Process messages immediately
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Optional: initial tiny transmission (currently disabled)
  // USB_CDC_SendString("\r\n");

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  // === LED: RED - Hardware initialized, starting software init 这是最简洁的灯语控制，可以修改三个参数自己组合颜色判断状态 ===
  LED_SetRGB(1, 0, 0);

  // Print boot message
  LOG_INFO(LOG_TAG_SYS, "\r\n========================================");
  LOG_INFO(LOG_TAG_SYS, "   RoboMaster Control System Boot");
  LOG_INFO(LOG_TAG_SYS, "========================================");

  // Initialize buzzer
  Buzzer_Init();

  // Initialize BMI088 with diagnostics
  BMI088_InitWithDiagnostics();

  // Initialize message center (needed for gimbal communication)
  MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN);
  remote_control_init();

  // Initialize logger module
  Logger_Init();
  // Configure logger rates for different subsystems，去logger里面找具体定义
  Logger_SetRate(LOG_TAG_CMD, 100);   // 10Hz for command controller CSV (SPINDBG)
  Logger_SetRate(LOG_TAG_IMU, 100);   // 10Hz for IMU CSV data
  Logger_SetRate(LOG_TAG_GIM, 50);    // 20Hz for gimbal PID tuning (PITCH/YAW_CSV)
  Logger_SetRate(LOG_TAG_RC, 0);      // No rate limit for RC (full 100Hz+ output)
  Logger_SetRate(LOG_TAG_CHA, 200);   // 5Hz for chassis status
  Logger_SetRate(LOG_TAG_SEN, 200);   // 5Hz for sentry status
  Logger_SetRate(LOG_TAG_SYS, 0);     // No rate limit for system messages
  Logger_SetRate(LOG_TAG_MOT, 0);     // No rate limit for motor init messages

  // === LED: YELLOW - CAN and motor initialization ===
  LED_SetRGB(1, 1, 0);

  // Initialize CAN managers early (needed for gimbal motors)
  const RobotConfig_t *robot_cfg = RobotConfig_Get();
  CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &hcan1, robot_cfg, &can1_registry);
  CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &hcan2, robot_cfg, &can2_registry);
  CAN_Manager_Start(&can1_manager);
  CAN_Manager_Start(&can2_manager);

  // Initialize motor driver module (loads config and initializes all motors)
  MotorDriver_ModuleInit();

  // Initialize gimbal early (before calibration)
  GimbalApp_Init();

  // CAN bus stabilization (removed delay for faster boot)
  LOG_INFO(LOG_TAG_SYS, "Starting system...");
  // HAL_Delay(200);  // Removed - CAN bus starts immediately

  // === LED: CYAN - Gimbal alignment ===
  LED_SetRGB(0, 1, 1);

  // Wait for gimbal to reach initial alignment position (if gimbal exists)
  if (robot_cfg->gimbal_motor_count > 0) {
    Gimbal_WaitForAlignment();
  }

  // Perform IMU calibration if enabled in robot configuration
  if (robot_cfg->enable_imu_calibration) {
    // Now start IMU calibration with gimbal in position
    LOG_INFO(LOG_TAG_SYS, "");
    LOG_INFO(LOG_TAG_SYS, "=== Starting IMU Calibration ===");
    LOG_INFO(LOG_TAG_SYS, "Keep the robot still!");

    // Set LED to blue during calibration
    LED_SetRGB(0, 0, 1);

    // Set callback to keep gimbal holding position during calibration
    if (robot_cfg->gimbal_motor_count > 0) {
      gyro_calibrate_set_callback(Gimbal_HoldPosition_Callback);
    }

    // Perform gyro calibration
    gyro_calibrate();

    LOG_INFO(LOG_TAG_SYS, "=== IMU Calibration Complete ===");
    LOG_INFO(LOG_TAG_SYS, "");

    // Clear the callback since CmdController will take over
    gyro_calibrate_set_callback(NULL);
  } else {
    LOG_INFO(LOG_TAG_SYS, "IMU calibration disabled for this robot type");
  }

  // === LED: MAGENTA - Application controllers initialization ===
  LED_SetRGB(1, 0, 1);

  // Continue with remaining initialization
  // Initialize command controller (central control)
  CmdController_Init();

  // Initialize remaining application controllers
  ChassisApp_Init();
  ShooterApp_Init();

  // Align swerve steer motors to initial position (sentry_swerve only)
  #if defined(ROBOT_TYPE_sentry_swerve)
  // LED: PURPLE (dimmer magenta) - Swerve steer alignment
  LED_SetRGB(1, 0, 1);
  Sentry_WaitForSteerAlignment();
  #endif

  // === LED: WHITE - Final peripherals initialization ===
  LED_SetRGB(1, 1, 1);

  // Subscribe to RC updates for logging (in dispatch context, not ISR)
  MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);

  // Initialize Vision Communication
  VisionComm_Init();

  // Wait for ESC boot
  HAL_Delay(WAIT_ESC_BOOT_MS);

  // Initialize WT61C-TTL IMU sensor on USART1
  WT61C_Init(&WT61C_UART_HANDLE);
  // Start UART DMA reception with idle line detection
  HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
  // Disable half-transfer interrupt to reduce callback overhead
  __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);

  // Initialize optional application modules before their RTOS step runs.
  RobotStatus app_runtime_status = AppRuntime_Init();
  if (app_runtime_status != ROBOT_STATUS_OK) {
    LOG_ERROR(LOG_TAG_SYS, "Optional application init failed: %d",
              (int)app_runtime_status);
  }

  LOG_INFO(LOG_TAG_SYS, "");
  LOG_INFO(LOG_TAG_SYS, "=== System Ready ===");
  LOG_INFO(LOG_TAG_SYS, "");

  // === LED: GREEN - System ready, starting the RTOS scheduler ===
  LED_SetRGB(0, 1, 0);

  // The static control task takes over the former main-loop work.
  if (!RobotRtos_Start()) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // The scheduler should never return; failures are handled above.
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
Set LED
*/
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, r ? GPIO_PIN_SET : GPIO_PIN_RESET); // R
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, g ? GPIO_PIN_SET : GPIO_PIN_RESET); // G
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, b ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
}

/*
UART DMA/Idle callback for WT61C sensor data reception
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    BspRc_OnRxEvent(Size);
    return;
  }
  if (huart == &WT61C_UART_HANDLE) {
    // Process received data
    WT61C_ProcessBytes(wt61c_rxbuf, Size);
    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&WT61C_UART_HANDLE, wt61c_rxbuf, RX_DMA_BUF_SZ);
    __HAL_DMA_DISABLE_IT(WT61C_UART_HANDLE.hdmarx, DMA_IT_HT);
  }
  // Note: Vision communication now uses USB CDC instead of USART6
  // The CDC_Receive_FS callback in usbd_cdc_if.c handles vision data reception
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
