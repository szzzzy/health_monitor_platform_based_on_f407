/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口与应用层调度
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_ecg.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "app_rtos.h"
#include "app_runtime.h"
#include "app_sched_diag.h"
#include "app_sd_card.h"
#include "iwdg.h"
#include "max30102.h"
#include "ssd1306.h"
#include "app_diag.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_MAIN_LOOP_DELAY_MS      5U
#define APP_SENSOR_DRAIN_BUDGET     24U
#define APP_SENSOR_BOOT_RETRY_MS    1000U
#define APP_DISPLAY_SKIP_THRESHOLD  8U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t  tim6_tick_flag = 0;
static volatile uint32_t tim6_isr_count = 0U;  /* TIM6 ISR 回调计数，仅供调试器观测 */
static AppState_t app;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* 初始化共享状态，并让各模块完成自己的默认配置 */
static void app_state_init(AppState_t *app);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 应用层统一初始化入口，main 中只保留调度 */
/* ---- 初始化共享 AppState 与所有子系统默认值 ---- */
static void app_state_init(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  (void)memset(app, 0, sizeof(*app));
  app_measurement_init_state(app);
  app_display_init_state(app);
  app_protocol_init();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char status_line[32];
  uint32_t last_status_tick;
  HAL_StatusTypeDef sensor_init_status;
  HAL_StatusTypeDef sensor_probe_status;
  uint32_t sensor_init_i2c_error;
  uint32_t sensor_init_retry_count;
  uint8_t sensor_part_id;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  app_state_init(&app);
  /* 尽早启用备份域写访问，确保后续 fault handler 可以写 BKP */
  APP_Diag_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* RTC 已就绪，读取上一轮崩溃记录 + 活体快照 (BKP 寄存器) */
  APP_Diag_ReadCrashToAppState(&app);
  APP_Watchdog_Refresh();
  /* TIM6 中断稍后在 RTOS 就绪后启动，避免 ISR 在调度器启动前调用 FreeRTOS API */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
  /* 启动显示、读取 RTC，并先给出开机状态页面
   * 先恢复 I2C1 总线，确保 OLED 和 MAX30102 不会因为上电总线卡死而初始化失败。 */
  (void)MX_I2C1_RecoverBus();
  app.i2c_recover_count++;
  ssd1306_Init();
  app_protocol_update_rtc_snapshot(&app);
  app_display_status_page(&app, "MAX30102 INIT", "SYSTEM BOOT");

  /* 传感器初始化失败时，保留串口命令与错误状态显示，便于现场排查 */
  sensor_init_retry_count = 0UL;
  sensor_init_status = max30102_init();
  while (sensor_init_status != HAL_OK)
  {
    sensor_init_retry_count++;
    sensor_init_i2c_error = HAL_I2C_GetError(&hi2c1);
    sensor_part_id = 0U;
    sensor_probe_status = max30102_read_reg(MAX30102_REG_PART_ID, &sensor_part_id);
    app_protocol_poll_uart_commands(&app);
    app_display_handle_buttons(&app);
    app_protocol_update_rtc_snapshot(&app);
    if (sensor_probe_status == HAL_OK)
    {
      (void)snprintf(status_line,
                     sizeof(status_line),
                     "S%u E%lu ID%02X R%lu",
                     (unsigned int)sensor_init_status,
                     (unsigned long)sensor_init_i2c_error,
                     (unsigned int)sensor_part_id,
                     (unsigned long)sensor_init_retry_count);
    }
    else
    {
      (void)snprintf(status_line,
                     sizeof(status_line),
                     "S%u E%lu P%u R%lu",
                     (unsigned int)sensor_init_status,
                     (unsigned long)sensor_init_i2c_error,
                     (unsigned int)sensor_probe_status,
                     (unsigned long)sensor_init_retry_count);
    }
    app_display_status_page(&app, "MAX30102 ERR", status_line);
    APP_Watchdog_Refresh();
    HAL_Delay(APP_SENSOR_BOOT_RETRY_MS);
    APP_Watchdog_Refresh();
    (void)MX_I2C1_RecoverBus();
    app.i2c_recover_count++;
    sensor_init_status = max30102_init();
  }

  /* MAX 恢复过程中可能执行了 I2C 总线恢复，OLED 可能在前面初始化失败。
   * 总线恢复后重新初始化 OLED，避免黑屏。 */
  if (app.oled_reinit_count < 0xFFFFFFFFUL)
  {
    app.oled_reinit_count++;
  }
  ssd1306_Init();

  app_measurement_reset_runtime();
  last_status_tick = 0U;

  /* SD 卡日志：延迟到首次 APP_DataLog_Service() 时懒启动。
   * 避免坏卡/无卡在启动阶段阻塞主功能（MAX30102/OLED/RTC/按键/串口）。
   * 此处仅初始化内部静态变量，不执行硬件访问。 */
  APP_DataLog_Init();
  app_runtime_update_sd_log_status(&app);
  APP_Watchdog_Refresh();

  /* 上电先采集一段"无手指"背景，建立 IR 基线 */
  while (app_measurement_baseline_ready() == 0U)
  {
    app_protocol_poll_uart_commands(&app);
    app_display_handle_buttons(&app);
    /* 排空所有待处理样本，使基线窗口始终与传感器 FIFO 对齐。 */
    while ((app_measurement_baseline_ready() == 0U) &&
           (app_measurement_collect_baseline_sample(&app) != 0U))
    {
      APP_Watchdog_Refresh();
    }

    app_measurement_service_sensor_watchdog(&app);

    if ((HAL_GetTick() - last_status_tick) >= 200U)
    {
      last_status_tick = HAL_GetTick();
      app_protocol_update_rtc_snapshot(&app);
      (void)snprintf(status_line,
                     sizeof(status_line),
                     "BASE %uP RC:%lu",
                     (unsigned int)app_measurement_get_baseline_progress_percent(),
                     (unsigned long)app.sensor_recover_count);
      app_display_status_page(&app, "KEEP FINGER OFF", status_line);
    }
    APP_Watchdog_Refresh();
    HAL_Delay(APP_MAIN_LOOP_DELAY_MS);
  }

  /* 基线就绪后，给后台跟踪器播种并立即发送一帧初始状态 */
  app.baseline_ir = app_measurement_get_baseline_average();
  app.baseline_range_ir = app_measurement_get_baseline_range();
  {
    uint32_t initial_noise_seed = (app.baseline_range_ir / 8U) + 1U;

    if (initial_noise_seed < 512U)
    {
      initial_noise_seed = 512U;
    }
    else if (initial_noise_seed > 3000U)
    {
      initial_noise_seed = 3000U;
    }

    app_measurement_seed_baseline_tracking(app.baseline_ir, initial_noise_seed);
  }
  app.baseline_ir = app_measurement_get_tracked_baseline();
  app_protocol_send_sensor_report(&app);

  /* 根据采集到的波动范围提示”稳定” / “噪声偏大” */
  (void)snprintf(status_line, sizeof(status_line), "BASE:%lu", (unsigned long)app.baseline_ir);
  if (app_measurement_baseline_is_stable() != 0U)
  {
    app_display_status_page(&app, "BASELINE OK", status_line);
  }
  else
  {
    app_display_status_page(&app, "BASELINE NOISY", status_line);
  }

  app.report_due = 1U;
  app.display_refresh_requested = 1U;

  /* ECG ADC 延后到 PPG 基线完成后启动，
     避免基线阶段约 5 秒无人消费样本时必然产生 DMA 溢出。 */
  app_ecg_adc_start();
  app_rtos_bind_state(&app);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000U);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint32_t APP_TIM6_GetIsrCount(void)
{
  return tim6_isr_count;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if ((htim != NULL) && (htim->Instance == TIM6))
  {
    tim6_isr_count++;
    tim6_tick_flag = 1U;
    max30102_mark_data_ready_from_isr();
    app_rtos_notify_max_from_isr();
    return;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  APP_Diag_CaptureCrash(DIAG_CRASH_ERROR_HANDLER, 0U, 0U);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
