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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "app_display.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "max30102.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_UI_REFRESH_PERIOD_MS 200U
#define APP_MAIN_LOOP_DELAY_MS   5U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 初始化共享状态，并让各模块完成自己的默认配置。 */
static void app_state_init(AppState_t *app);
/* 若当前轮次需要上报，则发送一帧测量报文。 */
static void app_send_report_if_due(AppState_t *app);
/* 若当前轮次需要刷新显示，则重绘当前测量页面。 */
static void app_refresh_display_if_needed(AppState_t *app);
static void app_schedule_periodic_refresh(AppState_t *app);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 应用层统一初始化入口，main 中只保留调度。 */
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

/* 把“是否上报”的时序控制收口到一个函数里，避免主循环继续膨胀。 */
static void app_send_report_if_due(AppState_t *app)
{
  if ((app == NULL) || (app->report_due == 0U))
  {
    return;
  }

  app_protocol_send_sensor_report(app);
  app->report_due = 0U;
}

/* OLED 刷新单独封装，让主循环更像调度器而不是细节堆场。 */
static void app_refresh_display_if_needed(AppState_t *app)
{
  if ((app == NULL) || (app->display_refresh_requested == 0U))
  {
    return;
  }

  app_protocol_update_rtc_snapshot(app);
  app_display_measurement_page(app);
  app->display_refresh_count++;
  app->display_last_refresh_tick = HAL_GetTick();
  app->display_refresh_requested = 0U;
}

static void app_schedule_periodic_refresh(AppState_t *app)
{
  static uint32_t last_refresh_tick = 0U;
  uint32_t now;

  if (app == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - last_refresh_tick) < APP_UI_REFRESH_PERIOD_MS)
  {
    return;
  }

  last_refresh_tick = now;
  app->report_due = 1U;
  app->display_refresh_requested = 1U;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  AppState_t app;
  char status_line[32];
  uint32_t last_status_tick;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  app_state_init(&app);
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
  /* USER CODE BEGIN 2 */
  /* 启动显示、读取 RTC，并先给出开机状态页。 */
  ssd1306_Init();
  app_protocol_update_rtc_snapshot(&app);
  app_display_status_page(&app, "MAX30102 INIT", "SYSTEM BOOT");

  /* 传感器初始化失败时，保留串口命令与错误状态显示，便于现场排查。 */
  if (max30102_init() != HAL_OK)
  {
    while (1)
    {
      app_protocol_poll_uart_commands(&app);
      app_display_handle_buttons(&app);
      app_protocol_update_rtc_snapshot(&app);
      app_display_status_page(&app, "MAX30102 ERR", "CHECK SENSOR");
      HAL_Delay(200U);
    }
  }

  app_measurement_reset_runtime();
  last_status_tick = 0U;

  /* 上电先采集一段“无手指”背景，建立 IR 基线。 */
  while (app_measurement_baseline_ready() == 0U)
  {
    app_protocol_poll_uart_commands(&app);
    app_display_handle_buttons(&app);
    /* Drain every pending sample so the baseline window stays aligned with the sensor FIFO. */
    while ((app_measurement_baseline_ready() == 0U) &&
           (app_measurement_collect_baseline_sample(&app) != 0U))
    {
    }

    if ((HAL_GetTick() - last_status_tick) >= 200U)
    {
      last_status_tick = HAL_GetTick();
      app_protocol_update_rtc_snapshot(&app);
      (void)snprintf(status_line,
                     sizeof(status_line),
                     "BASE %uP",
                     (unsigned int)app_measurement_get_baseline_progress_percent());
      app_display_status_page(&app, "KEEP FINGER OFF", status_line);
    }

    HAL_Delay(APP_MAIN_LOOP_DELAY_MS);
  }

  /* 基线就绪后，给后台跟踪器播种并立即发送一帧初始状态。 */
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

  /* 根据采集到的波动范围提示“稳定 / 噪声偏大”。 */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * 主循环保持为“调度器”角色：
     * - 轮询串口命令
     * - 处理页面切换
     * - 在有新样本时推进测量状态机
     * - 按较低频率统一上报与刷新 OLED
     */
    app_protocol_poll_uart_commands(&app);
    app_display_handle_buttons(&app);

    switch (app_measurement_read_sensor_sample(&app))
    {
      case APP_MEASUREMENT_READ_OK:
        app_measurement_update_adaptive_thresholds(&app);
        app_measurement_update_finger_state(&app);
        app_measurement_process(&app);
        app_measurement_update_periodic_flags(&app);
        break;

      case APP_MEASUREMENT_READ_ERROR:
        app_measurement_recover_sensor(&app);
        break;

      default:
        break;
    }

    app_schedule_periodic_refresh(&app);
    app_send_report_if_due(&app);
    app_refresh_display_if_needed(&app);
    HAL_Delay(APP_MAIN_LOOP_DELAY_MS);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

