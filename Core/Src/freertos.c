/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_rtos.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_ecg.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "iwdg.h"
#include "max30102.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_RTOS_MAX_NOTIFY_TIMEOUT_MS 20U
#define APP_RTOS_MAX_I2C_TIMEOUT_MS    8U
#define APP_RTOS_UI_PERIOD_MS          20U
#define APP_RTOS_DISPLAY_PERIOD_MS     200U
#define APP_RTOS_SD_PERIOD_MS          20U
#define APP_RTOS_WATCHDOG_PERIOD_MS    50U
#define APP_DISPLAY_SKIP_THRESHOLD     8U
#define APP_SD_SAFE_SAMPLE_AGE_MS      25U
#define APP_DISPLAY_FORCE_REFRESH_MS   1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static AppState_t *app_rtos_state = NULL;
static volatile uint8_t app_rtos_last_fifo_batch_count = 0U;
static volatile uint8_t app_rtos_ready = 0U;

void app_rtos_mark_ready(void)
{
  app_rtos_ready = 1U;
}
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MAXtask */
osThreadId_t MAXtaskHandle;
const osThreadAttr_t MAXtask_attributes = {
  .name = "MAXtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for watchdogtask */
osThreadId_t watchdogtaskHandle;
const osThreadAttr_t watchdogtask_attributes = {
  .name = "watchdogtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Uitask */
osThreadId_t UitaskHandle;
const osThreadAttr_t Uitask_attributes = {
  .name = "Uitask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SDtask */
osThreadId_t SDtaskHandle;
const osThreadAttr_t SDtask_attributes = {
  .name = "SDtask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t app_rtos_i2c_acquire(uint32_t timeout_ms);
static void app_rtos_i2c_release(void);
static void app_rtos_send_report_if_due(AppState_t *app);
static void app_rtos_refresh_display_if_needed(AppState_t *app);
static void app_rtos_update_sd_log_status(AppState_t *app);
static uint8_t app_rtos_sd_service_safe(const AppState_t *app);
static void app_rtos_service_max(AppState_t *app);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void app_rtos_bind_state(AppState_t *state)
{
  app_rtos_state = state;
}

void app_rtos_notify_max_from_isr(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if ((app_rtos_ready == 0U) || (MAXtaskHandle == NULL))
  {
    return;
  }

  vTaskNotifyGiveFromISR((TaskHandle_t)MAXtaskHandle,
                         &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

static uint8_t app_rtos_i2c_acquire(uint32_t timeout_ms)
{
  if (i2c1MutexHandle == NULL)
  {
    return 1U;
  }

  return (osMutexAcquire(i2c1MutexHandle, timeout_ms) == osOK) ? 1U : 0U;
}

static void app_rtos_i2c_release(void)
{
  if (i2c1MutexHandle != NULL)
  {
    (void)osMutexRelease(i2c1MutexHandle);
  }
}

static void app_rtos_send_report_if_due(AppState_t *app)
{
  if ((app == NULL) || (app->report_due == 0U))
  {
    return;
  }

  app_protocol_send_sensor_report(app);
  app->report_due = 0U;
}

static void app_rtos_refresh_display_if_needed(AppState_t *app)
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
  app->fifo_high_watermark = 0U;
}

static void app_rtos_update_sd_log_status(AppState_t *app)
{
  DataLogStatus_t st;

  if (app == NULL) { return; }

  APP_DataLog_GetStatus(&st);
  app->sd_buffered  = st.buffered;
  app->sd_dropped   = st.dropped;
  app->sd_written   = st.written;
  app->sd_paused    = st.paused;
  app->sd_error     = st.sd_error;
  app->sd_state     = st.state;
  app->sd_last_write_ms = st.last_write_ms;
  app->sd_total_written = (uint32_t)st.written;
  app->flush_pending = APP_DataLog_IsFlushPending();
}

static uint8_t app_rtos_sd_service_safe(const AppState_t *app)
{
  uint32_t now;

  if (app == NULL) { return 0U; }
  if (app->finger_present != 0U) { return 0U; }
  if (app->sensor_health != (uint8_t)SENSOR_HEALTH_OK) { return 0U; }
  if (app->contact_settle_samples > 0U) { return 0U; }
  /* Refuse SD I/O while the raw signal detector sees a finger
   * (prevent flush/write during the 8-beat confirm window). */
  if (app->raw_signal_present != 0U) { return 0U; }
  if (app->finger_on_confirm_count > 0U) { return 0U; }
  if (app->sensor_fifo_available_samples >= 4U) { return 0U; }
  if (app->sensor_last_sample_tick == 0UL) { return 0U; }

  now = HAL_GetTick();
  if ((now - app->sensor_last_sample_tick) >= APP_SD_SAFE_SAMPLE_AGE_MS)
  {
    return 0U;
  }

  return 1U;
}

static void app_rtos_service_max(AppState_t *app)
{
  uint8_t fifo_batch_count = 0U;

  if (app == NULL)
  {
    return;
  }

  (void)app_ecg_process_samples(app);

  if (max30102_should_service_fifo() != 0U)
  {
    if (app_rtos_i2c_acquire(APP_RTOS_MAX_I2C_TIMEOUT_MS) != 0U)
    {
      fifo_batch_count = app_measurement_drain_fifo_batch(app);

      if ((fifo_batch_count == 0U) &&
          (app->sensor_last_read_status == (uint8_t)APP_MEASUREMENT_READ_ERROR))
      {
        app_measurement_recover_sensor(app);
      }

      app_rtos_i2c_release();
    }
    else if (app->sensor_read_busy_count < 0xFFFFFFFFUL)
    {
      app->sensor_read_busy_count++;
    }
  }

  app_rtos_last_fifo_batch_count = fifo_batch_count;

  if (app_rtos_i2c_acquire(0U) != 0U)
  {
    app_measurement_service_sensor_watchdog(app);
    app_rtos_i2c_release();
  }
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MAXtask */
  MAXtaskHandle = osThreadNew(StartTask02, NULL, &MAXtask_attributes);

  /* creation of watchdogtask */
  watchdogtaskHandle = osThreadNew(StartTask03, NULL, &watchdogtask_attributes);

  /* creation of Uitask */
  UitaskHandle = osThreadNew(StartTask04, NULL, &Uitask_attributes);

  /* creation of SDtask */
  SDtaskHandle = osThreadNew(StartTask05, NULL, &SDtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the MAXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    if (app_rtos_state == NULL)
    {
      osDelay(10);
      continue;
    }

    (void)ulTaskNotifyTake(pdTRUE,
                           pdMS_TO_TICKS(APP_RTOS_MAX_NOTIFY_TIMEOUT_MS));
    app_rtos_service_max(app_rtos_state);
    app_rtos_state->max_task_heartbeat++;
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the watchdogtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    APP_Watchdog_Refresh();
    osDelay(APP_RTOS_WATCHDOG_PERIOD_MS);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Uitask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint32_t last_display_tick = 0U;
  uint32_t last_skip_tick = 0U;

  /* Infinite loop */
  for(;;)
  {
    uint32_t now;
    uint8_t should_skip;
    uint8_t force_refresh;

    if (app_rtos_state == NULL)
    {
      osDelay(10);
      continue;
    }

    app_rtos_state->ui_task_heartbeat++;

    app_protocol_poll_uart_commands(app_rtos_state);
    app_display_handle_buttons(app_rtos_state);

    now = HAL_GetTick();
    if ((now - last_display_tick) >= APP_RTOS_DISPLAY_PERIOD_MS)
    {
      last_display_tick = now;
      app_rtos_state->display_refresh_requested = 1U;
    }

    if (app_rtos_state->display_refresh_requested != 0U)
    {
      should_skip = ((app_rtos_last_fifo_batch_count >= APP_DISPLAY_SKIP_THRESHOLD) ||
                     (app_measurement_should_skip_display(app_rtos_state) != 0U));

      /* 强制刷新：连续跳过超过 APP_DISPLAY_FORCE_REFRESH_MS，忽略所有门控 */
      force_refresh = 0U;
      if (should_skip != 0U)
      {
        if (last_skip_tick == 0U)
        {
          last_skip_tick = now;
        }
        else if ((now - last_skip_tick) >= APP_DISPLAY_FORCE_REFRESH_MS)
        {
          force_refresh = 1U;
          app_rtos_state->ui_forced_count++;
        }
      }

      if ((should_skip == 0U) || (force_refresh != 0U))
      {
        if (app_rtos_i2c_acquire(0U) != 0U)
        {
          app_rtos_refresh_display_if_needed(app_rtos_state);
          app_rtos_i2c_release();
          last_skip_tick = 0U;
        }
        /* 拿不到 I2C mutex 就下轮再试，不清 display_refresh_requested */
      }
      else
      {
        app_rtos_state->display_skipped_count++;
        app_rtos_state->last_ui_skip_tick = now;
      }
    }
    else
    {
      last_skip_tick = 0U;
    }

    app_rtos_send_report_if_due(app_rtos_state);
    osDelay(APP_RTOS_UI_PERIOD_MS);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the SDtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    if (app_rtos_state == NULL)
    {
      osDelay(10);
      continue;
    }

    APP_DataLog_SetMeasurementActive(
        (app_rtos_state->finger_present != 0U) &&
        (app_rtos_state->contact_settle_samples == 0U) &&
        (app_rtos_state->sensor_health == (uint8_t)SENSOR_HEALTH_OK));

    if (app_rtos_sd_service_safe(app_rtos_state) != 0U)
    {
      if (APP_DataLog_IsFlushPending() != 0U)
      {
        (void)APP_DataLog_ServiceDeferredStop();
      }
      else
      {
        (void)APP_DataLog_ServiceBudget(10U, 512U);
      }
    }

    app_rtos_update_sd_log_status(app_rtos_state);
    osDelay(APP_RTOS_SD_PERIOD_MS);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
