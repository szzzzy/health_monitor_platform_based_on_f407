/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS 应用代码
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
#include "app_runtime.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_ecg.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "iwdg.h"
#include "max30102.h"
#include "app_diag.h"
#include "tim.h"
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
/* definition of defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* definition of MAXtask */
osThreadId_t MAXtaskHandle;
const osThreadAttr_t MAXtask_attributes = {
  .name = "MAXtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* definition of watchdogtask */
osThreadId_t watchdogtaskHandle;
const osThreadAttr_t watchdogtask_attributes = {
  .name = "watchdogtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* definition of Uitask */
osThreadId_t UitaskHandle;
const osThreadAttr_t Uitask_attributes = {
  .name = "Uitask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* definition of SDtask */
osThreadId_t SDtaskHandle;
const osThreadAttr_t SDtask_attributes = {
  .name = "SDtask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* definition of i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
  .name = "i2c1Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t app_rtos_i2c_acquire(uint32_t timeout_ms);
static void app_rtos_i2c_release(void);
static void app_rtos_service_max(AppState_t *app);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* 钩子函数原型 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/**
 ******************************************************************************
 * @brief  绑定应用程序状态指针以供 RTOS 任务访问。
 * @param  state 指向全局 AppState 结构体的指针
 * @return 无。
 * @note  必须在启动时、任何任务运行之前调用一次。
 ******************************************************************************
 */
void app_rtos_bind_state(AppState_t *state)
{
  app_rtos_state = state;
}

/**
 ******************************************************************************
 * @brief  获取已绑定的应用程序状态指针。
 * @param  无。
 * @return 指向 AppState 结构体的指针，若尚未绑定则返回 NULL。
 * @note  所有任务通过此函数访问共享的应用程序状态。
 ******************************************************************************
 */
AppState_t *app_rtos_get_state(void)
{
  return app_rtos_state;
}

/**
 ******************************************************************************
 * @brief  从中断上下文通知 MAX30102 处理任务。
 * @param  无。
 * @return 无。
 * @note  使用 vTaskNotifyGiveFromISR；若尚未初始化则为空操作。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  带超时地获取 I2C1 总线互斥锁。
 * @param  timeout_ms 最大等待时间（毫秒）
 * @return 成功返回 1，超时或句柄为 NULL 返回 0。
 * @note  序列化 MAX 任务与 UI 任务之间的 I2C 访问。
 ******************************************************************************
 */
static uint8_t app_rtos_i2c_acquire(uint32_t timeout_ms)
{
  if (i2c1MutexHandle == NULL)
  {
    return 1U;
  }

  return (osMutexAcquire(i2c1MutexHandle, timeout_ms) == osOK) ? 1U : 0U;
}

/**
 ******************************************************************************
 * @brief  释放 I2C1 总线互斥锁。
 * @param  无。
 * @return 无。
 * @note  句柄为 NULL 时调用安全（空操作）。
 ******************************************************************************
 */
static void app_rtos_i2c_release(void)
{
  if (i2c1MutexHandle != NULL)
  {
    (void)osMutexRelease(i2c1MutexHandle);
  }
}

/**
 ******************************************************************************
 * @brief  运行 MAX30102 服务周期（心电图、FIFO 排空、看门狗）。
 * @param  app 指向应用程序状态的指针
 * @return 无。
 * @note  从 MAX 任务调用；内部获取 I2C 互斥锁。
 *        更新阶段跟踪变量以便诊断可见。
 ******************************************************************************
 */
static void app_rtos_service_max(AppState_t *app)
{
  uint8_t fifo_batch_count = 0U;

  if (app == NULL)
  {
    return;
  }

  app->max_task_phase = PHASE_MAX_ECG;
  (void)app_ecg_process_samples(app);

  app->max_task_phase = PHASE_MAX_FIFO_CHECK;
  if (max30102_should_service_fifo() != 0U)
  {
    app->max_task_phase = PHASE_MAX_I2C_ACQ;
    if (app_rtos_i2c_acquire(APP_RTOS_MAX_I2C_TIMEOUT_MS) != 0U)
    {
      app->max_task_phase = PHASE_MAX_FIFO_DRAIN;
      fifo_batch_count = app_measurement_drain_fifo_batch(app);

      if ((fifo_batch_count == 0U) &&
          (app->sensor_last_read_status == (uint8_t)APP_MEASUREMENT_READ_ERROR))
      {
        app->max_task_phase = PHASE_MAX_RECOVERY;
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

  app->max_task_phase = PHASE_MAX_WATCHDOG;
  if (app_rtos_i2c_acquire(0U) != 0U)
  {
    app_measurement_service_sensor_watchdog(app);
    app_rtos_i2c_release();
  }
}

/**
 ******************************************************************************
 * @brief  FreeRTOS 栈溢出钩子函数。
 * @param  xTask 溢出任务的句柄（可能已损坏）
 * @param  pcTaskName 溢出任务的名称（可能已损坏）
 * @return 无。
 * @note  捕获诊断快照，然后进入无限循环，依赖 IWDG 复位系统。
 ******************************************************************************
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* 在进入未定义行为区域之前记录崩溃。
    * 栈溢出意味着任务的 TCB 可能已损坏。 */
   (void)xTask;
   (void)pcTaskName;
   APP_Diag_CaptureCrash(DIAG_CRASH_STACKOVF, 0U, 0U);
   /* 不返回 — 陷入 HardFault 或 IWDG 复位 */
   for (;;) {}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
/**
 ******************************************************************************
 * @brief  FreeRTOS malloc 失败钩子函数。
 * @param  无。
 * @return 无。
 * @note  捕获诊断快照，然后进入无限循环，依赖 IWDG 复位系统。
 ******************************************************************************
 */
void vApplicationMallocFailedHook(void)
{
   APP_Diag_CaptureCrash(DIAG_CRASH_MALLOCFAIL, 0U, 0U);
   for (;;) {}
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS 初始化
  * @param  无
  * @retval 无
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* 创建互斥锁 */
  /* 创建 i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 添加互斥锁，... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 添加信号量，... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 启动定时器，添加新定时器，... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 添加队列，... */
  /* USER CODE END RTOS_QUEUES */

  /* 创建线程 */
  /* 创建 defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* 创建 MAXtask */
  MAXtaskHandle = osThreadNew(StartTask02, NULL, &MAXtask_attributes);

  /* 创建 watchdogtask */
  watchdogtaskHandle = osThreadNew(StartTask03, NULL, &watchdogtask_attributes);

  /* 创建 Uitask */
  UitaskHandle = osThreadNew(StartTask04, NULL, &Uitask_attributes);

  /* 创建 SDtask */
  SDtaskHandle = osThreadNew(StartTask05, NULL, &SDtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 添加线程，... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 添加事件，... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  实现 defaultTask 线程的函数。
  * @param  argument: 未使用
  * @retval 无
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 无限循环 */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief 实现 MAXtask 线程的函数。
* @param argument: 未使用
* @retval 无
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  static uint8_t  first_run       = 1U;
  static uint32_t last_wake_tick  = 0U;
  uint32_t now_tick;
  uint32_t gap_ms;
  uint32_t notified;
  /* 无限循环 */
  for(;;)
  {
    if (app_rtos_state == NULL)
    {
      osDelay(10);
      continue;
    }

    /* 调度器已启动，首次执行时闭合 TIM6 100 Hz 调度闭环。
     * 必须在这里而非 MX_FREERTOS_Init() 中启动 TIM6，
     * 否则 ISR 可能在 osKernelStart() 之前触发并调用 FreeRTOS API。 */
    if (first_run != 0U)
    {
      first_run = 0U;
      app_rtos_mark_ready();
      HAL_TIM_Base_Start_IT(&htim6);
    }

    app_rtos_state->max_task_phase = PHASE_MAX_NOTIFY_WAIT;
    notified = ulTaskNotifyTake(pdTRUE,
                                pdMS_TO_TICKS(APP_RTOS_MAX_NOTIFY_TIMEOUT_MS));

    /* 调度可观测性：记录超时唤醒和最大唤醒间隔 */
    if (notified == 0U)
    {
      app_rtos_state->max_task_timeout_count++;
    }

    now_tick = xTaskGetTickCount();
    if (last_wake_tick != 0U)
    {
      gap_ms = (now_tick - last_wake_tick) * (uint32_t)portTICK_PERIOD_MS;
      if (gap_ms > app_rtos_state->max_task_gap_ms)
      {
        app_rtos_state->max_task_gap_ms = gap_ms;
      }
    }
    last_wake_tick = now_tick;

    app_rtos_service_max(app_rtos_state);
    app_rtos_state->max_task_phase = PHASE_MAX_HEARTBEAT;
    app_rtos_state->max_task_heartbeat++;
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief 实现 watchdogtask 线程的函数。
* @param argument: 未使用
* @retval 无
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* 无限循环 */
  for(;;)
  {
    static uint8_t liveness_div = 0U;

    if (app_rtos_state != NULL) {
      app_rtos_state->wdt_task_phase = PHASE_WDT_REFRESH;
    }
    APP_Watchdog_Refresh();

    /* 每 20 拍 (~1s) 保存一次活体快照到 BKP */
    liveness_div++;
    if ((app_rtos_state != NULL) && (liveness_div >= 20U))
    {
      liveness_div = 0U;
      APP_Diag_SaveLiveness(app_rtos_state);
    }

    if (app_rtos_state != NULL) {
      app_rtos_state->wdt_task_phase = PHASE_WDT_DELAY;
    }
    osDelay(APP_RTOS_WATCHDOG_PERIOD_MS);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief 实现 Uitask 线程的函数。
* @param argument: 未使用
* @retval 无
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint32_t last_display_tick = 0U;
  uint32_t last_skip_tick = 0U;

  /* 无限循环 */
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

    app_rtos_state->ui_task_phase = PHASE_UI_IDLE;
    app_rtos_state->ui_task_heartbeat++;

    app_rtos_state->ui_task_phase = PHASE_UI_POLL_UART;
    app_protocol_poll_uart_commands(app_rtos_state);

    app_rtos_state->ui_task_phase = PHASE_UI_BUTTONS;
    app_display_handle_buttons(app_rtos_state);

    now = HAL_GetTick();
    app_rtos_state->ui_task_phase = PHASE_UI_DISPLAY_CHECK;
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
        app_rtos_state->ui_task_phase = PHASE_UI_DISPLAY_REFRESH;
        if (app_rtos_i2c_acquire(0U) != 0U)
        {
          app_runtime_refresh_display_if_needed(app_rtos_state);
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

    app_rtos_state->ui_task_phase = PHASE_UI_REPORT_SEND;
    app_runtime_send_report_if_due(app_rtos_state);

    /* 每 20 拍 (~400ms) 采集一次栈水印 */
    {
      static uint8_t hwm_div = 0U;
      hwm_div++;
      if (hwm_div >= 20U)
      {
        uint16_t hwm_m, hwm_u, hwm_s, hwm_w;
        hwm_div = 0U;
        (void)APP_Diag_SampleStackHWM(&hwm_m, &hwm_u, &hwm_s, &hwm_w);
        app_rtos_state->max_task_stack_hwm = hwm_m;
        app_rtos_state->ui_task_stack_hwm  = hwm_u;
        app_rtos_state->sd_task_stack_hwm  = hwm_s;
        app_rtos_state->wdt_task_stack_hwm = hwm_w;
        /* 持久化到 BKP ：若最小剩余超过 0 则写入，否则保留上次值 */
        if ((hwm_m > 0U) || (hwm_u > 0U) || (hwm_s > 0U) || (hwm_w > 0U))
        {
          APP_Diag_SaveStackHWMBkp(&hwm_m, &hwm_u, &hwm_s, &hwm_w);
        }
      }
    }

    app_rtos_state->ui_task_phase = PHASE_UI_DELAY;
    osDelay(APP_RTOS_UI_PERIOD_MS);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief 实现 SDtask 线程的函数。
* @param argument: 未使用
* @retval 无
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* 无限循环 */
  for(;;)
  {
    if (app_rtos_state == NULL)
    {
      osDelay(10);
      continue;
    }

    app_rtos_state->sd_task_phase = PHASE_SD_SET_ACTIVE;
    APP_DataLog_SetMeasurementActive(
        (app_rtos_state->finger_present != 0U) &&
        (app_rtos_state->contact_settle_samples == 0U) &&
        (app_rtos_state->sensor_health == (uint8_t)SENSOR_HEALTH_OK));

    app_rtos_state->sd_task_phase = PHASE_SD_SAFE_CHECK;
    if (app_runtime_sd_service_safe(app_rtos_state) != 0U)
    {
      if (APP_DataLog_IsFlushPending() != 0U)
      {
        app_rtos_state->sd_task_phase = PHASE_SD_FLUSH;
        (void)APP_DataLog_ServiceDeferredStop();
      }
      else
      {
        app_rtos_state->sd_task_phase = PHASE_SD_SERVICE_BUDGET;
        (void)APP_DataLog_ServiceBudget(10U);
      }
    }

    app_rtos_state->sd_task_phase = PHASE_SD_STATUS;
    app_runtime_update_sd_log_status(app_rtos_state);
    app_rtos_state->sd_task_phase = PHASE_SD_DELAY;
    osDelay(APP_RTOS_SD_PERIOD_MS);
  }
  /* USER CODE END StartTask05 */
}

/* 私有应用程序代码 --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
