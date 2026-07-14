/**
  ******************************************************************************
  * @file    app_runtime.c
  * @brief   应用运行时胶水层：上报、显示刷新、SD 状态同步与 EEPROM 统计。
  *
  * 本模块不实现测量算法，也不直接访问 MAX30102/ECG 采样链路。它只在
  * RTOS 任务或 boot 阶段调用公共服务函数，避免 main.c / freertos.c
  * 各自维护重复逻辑。
  ******************************************************************************
  */

#include "app_runtime.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "eeprom_store.h"
#include "main.h"

#define APP_SD_SAFE_SAMPLE_AGE_MS  25U
#define APP_EEPROM_RUNTIME_SYNC_MS 3600000UL
#define APP_EEPROM_RUN_X10_MS      360000UL

/* ---- uint32 饱和加法：EEPROM 累计计数防回绕 ---- */
static uint32_t app_runtime_saturating_add_u32(uint32_t a, uint32_t b)
{
  if ((0xFFFFFFFFUL - a) < b)
  {
    return 0xFFFFFFFFUL;
  }

  return a + b;
}

static uint8_t app_runtime_claim_event(uint8_t *event_flag)
{
  uint32_t primask;
  uint8_t claimed;

  if (event_flag == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  claimed = (*event_flag != 0U) ? 1U : 0U;
  *event_flag = 0U;
  if ((primask & 1UL) == 0UL)
  {
    __enable_irq();
  }

  return claimed;
}

static uint8_t app_runtime_claim_display_event(AppState_t *app)
{
  uint32_t primask;
  uint8_t claimed;

  if (app == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  claimed = (app->display_refresh_requested != 0U) ? 1U : 0U;
  app->display_refresh_requested = 0U;
  if (claimed != 0U)
  {
    app->fifo_high_watermark = 0U;
  }
  if ((primask & 1UL) == 0UL)
  {
    __enable_irq();
  }

  return claimed;
}

/**
 *******************************************************************************
 * @brief  若 report_due 置位，则发送一帧 USART M 测量报文。
 * @param  app AppState 指针。
 * @note   发送成功与否由协议层更新 uart_tx_message_valid；本函数只清节拍标志。
 *******************************************************************************
 */
void app_runtime_send_report_if_due(AppState_t *app)
{
  if ((app == NULL) ||
      (app_runtime_claim_event(&app->report_due) == 0U))
  {
    return;
  }

  app_protocol_send_sensor_report(app);
  if (app->uart_tx_message_valid == false)
  {
    app->report_due = 1U;
  }
}

/**
 *******************************************************************************
 * @brief  若 display_refresh_requested 置位，则刷新 OLED 测量页面。
 * @param  app AppState 指针。
 * @note   刷新前读取 RTC 并复制 AppState 快照，OLED 绘制只读副本，
 *         避免跨任务半更新字段影响显示。
 *******************************************************************************
 */
void app_runtime_refresh_display_if_needed(AppState_t *app)
{
  AppState_t snapshot;

  if ((app == NULL) ||
      (app_runtime_claim_display_event(app) == 0U))
  {
    return;
  }

  app_protocol_update_rtc_snapshot(app);
  if (app_state_take_snapshot(app, &snapshot) == 0U)
  {
    app->display_refresh_requested = 1U;
    return;
  }

  app_display_measurement_page(&snapshot);
  app->display_refresh_count++;
  app->display_last_refresh_tick = HAL_GetTick();
}

/**
 *******************************************************************************
 * @brief  将 SD 日志引擎状态同步到 AppState。
 * @param  app AppState 指针。
 * @note   OLED 和 USART 只读 AppState，不直接访问日志模块内部状态。
 *******************************************************************************
 */
void app_runtime_update_sd_log_status(AppState_t *app)
{
  DataLogStatus_t st;

  if (app == NULL) { return; }

  APP_DataLog_GetStatus(&st);
  app->sd_log_active = APP_DataLog_IsActive() ? 1U : 0U;
  app->sd_buffered  = st.buffered;
  app->sd_dropped   = st.dropped;
  app->sd_written   = st.written;
  app->sd_paused    = st.paused;
  app->sd_error     = st.sd_error;
  app->sd_state     = st.state;
  app->sd_last_write_ms = st.last_write_ms;
  app->sd_total_written = st.total_written;
  app->sd_unsynced = st.unsynced;
  app->sd_sync_error_count = st.sync_error_count;
  app->sd_retention_ms = st.retention_ms;
  app->sd_rolling_mode = st.rolling_mode;
  app->flush_pending = APP_DataLog_IsFlushPending();
}

/**
 *******************************************************************************
 * @brief  周期性把运行时累计统计同步到 EEPROM。
 * @param  app AppState 指针。
 * @note   约每小时写一次，使用启动基线 + elapsed 方式减少 EEPROM 写入频率。
 *******************************************************************************
 */
void app_runtime_service_eeprom_stats(const AppState_t *app)
{
  static uint8_t initialized = 0U;
  static uint32_t base_tick;
  static uint32_t last_sync_tick;
  static uint32_t base_run_hours_x10;
  static uint32_t runtime_remainder_ms;
  static uint32_t base_sensor_read_error_count;
  static uint32_t base_sensor_recovery_count;
  uint32_t now;
  uint32_t elapsed_run_x10;
  uint32_t elapsed_ms;

  if (app == NULL)
  {
    return;
  }

  eeprom_store_set_measurement_active(app->finger_present != 0U);
  if (app->finger_present == 0U)
  {
    (void)eeprom_store_service_deferred();
  }
  if (!g_eeprom.initialized)
  {
    return;
  }

  now = HAL_GetTick();
  if (initialized == 0U)
  {
    initialized = 1U;
    base_tick = now;
    last_sync_tick = now;
    base_run_hours_x10 = g_eeprom.total_run_hours_x10;
    base_sensor_read_error_count = g_eeprom.sensor_read_error_count;
    base_sensor_recovery_count = g_eeprom.sensor_recovery_count;
    runtime_remainder_ms = 0U;
    return;
  }

  elapsed_ms = now - base_tick;
  base_tick = now;
  runtime_remainder_ms += elapsed_ms;
  elapsed_run_x10 = runtime_remainder_ms / APP_EEPROM_RUN_X10_MS;
  runtime_remainder_ms %= APP_EEPROM_RUN_X10_MS;
  if (elapsed_run_x10 != 0U)
  {
    base_run_hours_x10 = app_runtime_saturating_add_u32(base_run_hours_x10,
                                                         elapsed_run_x10);
  }

  if ((now - last_sync_tick) < APP_EEPROM_RUNTIME_SYNC_MS)
  {
    return;
  }

  if (eeprom_store_update_runtime(
      base_run_hours_x10,
      app_runtime_saturating_add_u32(base_sensor_read_error_count, app->sensor_read_error_count),
      app_runtime_saturating_add_u32(base_sensor_recovery_count, app->sensor_recover_count)))
  {
    last_sync_tick = now;
  }
}

/**
 *******************************************************************************
 * @brief  判断当前是否适合让 SD 后台任务执行写入服务。
 * @param  app AppState 指针。
 * @return 1 = 安全窗口，可服务 SD；0 = 应优先保证 MAX30102/显示链路。
 * @note   本函数只门控普通后台写入：要求无手指、传感器健康、FIFO 不积压且
 *         最近样本足够新。停止排空使用 app_runtime_sd_flush_safe()，避免在
 *         停止产样后被“最近样本”条件永久卡住。
 *******************************************************************************
 */
uint8_t app_runtime_sd_service_safe(const AppState_t *app)
{
  uint32_t now;

  if (app == NULL) { return 0U; }
  if (app->finger_present != 0U) { return 0U; }
  if (app->sensor_health != (uint8_t)SENSOR_HEALTH_OK) { return 0U; }
  if (app->contact_settle_samples > 0U) { return 0U; }
  if (app->raw_signal_present != 0U) { return 0U; }
  if (app->finger_on_confirm_count > 0U) { return 0U; }
  if (app->ir_signal_delta >= (app->adaptive_finger_on_delta / 2U)) { return 0U; }
  if (app->sensor_fifo_available_samples >= 4U) { return 0U; }
  if (app->sensor_last_sample_tick == 0UL) { return 0U; }

  now = HAL_GetTick();
  if ((now - app->sensor_last_sample_tick) >= APP_SD_SAFE_SAMPLE_AGE_MS)
  {
    return 0U;
  }

  return 1U;
}

/**
 *******************************************************************************
 * @brief  判断停止测量后的 SD 延迟排空是否可推进。
 * @param  app AppState 指针。
 * @return 1 = 无已确认手指，可推进停止排空；0 = 手指在位或参数无效。
 * @note   与普通后台写入不同，停止排空不能依赖传感器健康状态或最近样本时间；
 *         停止产样或 I2C 异常时这些条件不会自行恢复。已确认手指始终优先，
 *         此时禁止可能阻塞的 f_write/f_sync/f_close。
 *******************************************************************************
 */
uint8_t app_runtime_sd_flush_safe(const AppState_t *app)
{
  if (app == NULL) { return 0U; }
  return (app->finger_present == 0U) ? 1U : 0U;
}
