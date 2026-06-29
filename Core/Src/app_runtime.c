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

static uint32_t app_runtime_saturating_add_u32(uint32_t a, uint32_t b)
{
  if ((0xFFFFFFFFUL - a) < b)
  {
    return 0xFFFFFFFFUL;
  }

  return a + b;
}

void app_runtime_send_report_if_due(AppState_t *app)
{
  if ((app == NULL) || (app->report_due == 0U))
  {
    return;
  }

  app_protocol_send_sensor_report(app);
  app->report_due = 0U;
}

void app_runtime_refresh_display_if_needed(AppState_t *app)
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

void app_runtime_update_sd_log_status(AppState_t *app)
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
  app->sd_total_written = st.total_written;
  app->flush_pending = APP_DataLog_IsFlushPending();
}

void app_runtime_service_eeprom_stats(const AppState_t *app)
{
  static uint8_t initialized = 0U;
  static uint32_t base_tick;
  static uint32_t last_sync_tick;
  static uint32_t base_run_hours_x10;
  static uint32_t base_sensor_read_error_count;
  static uint32_t base_sensor_recovery_count;
  uint32_t now;
  uint32_t elapsed_run_x10;

  if ((app == NULL) || (!g_eeprom.initialized))
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
    return;
  }

  if ((now - last_sync_tick) < APP_EEPROM_RUNTIME_SYNC_MS)
  {
    return;
  }

  elapsed_run_x10 = (now - base_tick) / APP_EEPROM_RUN_X10_MS;
  eeprom_store_update_runtime(
      app_runtime_saturating_add_u32(base_run_hours_x10, elapsed_run_x10),
      app_runtime_saturating_add_u32(base_sensor_read_error_count, app->sensor_read_error_count),
      app_runtime_saturating_add_u32(base_sensor_recovery_count, app->sensor_recover_count));
  last_sync_tick = now;
}

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
