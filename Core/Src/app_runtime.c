#include "app_runtime.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_measurement.h"
#include "app_protocol.h"
#include "main.h"

#define APP_SD_SAFE_SAMPLE_AGE_MS  25U

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
