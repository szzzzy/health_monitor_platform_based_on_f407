#include "app_measurement.h"

#include <string.h>

#include "app_bpm_filter.h"
#include "app_display.h"
#include "app_hrv.h"
#include "app_motion.h"
#include "app_oxy_status.h"
#include "app_ppg_pulse.h"
#include "app_ppg_signal.h"
#include "app_rr.h"
#include "app_spo2_filter.h"
#include "i2c.h"
#include "max30102.h"

#define APP_SENSOR_RECOVERY_ERROR_COUNT   8U
#define APP_BPM_EVALUATE_INTERVAL_SAMPLES 3U
#define APP_BPM_ACORR_BLEND_MAX_DIFF      12U
#define APP_SIGNAL_QUALITY_MIN_FOR_SPO2   30U
#define APP_SIGNAL_QUALITY_MIN_FOR_BPM    25U
#define APP_ADVANCED_PULSE_STALE_SAMPLES  (MAX30102_ALGO_SAMPLE_RATE_HZ * 3U)

static uint8_t fifo_buf[6];
static MAX30102_Baseline_t baseline_data;
static MAX30102_SpO2_t spo2_state;
static struct
{
  uint32_t red_value;
  uint32_t ir_value;
  uint8_t initialized;
} sample_debug_state;
static uint8_t bpm_update_decimator;

static void app_reset_measurement_outputs(AppState_t *app);
static void app_invalidate_advanced_outputs(AppState_t *app);
static void app_reset_advanced_metrics(AppState_t *app);

void app_measurement_init_state(AppState_t *app)
{
  app_ppg_signal_init_state(app);
}

void app_measurement_reset_runtime(void)
{
  max30102_baseline_reset(&baseline_data);
  max30102_spo2_reset(&spo2_state);
  (void)memset(&sample_debug_state, 0, sizeof(sample_debug_state));
  bpm_update_decimator = 0U;
  app_spo2_filter_reset(NULL);
  app_motion_reset(NULL);
  app_ppg_signal_reset_envelope();
  app_reset_advanced_metrics(NULL);
  app_display_reset_waveforms();
}

uint8_t app_measurement_collect_baseline_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;

  if (app == NULL)
  {
    return 0U;
  }

  read_status = max30102_read_fifo(fifo_buf, 6U);
  if (read_status != HAL_OK)
  {
    if (read_status == HAL_BUSY)
    {
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
      app->sensor_read_busy_count++;
      app->sensor_error_streak = 0U;
      return 0U;
    }

    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
    app->sensor_read_error_count++;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    if (app->sensor_error_streak < 0xFFU)
    {
      app->sensor_error_streak++;
    }
    app_measurement_recover_sensor(app);
    return 0U;
  }

  max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
  max30102_baseline_add_ir(&baseline_data, app->ir_value);
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
  app->sensor_read_ok_count++;
  app->sensor_error_streak = 0U;
  app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
  app->sensor_last_sample_tick = HAL_GetTick();
  return 1U;
}

uint8_t app_measurement_baseline_ready(void)
{
  return max30102_baseline_is_ready(&baseline_data, APP_MEASUREMENT_BASELINE_SAMPLES);
}

uint16_t app_measurement_get_baseline_progress_percent(void)
{
  if (baseline_data.sample_count >= APP_MEASUREMENT_BASELINE_SAMPLES)
  {
    return 100U;
  }

  return (uint16_t)((baseline_data.sample_count * 100U) / APP_MEASUREMENT_BASELINE_SAMPLES);
}

uint32_t app_measurement_get_baseline_average(void)
{
  return max30102_baseline_get_average_ir(&baseline_data);
}

uint32_t app_measurement_get_baseline_range(void)
{
  return max30102_baseline_get_range_ir(&baseline_data);
}

uint32_t app_measurement_get_tracked_baseline(void)
{
  return max30102_baseline_get_tracked_ir(&baseline_data);
}

void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir)
{
  max30102_baseline_seed_tracking(&baseline_data, baseline_ir, noise_ir);
}

uint8_t app_measurement_baseline_is_stable(void)
{
  return max30102_baseline_is_stable(&baseline_data, APP_MEASUREMENT_BASELINE_STABLE_RANGE);
}

AppMeasurementReadStatus_t app_measurement_read_sensor_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;
  const MAX30102_FifoDebug_t *fifo_debug;

  if (app == NULL)
  {
    return APP_MEASUREMENT_READ_ERROR;
  }

  read_status = max30102_read_fifo(fifo_buf, 6U);
  fifo_debug = max30102_get_fifo_debug();
  if (fifo_debug != NULL)
  {
    app->sensor_fifo_overflow_count = fifo_debug->overflow_count;
    app->sensor_fifo_write_ptr = fifo_debug->write_ptr;
    app->sensor_fifo_read_ptr = fifo_debug->read_ptr;
    app->sensor_fifo_available_samples = fifo_debug->available_samples;
  }

  if (read_status == HAL_OK)
  {
    max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
    if (sample_debug_state.initialized != 0U)
    {
      if ((sample_debug_state.red_value != app->red_value) ||
          (sample_debug_state.ir_value != app->ir_value))
      {
        app->sensor_sample_change_count++;
      }
      else
      {
        app->sensor_sample_same_count++;
      }
    }
    else
    {
      app->sensor_sample_change_count++;
      sample_debug_state.initialized = 1U;
    }

    sample_debug_state.red_value = app->red_value;
    sample_debug_state.ir_value = app->ir_value;
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_ppg_signal_update_activity(app);
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
    app->sensor_read_ok_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    app->sensor_last_sample_tick = HAL_GetTick();
    return APP_MEASUREMENT_READ_OK;
  }

  if (read_status == HAL_BUSY)
  {
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
    app->sensor_read_busy_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    return APP_MEASUREMENT_READ_WAIT;
  }

  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
  app->sensor_read_error_count++;
  app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
  if (app->sensor_error_streak < 0xFFU)
  {
    app->sensor_error_streak++;
  }

  return APP_MEASUREMENT_READ_ERROR;
}

void app_measurement_update_adaptive_thresholds(AppState_t *app)
{
  app_ppg_signal_update_adaptive_thresholds(app, &baseline_data);
}

void app_measurement_update_finger_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->raw_signal_present = app_ppg_signal_is_raw_present(app);

  if (app->finger_present == 0U)
  {
    app->finger_off_confirm_count = 0U;

    if (app->raw_signal_present == 0U)
    {
      app->finger_on_confirm_count = 0U;
      app_ppg_signal_track_background_ir(app, &baseline_data);
      return;
    }

    if (app->finger_on_confirm_count < 0xFFU)
    {
      app->finger_on_confirm_count++;
    }

    if (app->finger_on_confirm_count >= APP_PPG_SIGNAL_FINGER_ON_CONFIRM_COUNT)
    {
      app->finger_present = 1U;
      app->finger_on_confirm_count = 0U;
      app_reset_measurement_outputs(app);
      app_ppg_signal_reset_envelope();
      app->raw_signal_present = 1U;
      app->report_due = 1U;
      app->display_refresh_requested = 1U;
    }

    return;
  }

  app->finger_on_confirm_count = 0U;

  if (app->raw_signal_present != 0U)
  {
    app->finger_off_confirm_count = 0U;
    return;
  }

  if (app->finger_off_confirm_count < 0xFFU)
  {
    app->finger_off_confirm_count++;
  }

  if (app->finger_off_confirm_count >= APP_PPG_SIGNAL_FINGER_OFF_CONFIRM_COUNT)
  {
    app->finger_present = 0U;
    app->finger_off_confirm_count = 0U;
    app->finger_on_confirm_count = 0U;
    app_ppg_signal_track_background_ir(app, &baseline_data);
    max30102_baseline_seed_tracking(&baseline_data,
                                    app->baseline_ir,
                                    APP_PPG_SIGNAL_REACQUIRE_NOISE_IR);
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_reset_measurement_outputs(app);
    app_ppg_signal_reset_envelope();
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }
}

void app_measurement_process(AppState_t *app)
{
  MAX30102_SignalMetrics_t signal_metrics;
  MAX30102_PulseInfo_t pulse_info;
  uint8_t raw_bpm_valid = 0U;
  uint8_t raw_bpm_value = 0U;
  uint8_t acorr_bpm_valid = 0U;
  uint8_t acorr_bpm = 0U;
  uint8_t raw_spo2_valid = 0U;
  uint8_t raw_spo2_value = 0U;
  uint8_t signal_quality = 0U;
  int32_t red_waveform_sample = 0;
  int32_t ir_waveform_sample = 0;

  if ((app == NULL) || (app->finger_present == 0U))
  {
    return;
  }

  max30102_spo2_add_sample(&spo2_state, app->red_value, app->ir_value);
  if (max30102_spo2_get_latest_filtered(&spo2_state,
                                         &red_waveform_sample,
                                         &ir_waveform_sample) != 0U)
  {
    app_display_add_ir_sample(ir_waveform_sample);
    app_display_add_red_sample(red_waveform_sample);
  }

  if (max30102_get_signal_metrics(&spo2_state, &signal_metrics) != 0U)
  {
    if (max30102_calculate_signal_quality(&spo2_state, &signal_metrics, &signal_quality) != 0U)
    {
      app_oxy_status_update_from_metrics(app, &signal_metrics, signal_quality);
    }
    else
    {
      app_oxy_status_update_from_metrics(app, &signal_metrics, 0U);
    }

    app_motion_update_artifact(app, &signal_metrics, signal_quality);
  }
  else
  {
    app_oxy_status_clear_instant(app);
    app_motion_update_artifact(app, NULL, 0U);
    app_invalidate_advanced_outputs(app);
  }

  if (app->motion_artifact != 0U)
  {
    app->bpm_valid = 0U;
    app_spo2_filter_update_output(app, 0U, 0U);
    app_invalidate_advanced_outputs(app);
    app_ppg_pulse_reset();
    return;
  }

  if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    app->rr_valid = 0U;
  }
  else if (app->signal_quality < APP_RR_SIGNAL_QUALITY_MIN)
  {
    app->rr_valid = 0U;
  }

  if (app_hrv_is_peak_stale(spo2_state.total_samples, APP_ADVANCED_PULSE_STALE_SAMPLES) != 0U)
  {
    app_invalidate_advanced_outputs(app);
  }

  if (app_ppg_pulse_update(app, ir_waveform_sample, spo2_state.total_samples, &pulse_info) != 0U)
  {
    app_ppg_pulse_process_metrics(app, &pulse_info, spo2_state.total_samples);
  }

  raw_spo2_valid = max30102_calculate_spo2(&spo2_state, &raw_spo2_value);
  if ((raw_spo2_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2))
  {
    raw_spo2_valid = 0U;
  }
  app_spo2_filter_update_output(app, raw_spo2_valid, raw_spo2_value);

  bpm_update_decimator++;
  if (bpm_update_decimator < APP_BPM_EVALUATE_INTERVAL_SAMPLES)
  {
    return;
  }

  bpm_update_decimator = 0U;
  raw_bpm_valid = max30102_calculate_bpm_with_pulse(&spo2_state, &raw_bpm_value, &pulse_info);

  if (app->signal_quality >= APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    acorr_bpm_valid = max30102_autocorr_bpm(&spo2_state, &acorr_bpm);
  }

  if ((raw_bpm_valid != 0U) && (acorr_bpm_valid != 0U))
  {
    uint8_t diff = (raw_bpm_value > acorr_bpm) ? (raw_bpm_value - acorr_bpm)
                                               : (acorr_bpm - raw_bpm_value);
    if (diff <= APP_BPM_ACORR_BLEND_MAX_DIFF)
    {
      raw_bpm_value = (uint8_t)(((uint16_t)raw_bpm_value + (uint16_t)acorr_bpm * 3U + 2U) / 4U);
    }
  }
  else if (acorr_bpm_valid != 0U)
  {
    raw_bpm_valid = 1U;
    raw_bpm_value = acorr_bpm;
  }

  if ((raw_bpm_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM))
  {
    raw_bpm_valid = 0U;
    pulse_info.beat_valid = 0U;
  }

  if (pulse_info.beat_valid != 0U)
  {
    app_ppg_pulse_process_metrics(app, &pulse_info, spo2_state.total_samples);
  }
  else if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    app->rr_valid = 0U;
  }

  if (app_bpm_filter_update(app, raw_bpm_valid, raw_bpm_value) != 0U)
  {
    bpm_update_decimator = 0U;
  }
}

void app_measurement_update_periodic_flags(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->refresh_div++;
  if (app->refresh_div < 20U)
  {
    return;
  }

  app->refresh_div = 0U;
  app->report_due = 1U;
  app->display_refresh_requested = 1U;
}

void app_measurement_recover_sensor(AppState_t *app)
{
  if ((app == NULL) || (app->sensor_error_streak < APP_SENSOR_RECOVERY_ERROR_COUNT))
  {
    return;
  }

  if (MX_I2C1_RecoverBus() != HAL_OK)
  {
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    return;
  }

  if (max30102_init() != HAL_OK)
  {
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    return;
  }

  app->sensor_error_streak = 0U;
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
  app->sensor_recover_count++;
  app_ppg_signal_reset_envelope();
  app_reset_measurement_outputs(app);
  max30102_baseline_seed_tracking(&baseline_data,
                                  app->baseline_ir,
                                  APP_PPG_SIGNAL_REACQUIRE_NOISE_IR);
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
}

static void app_reset_measurement_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_display_reset_waveforms();
  max30102_spo2_reset(&spo2_state);
  app_bpm_filter_reset(app);
  app_spo2_filter_reset(app);
  app_motion_reset(app);
  app->raw_signal_present = 0U;
  app->ir_signal_delta = 0U;
  app->ir_signal_span = 0U;
  app->red_signal_span = 0U;
  app_oxy_status_reset(app);
  app_reset_advanced_metrics(app);
}

static void app_invalidate_advanced_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_hrv_invalidate_outputs(app);
  app->rr_valid = 0U;
}

static void app_reset_advanced_metrics(AppState_t *app)
{
  app_hrv_reset(app);
  app_rr_reset(app);
  app_ppg_pulse_reset();
}
