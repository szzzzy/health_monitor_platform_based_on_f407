#include "app_ppg_pulse.h"

#include <string.h>

#include "app_display.h"
#include "app_hrv.h"
#include "app_rr.h"

#define APP_STREAM_PULSE_MIN_RMS       4U
#define APP_STREAM_PULSE_THRESHOLD_DIV 4U
#define APP_STREAM_PULSE_PROM_DIV      3U

static struct
{
  int32_t previous2;
  int32_t previous1;
  uint8_t sample_count;
  uint8_t last_peak_valid;
  int8_t last_polarity;
  uint32_t last_peak_sample;
} stream_pulse_state;

void app_ppg_pulse_reset(void)
{
  (void)memset(&stream_pulse_state, 0, sizeof(stream_pulse_state));
}

uint8_t app_ppg_pulse_update(AppState_t *app,
                             int32_t filtered_sample,
                             uint32_t total_samples,
                             MAX30102_PulseInfo_t *pulse_info)
{
  const uint32_t min_interval_samples =
      (((uint32_t)APP_HRV_IBI_MIN_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;
  const uint32_t max_interval_samples =
      (((uint32_t)APP_HRV_IBI_MAX_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;
  uint32_t current_sample_number;
  uint32_t candidate_sample_number;
  uint32_t interval_samples;
  uint32_t threshold;
  uint32_t prominence_threshold;
  uint32_t positive_prominence = 0U;
  uint32_t negative_prominence = 0U;
  uint32_t magnitude = 0U;
  int32_t neighbor_reference;
  int8_t polarity = 0;
  uint8_t pulse_detected = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((app == NULL) || (pulse_info == NULL) || (app->finger_present == 0U))
  {
    app_ppg_pulse_reset();
    return 0U;
  }

  if (stream_pulse_state.sample_count < 2U)
  {
    if (stream_pulse_state.sample_count == 0U)
    {
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 1U;
    }
    else
    {
      stream_pulse_state.previous2 = stream_pulse_state.previous1;
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 2U;
    }
    return 0U;
  }

  threshold = app->signal_ir_ac_rms / APP_STREAM_PULSE_THRESHOLD_DIV;
  if (threshold < APP_STREAM_PULSE_MIN_RMS)
  {
    threshold = APP_STREAM_PULSE_MIN_RMS;
  }

  prominence_threshold = threshold / APP_STREAM_PULSE_PROM_DIV;
  if (prominence_threshold < 2U)
  {
    prominence_threshold = 2U;
  }

  if ((stream_pulse_state.previous1 > stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 >= filtered_sample) &&
      (stream_pulse_state.previous1 > (int32_t)threshold))
  {
    neighbor_reference = (stream_pulse_state.previous2 > filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (stream_pulse_state.previous1 > neighbor_reference)
    {
      positive_prominence = (uint32_t)(stream_pulse_state.previous1 - neighbor_reference);
    }
  }

  if ((stream_pulse_state.previous1 < stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 <= filtered_sample) &&
      (stream_pulse_state.previous1 < -((int32_t)threshold)))
  {
    neighbor_reference = (stream_pulse_state.previous2 < filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (neighbor_reference > stream_pulse_state.previous1)
    {
      negative_prominence = (uint32_t)(neighbor_reference - stream_pulse_state.previous1);
    }
  }

  if ((positive_prominence >= prominence_threshold) ||
      (negative_prominence >= prominence_threshold))
  {
    if (positive_prominence >= negative_prominence)
    {
      polarity = 1;
      magnitude = (uint32_t)stream_pulse_state.previous1;
    }
    else
    {
      polarity = -1;
      magnitude = (uint32_t)(-stream_pulse_state.previous1);
    }
  }

  if ((polarity != 0) && (total_samples >= 2U))
  {
    current_sample_number = total_samples - 1U;
    candidate_sample_number = current_sample_number - 1U;

    if (stream_pulse_state.last_peak_valid == 0U)
    {
      stream_pulse_state.last_peak_valid = 1U;
      stream_pulse_state.last_polarity = polarity;
      stream_pulse_state.last_peak_sample = candidate_sample_number;
    }
    else if (polarity != stream_pulse_state.last_polarity)
    {
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        stream_pulse_state.last_polarity = polarity;
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
    }
    else
    {
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
      else if (interval_samples >= min_interval_samples)
      {
        stream_pulse_state.last_peak_sample = candidate_sample_number;
        pulse_info->beat_valid = 1U;
        pulse_info->interval_samples = (uint16_t)interval_samples;
        pulse_info->latest_ibi_ms = (uint16_t)((interval_samples * 1000U +
                                                (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                               MAX30102_ALGO_SAMPLE_RATE_HZ);
        pulse_info->latest_peak_sample = candidate_sample_number;
        pulse_info->beat_amplitude = magnitude;
        pulse_detected = 1U;
      }
    }
  }

  stream_pulse_state.previous2 = stream_pulse_state.previous1;
  stream_pulse_state.previous1 = filtered_sample;
  return pulse_detected;
}

void app_ppg_pulse_process_metrics(AppState_t *app,
                                   const MAX30102_PulseInfo_t *pulse_info,
                                   uint32_t total_samples)
{
  if ((app == NULL) || (pulse_info == NULL) || (pulse_info->beat_valid == 0U))
  {
    return;
  }

  if (app_hrv_mark_peak_seen(pulse_info->latest_peak_sample) == 0U)
  {
    return;
  }

  if (app_hrv_add_ibi(app, pulse_info->latest_ibi_ms) == 0U)
  {
    return;
  }

  app_display_add_ir_pulse_marker();
  (void)total_samples;

  if ((app->signal_quality >= APP_RR_SIGNAL_QUALITY_MIN) && (pulse_info->beat_amplitude != 0U))
  {
    app_rr_add_beat(pulse_info->latest_peak_sample, pulse_info->beat_amplitude);
    app_rr_update_output(app);
  }
  else if (app->signal_quality < APP_RR_SIGNAL_QUALITY_MIN)
  {
    app->rr_valid = 0U;
  }
}
