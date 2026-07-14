/**
  ******************************************************************************
  * @file    max30102_bpm.c
  * @brief   MAX30102 BPM 峰值检测 + FFT 自相关心率估算
  ******************************************************************************
  */

#include "max30102_bpm.h"
#include "max30102_algo_utils.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 * BPM / SpO2 算法参数
 *
 * 所有参数按 100 Hz 采样率设计。若修改 MAX30102_DEFAULT_SPO2_CONFIG
 * 中的采样率，以下阈值需要同步缩放。
 * ========================================================================= */
#define MAX30102_BPM_MIN_RESULT            35U
#define MAX30102_BPM_MAX_RESULT            220U
#define MAX30102_BPM_MIN_AMPLITUDE         18U
#define MAX30102_BPM_MIN_AC_RMS            3U
#define MAX30102_BPM_THRESHOLD_NUM         3U
#define MAX30102_BPM_THRESHOLD_DEN         100U
#define MAX30102_BPM_MIN_THRESHOLD         3U
#define MAX30102_BPM_PROMINENCE_NUM        2U
#define MAX30102_BPM_PROMINENCE_DEN        100U
#define MAX30102_BPM_MIN_PROMINENCE        3U
#define MAX30102_BPM_MAX_INTERVAL_JITTER   85U
#define MAX30102_BPM_MIN_EDGE_DELTA        2U
#define MAX30102_BPM_MAX_PEAK_COUNT        24U
#define MAX30102_BPM_RANGE_CAP_RMS_FACTOR  4U
#define MAX30102_BPM_PEAK_AMP_HALF_WINDOW  10U

/* FFT 自相关参数 */
#define FFT_SIZE  256U
#define LAG_MIN    27U
#define LAG_MAX   171U
#define AUTOCORR_MIN_PEAK_RATIO 0.06f

static float32_t fft_in[FFT_SIZE];
static float32_t fft_out[FFT_SIZE];
static arm_rfft_fast_instance_f32 rfft_inst;
static uint8_t rfft_initialized = 0U;

/**
 * @brief  一次性初始化自相关使用的 RFFT 实例。
 * @note   为 FFT_SIZE（256）点初始化 arm_rfft_fast_instance_f32。
 */
static void autocorr_ensure_init(void)
{
  if (rfft_initialized == 0U)
  {
    arm_rfft_fast_init_f32(&rfft_inst, FFT_SIZE);
    rfft_initialized = 1U;
  }
}

/**
 * @brief  通过 IR 波形峰值检测估算心率。
 * @param  spo2_state SpO2 状态结构体指针。
 * @param  bpm_value  估算 BPM 的输出指针。
 * @return 成功返回 1；峰值不足或 BPM 超出范围时返回 0。
 * @note   便捷包装函数；调用 max30102_calculate_bpm_with_pulse，
 *         并将 pulse_info 设置为 NULL。
 */
uint8_t max30102_calculate_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm_value)
{
  return max30102_calculate_bpm_with_pulse(spo2_state, bpm_value, NULL);
}

/**
 * @brief  估算 BPM，并输出用于交叉校验的脉搏信息。
 * @param  spo2_state SpO2 状态结构体指针。
 * @param  bpm_value  估算 BPM 的输出指针。
 * @param  pulse_info 详细脉搏信息输出指针（可为 NULL）。
 * @return 成功返回 1；峰值不足或 BPM 超出范围时返回 0。
 * @note   使用平滑后的 3 点峰值检测，并结合显著性和边沿阈值。
 *         提供 pulse_info 时，填充 IBI、幅度和样本时序，
 *         供上游流式脉冲检测使用。
 */
uint8_t max30102_calculate_bpm_with_pulse(const MAX30102_SpO2_t *spo2_state,
                                          uint8_t *bpm_value,
                                          MAX30102_PulseInfo_t *pulse_info)
{
  uint16_t i;
  uint16_t sample_index;
  uint16_t sample_count;
  uint16_t start_index;
  uint16_t peak_positions[MAX30102_BPM_MAX_PEAK_COUNT];
  uint16_t peak_count;
  int32_t ir_min;
  int32_t ir_second_min;
  int32_t ir_max;
  int32_t ir_second_max;
  uint32_t amplitude;
  uint32_t threshold_amplitude;
  uint32_t amplitude_cap;
  uint32_t ir_ac_rms;
  int32_t threshold;
  int32_t prominence_threshold;
  int32_t edge_threshold;
  int32_t prev2;
  int32_t prev1;
  int32_t current;
  int32_t next1;
  int32_t next2;
  int32_t smoothed_prev1;
  int32_t smoothed_current;
  int32_t smoothed_next;
  int32_t left_min;
  int32_t right_min;
  int32_t prominence;
  int32_t left_edge;
  int32_t right_edge;
  uint16_t min_peak_distance;
  uint16_t max_interval_samples;
  uint32_t interval_sum;
  uint16_t interval_count;
  uint16_t min_interval;
  uint16_t max_interval;
  uint32_t average_interval;
  uint32_t allowed_jitter;
  uint32_t bpm_estimate;
  uint32_t peak_span_samples;
  uint16_t interval_samples;
  uint16_t latest_interval_samples;
  uint16_t latest_peak_position;
  uint32_t oldest_sample_number;
  int32_t filtered_value;
  uint8_t fallback_used = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((spo2_state == NULL) || (bpm_value == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_BPM_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  ir_min = INT32_MAX;
  ir_second_min = INT32_MAX;
  ir_max = INT32_MIN;
  ir_second_max = INT32_MIN;
  start_index = (sample_count < MAX30102_SPO2_WINDOW_SIZE) ? 0U : spo2_state->write_index;
  sample_index = start_index;

  for (i = 0U; i < sample_count; i++)
  {
    filtered_value = spo2_state->ir_filtered_samples[sample_index];

    if (filtered_value < ir_min)          { ir_second_min = ir_min; ir_min = filtered_value; }
    else if (filtered_value < ir_second_min) { ir_second_min = filtered_value; }

    if (filtered_value > ir_max)          { ir_second_max = ir_max; ir_max = filtered_value; }
    else if (filtered_value > ir_second_max) { ir_second_max = filtered_value; }

    sample_index++;
    if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  }

  amplitude = (uint32_t)((int64_t)ir_max - (int64_t)ir_min);

  if (amplitude < MAX30102_BPM_MIN_AMPLITUDE) { return 0U; }

  ir_ac_rms = max30102_calculate_window_rms(spo2_state->ir_filtered_square_sum, sample_count);
  if (ir_ac_rms < MAX30102_BPM_MIN_AC_RMS) { return 0U; }

  if (ir_second_min == INT32_MAX) { ir_second_min = ir_min; }
  if (ir_second_max == INT32_MIN) { ir_second_max = ir_max; }

  threshold_amplitude = (uint32_t)((int64_t)ir_second_max - (int64_t)ir_second_min);
  amplitude_cap = ir_ac_rms * MAX30102_BPM_RANGE_CAP_RMS_FACTOR;
  if (threshold_amplitude > amplitude_cap) { threshold_amplitude = amplitude_cap; }
  if (threshold_amplitude < MAX30102_BPM_MIN_AMPLITUDE) { threshold_amplitude = MAX30102_BPM_MIN_AMPLITUDE; }

  threshold = (int32_t)((threshold_amplitude * MAX30102_BPM_THRESHOLD_NUM) /
                        MAX30102_BPM_THRESHOLD_DEN);
  if (threshold < (int32_t)MAX30102_BPM_MIN_THRESHOLD) { threshold = MAX30102_BPM_MIN_THRESHOLD; }

  prominence_threshold = (int32_t)((threshold_amplitude * MAX30102_BPM_PROMINENCE_NUM) /
                                   MAX30102_BPM_PROMINENCE_DEN);
  if (prominence_threshold < (int32_t)MAX30102_BPM_MIN_PROMINENCE) { prominence_threshold = MAX30102_BPM_MIN_PROMINENCE; }
  if (prominence_threshold < threshold) { prominence_threshold = threshold; }

  edge_threshold = (int32_t)(ir_ac_rms / 5U);
  if (edge_threshold < (int32_t)MAX30102_BPM_MIN_EDGE_DELTA) { edge_threshold = MAX30102_BPM_MIN_EDGE_DELTA; }

  min_peak_distance = (uint16_t)((MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / MAX30102_BPM_MAX_RESULT);
  if (min_peak_distance < 10U) { min_peak_distance = 10U; }

  max_interval_samples = (uint16_t)((MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / MAX30102_BPM_MIN_RESULT);

  peak_count = 0U;
  if (sample_count < 5U) { return 0U; }

  sample_index = start_index;
  prev2 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  prev1 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  current = spo2_state->ir_filtered_samples[sample_index];
  sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  next1 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  next2 = spo2_state->ir_filtered_samples[sample_index];
  sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }

  for (i = 2U; i + 2U < sample_count; i++)
  {
    smoothed_prev1 = (prev2 + prev1 + current) / 3;
    smoothed_current = (prev1 + current + next1) / 3;
    smoothed_next = (current + next1 + next2) / 3;
    left_min = prev2; if (prev1 < left_min) { left_min = prev1; }
    right_min = next1; if (next2 < right_min) { right_min = next2; }

    prominence = smoothed_current - ((left_min > right_min) ? left_min : right_min);
    left_edge = smoothed_current - smoothed_prev1;
    right_edge = smoothed_current - smoothed_next;

    if ((smoothed_current > threshold) &&
        (smoothed_current >= smoothed_prev1) &&
        (smoothed_current >= smoothed_next) &&
        (prominence >= prominence_threshold) &&
        ((left_edge >= edge_threshold) || (right_edge >= edge_threshold)))
    {
      if ((peak_count == 0U) ||
          ((i - peak_positions[peak_count - 1U]) >= min_peak_distance))
      {
        if (peak_count < MAX30102_BPM_MAX_PEAK_COUNT)
        {
          peak_positions[peak_count] = i;
          peak_count++;
        }
      }
    }

    if ((i + 3U) < sample_count)
    {
      prev2 = prev1; prev1 = current; current = next1; next1 = next2;
      next2 = spo2_state->ir_filtered_samples[sample_index];
      sample_index++; if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
    }
  }

  if (peak_count < 2U) { return 0U; }

  interval_sum = 0U; interval_count = 0U;
  min_interval = 0xFFFFU; max_interval = 0U;

  for (i = 1U; i < peak_count; i++)
  {
    interval_samples = (uint16_t)(peak_positions[i] - peak_positions[i - 1U]);

    if ((interval_samples >= min_peak_distance) && (interval_samples <= max_interval_samples))
    {
      interval_sum += interval_samples; interval_count++;
      if (interval_samples < min_interval) { min_interval = interval_samples; }
      if (interval_samples > max_interval) { max_interval = interval_samples; }
    }
  }

  if (interval_count == 0U)
  {
    if (peak_count < 2U) { return 0U; }
    peak_span_samples = (uint32_t)(peak_positions[peak_count - 1U] - peak_positions[0]);
    if (peak_span_samples == 0U) { return 0U; }
    interval_sum = peak_span_samples; interval_count = (uint16_t)(peak_count - 1U);
    fallback_used = 1U;
  }

  if (interval_count >= 3U)
  {
    average_interval = (interval_sum + (interval_count / 2U)) / interval_count;
    allowed_jitter = (average_interval * MAX30102_BPM_MAX_INTERVAL_JITTER) / 100U;
    if (allowed_jitter < 3U) { allowed_jitter = 3U; }

    if ((uint32_t)(max_interval - min_interval) > allowed_jitter)
    {
      peak_span_samples = (uint32_t)(peak_positions[peak_count - 1U] - peak_positions[0]);
      if (peak_span_samples > 0U) { interval_sum = peak_span_samples; interval_count = (uint16_t)(peak_count - 1U); fallback_used = 1U; }
      else { return 0U; }
    }
  }

  if ((fallback_used == 0U) && (interval_count >= 3U) &&
      (interval_sum > ((uint32_t)min_interval + (uint32_t)max_interval)))
  {
    interval_sum -= (uint32_t)min_interval + (uint32_t)max_interval;
    interval_count -= 2U;
  }

  bpm_estimate = (uint32_t)(((uint32_t)60U * MAX30102_ALGO_SAMPLE_RATE_HZ * interval_count +
                             (interval_sum / 2U)) / interval_sum);

  if ((bpm_estimate < MAX30102_BPM_MIN_RESULT) || (bpm_estimate > MAX30102_BPM_MAX_RESULT))
  {
    return 0U;
  }

  if (pulse_info != NULL)
  {
    latest_interval_samples = 0U; latest_peak_position = 0U;

    for (i = (uint16_t)(peak_count - 1U); i > 0U; i--)
    {
      interval_samples = (uint16_t)(peak_positions[i] - peak_positions[i - 1U]);
      if ((interval_samples >= min_peak_distance) && (interval_samples <= max_interval_samples))
      {
        latest_interval_samples = interval_samples; latest_peak_position = peak_positions[i];
        break;
      }
    }

    if ((latest_interval_samples != 0U) && (spo2_state->total_samples >= sample_count))
    {
      uint16_t peak_window_start, peak_window_end, peak_sample_index, peak_offset;
      int32_t peak_value, local_min, local_value;

      peak_window_start = (latest_peak_position > MAX30102_BPM_PEAK_AMP_HALF_WINDOW) ?
                          (uint16_t)(latest_peak_position - MAX30102_BPM_PEAK_AMP_HALF_WINDOW) : 0U;
      peak_window_end = (uint16_t)(latest_peak_position + MAX30102_BPM_PEAK_AMP_HALF_WINDOW);
      if (peak_window_end >= sample_count) { peak_window_end = (uint16_t)(sample_count - 1U); }

      peak_sample_index = (uint16_t)((start_index + latest_peak_position) % MAX30102_SPO2_WINDOW_SIZE);
      peak_value = spo2_state->ir_filtered_samples[peak_sample_index];
      local_min = peak_value;

      for (peak_offset = peak_window_start; peak_offset <= peak_window_end; peak_offset++)
      {
        peak_sample_index = (uint16_t)((start_index + peak_offset) % MAX30102_SPO2_WINDOW_SIZE);
        local_value = spo2_state->ir_filtered_samples[peak_sample_index];
        if (local_value < local_min) { local_min = local_value; }
      }

      oldest_sample_number = spo2_state->total_samples - sample_count;
      pulse_info->beat_valid = 1U;
      pulse_info->interval_samples = latest_interval_samples;
      pulse_info->latest_ibi_ms = (uint16_t)(((uint32_t)latest_interval_samples * 1000U +
                                              (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                             MAX30102_ALGO_SAMPLE_RATE_HZ);
      pulse_info->latest_peak_sample = oldest_sample_number + latest_peak_position;
      pulse_info->beat_amplitude = (peak_value > local_min) ? (uint32_t)(peak_value - local_min) : 0U;
    }
  }

  *bpm_value = (uint8_t)bpm_estimate;
  return 1U;
}

/**
 * @brief  使用基于 FFT 的自相关（Wiener-Khinchin）估算 BPM。
 * @param  spo2_state SpO2 状态结构体指针。
 * @param  bpm        估算 BPM 的输出指针。
 * @return 成功返回 1；样本不足或未找到有效峰时返回 0。
 * @note   使用 256 点 RFFT 和 Hann 窗。100 Hz 采样率下，
 *         延迟范围 [27, 171] 的自相关峰对应 [35, 220] BPM。
 *         对低灌注信号比峰值检测更稳健。
 */
uint8_t max30102_autocorr_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm)
{
  uint16_t i;
  uint16_t sample_count;
  uint16_t start_index;
  uint16_t sample_index;
  float32_t mean_val;
  uint32_t lag;
  float32_t max_val;
  uint32_t max_lag;
  uint32_t bpm_estimate;

  if ((spo2_state == NULL) || (bpm == NULL))
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  if (sample_count < FFT_SIZE)
  {
    return 0U;
  }

  autocorr_ensure_init();

  start_index = spo2_state->write_index;
  sample_index = start_index;
  for (i = 0U; i < FFT_SIZE; i++)
  {
    fft_in[i] = (float32_t)spo2_state->ir_filtered_samples[sample_index];
    sample_index++;
    if (sample_index >= MAX30102_SPO2_WINDOW_SIZE) { sample_index = 0U; }
  }

  arm_mean_f32(fft_in, FFT_SIZE, &mean_val);
  for (i = 0U; i < FFT_SIZE; i++)
  {
    fft_in[i] -= mean_val;
    fft_in[i] *= 0.5f * (1.0f - arm_cos_f32(2.0f * 3.14159265f * (float32_t)i
                                             / (float32_t)(FFT_SIZE - 1U)));
  }

  arm_rfft_fast_f32(&rfft_inst, fft_in, fft_out, 0U);

  {
    float32_t re, im;
    re = fft_out[0]; fft_out[0] = re * re;
    re = fft_out[1]; fft_out[1] = re * re;
    for (i = 2U; i < FFT_SIZE; i += 2U)
    {
      re = fft_out[i]; im = fft_out[i + 1U];
      fft_out[i] = re * re + im * im;
      fft_out[i + 1U] = 0.0f;
    }
  }

  arm_rfft_fast_f32(&rfft_inst, fft_out, fft_in, 1U);

  for (i = 0U; i < FFT_SIZE; i++) { fft_in[i] /= (float32_t)FFT_SIZE; }

  max_val = -1e30f; max_lag = LAG_MIN;
  for (lag = LAG_MIN; lag <= LAG_MAX; lag++)
  {
    if (fft_in[lag] > max_val) { max_val = fft_in[lag]; max_lag = lag; }
  }

  if (max_val < (fft_in[0] * AUTOCORR_MIN_PEAK_RATIO)) { return 0U; }

  bpm_estimate = (MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / max_lag;
  if ((bpm_estimate < MAX30102_BPM_MIN_RESULT) || (bpm_estimate > MAX30102_BPM_MAX_RESULT))
  {
    return 0U;
  }

  *bpm = (uint8_t)bpm_estimate;
  return 1U;
}
