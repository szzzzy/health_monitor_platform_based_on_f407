/*
 * RR 模块——RIAV (Respiratory-Induced Amplitude Variation) 呼吸率估计。
 *
 * 原理: 呼吸会调节胸腔压力→每搏输出量→PPG 脉搏幅度。
 *       对逐拍幅值序列做包络检测，提取呼吸频率。
 *
 * 门控条件（比 IBI/HRV 更慢更严格）:
 *   - SQ >= 25 (与 BPM 一致; 弱但稳定的 PPG 仍可累积包络)
 *   - >= 8 个心搏 (vs HRV 的 >=3)
 *   - >= 600 samples = 6 秒时间窗口
 *   - 脉搏幅度调制深度 >= amplitude_mean/25 且 >=4 (排除噪声引起的假调制)
 *   - 输出范围 [8, 30] breaths/min
 *
 * 历史管理:
 *   - Reset (手指离开/测量重置): 清零 amplitude buffer + rr_bpm。
 *   - SQ 不足/motion: 只标 valid=0，不清 buffer。
 *   - Motion artifact: app_measurement 层冻结，不清 RR 历史。
 *
 * 未实现: RIFV (Respiratory-Induced Frequency Variation)、RIIDV。
 */

#include "app_rr.h"

#include <string.h>

#include "max30102.h"

#define APP_RR_RIAV_HISTORY_SIZE  32U
#define APP_RR_MIN_BEAT_COUNT     6U
/* 4 秒最小时间窗口（100 Hz 采样率）。 */
#define APP_RR_MIN_WINDOW_SAMPLES 400U
#define APP_RR_MIN_RESULT         8U
#define APP_RR_MAX_RESULT         30U
/* 调制深度须 >= amplitude_mean / 25 (4%) 且 >= 4 个原始单位。 */
#define APP_RR_MIN_MODULATION_DIV 25U

static struct
{
  uint32_t peak_sample[APP_RR_RIAV_HISTORY_SIZE];
  uint32_t amplitude[APP_RR_RIAV_HISTORY_SIZE];
  uint8_t write_index;
  uint8_t count;
} rr_state;

static uint8_t app_rr_order_to_index(uint8_t order);
static uint32_t app_rr_get_peak_sample(uint8_t order);
static uint32_t app_rr_get_amplitude(uint8_t order);

void app_rr_reset(AppState_t *app)
{
  (void)memset(&rr_state, 0, sizeof(rr_state));

  if (app == NULL)
  {
    return;
  }

  app->rr_valid = 0U;
  app->rr_bpm = 0U;
}

void app_rr_add_beat(uint32_t peak_sample, uint32_t amplitude)
{
  rr_state.peak_sample[rr_state.write_index] = peak_sample;
  rr_state.amplitude[rr_state.write_index] = amplitude;
  rr_state.write_index = (uint8_t)((rr_state.write_index + 1U) % APP_RR_RIAV_HISTORY_SIZE);
  if (rr_state.count < APP_RR_RIAV_HISTORY_SIZE)
  {
    rr_state.count++;
  }
}

void app_rr_update_output(AppState_t *app)
{
  uint8_t i;
  uint8_t peak_count = 0U;
  uint32_t resp_peak_samples[8];
  uint32_t amplitude;
  uint32_t amplitude_min = 0xFFFFFFFFUL;
  uint32_t amplitude_max = 0U;
  uint64_t amplitude_sum = 0ULL;
  uint32_t amplitude_mean;
  uint32_t modulation_span;
  uint32_t modulation_threshold;
  uint32_t window_span;
  uint32_t interval_sum = 0U;
  uint8_t interval_count = 0U;
  uint32_t min_rr_interval;
  uint32_t max_rr_interval;
  uint32_t rr_estimate;
  uint32_t prev_amp;
  uint32_t curr_amp;
  uint32_t next_amp;
  uint32_t neighbor_max;
  uint32_t interval_samples;

  if (app == NULL)
  {
    return;
  }

  /* 门控 1：amplitude ring buffer 中的最低拍数。 */
  if (rr_state.count < APP_RR_MIN_BEAT_COUNT)
  {
    return;
  }

  /* 门控 2：最小时间窗口（100 Hz 下 6 秒）。 */
  window_span = app_rr_get_peak_sample((uint8_t)(rr_state.count - 1U)) - app_rr_get_peak_sample(0U);
  if (window_span < APP_RR_MIN_WINDOW_SAMPLES)
  {
    return;
  }

  for (i = 0U; i < rr_state.count; i++)
  {
    amplitude = app_rr_get_amplitude(i);
    amplitude_sum += amplitude;
    if (amplitude < amplitude_min)
    {
      amplitude_min = amplitude;
    }

    if (amplitude > amplitude_max)
    {
      amplitude_max = amplitude;
    }
  }

  amplitude_mean = (uint32_t)((amplitude_sum + (rr_state.count / 2U)) / rr_state.count);
  modulation_span = amplitude_max - amplitude_min;
  /*
   * 门控 3：幅度调制必须可见。
   * 要求调制深度 >= amplitude_mean/25 (4%) 且 >= 4 个原始单位。
   * 排除噪声驱动的假包络波动。
   */
  if ((amplitude_mean == 0U) ||
      (modulation_span < (amplitude_mean / APP_RR_MIN_MODULATION_DIV)) ||
      (modulation_span < 4U))
  {
    return;
  }

  modulation_threshold = modulation_span / 6U;
  if (modulation_threshold < 2U)
  {
    modulation_threshold = 2U;
  }

  for (i = 1U; (i + 1U) < rr_state.count; i++)
  {
    prev_amp = app_rr_get_amplitude((uint8_t)(i - 1U));
    curr_amp = app_rr_get_amplitude(i);
    next_amp = app_rr_get_amplitude((uint8_t)(i + 1U));
    neighbor_max = (prev_amp > next_amp) ? prev_amp : next_amp;

    if ((curr_amp >= prev_amp) &&
        (curr_amp > next_amp) &&
        ((curr_amp - neighbor_max) >= modulation_threshold))
    {
      if (peak_count < (sizeof(resp_peak_samples) / sizeof(resp_peak_samples[0])))
      {
        resp_peak_samples[peak_count] = app_rr_get_peak_sample(i);
        peak_count++;
      }
    }
  }

  if (peak_count < 2U)
  {
    return;
  }

  min_rr_interval = ((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / APP_RR_MAX_RESULT;
  max_rr_interval = ((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / APP_RR_MIN_RESULT;

  for (i = 1U; i < peak_count; i++)
  {
    interval_samples = resp_peak_samples[i] - resp_peak_samples[i - 1U];

    if ((interval_samples >= min_rr_interval) && (interval_samples <= max_rr_interval))
    {
      interval_sum += interval_samples;
      interval_count++;
    }
  }

  if (interval_count == 0U)
  {
    return;
  }

  rr_estimate = (((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U * interval_count) +
                 (interval_sum / 2U)) / interval_sum;
  if ((rr_estimate < APP_RR_MIN_RESULT) || (rr_estimate > APP_RR_MAX_RESULT))
  {
    return;
  }

  app->rr_valid = 1U;
  app->rr_bpm = (uint8_t)rr_estimate;
}

static uint8_t app_rr_order_to_index(uint8_t order)
{
  uint8_t start_index;

  if (rr_state.count < APP_RR_RIAV_HISTORY_SIZE)
  {
    start_index = 0U;
  }
  else
  {
    start_index = rr_state.write_index;
  }

  return (uint8_t)((start_index + order) % APP_RR_RIAV_HISTORY_SIZE);
}

static uint32_t app_rr_get_peak_sample(uint8_t order)
{
  return rr_state.peak_sample[app_rr_order_to_index(order)];
}

static uint32_t app_rr_get_amplitude(uint8_t order)
{
  return rr_state.amplitude[app_rr_order_to_index(order)];
}
