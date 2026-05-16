/*
 * HRV 时域与 Poincare 短窗口指标。
 *
 * 数据流：
 *   流式脉冲检测 → app_hrv_accept_ibi (跳变过滤) → 32 拍 ring buffer
 *   → app_hrv_update_outputs: SDNN, RMSSD, SD1, SD2, SD1/SD2 x100, rhythm_irregular。
 *
 * IBI 接受规则 (app_hrv_accept_ibi):
 *   - 绝对范围: [300, 2000] ms。
 *   - 相对跳变: 距已有均值偏差 ≤50%（仅当 buffer 中有 >=4 个 IBI 时启用）。
 *
 * Poincare 指标 (基于同一 32 拍窗口):
 *   SD1 = RMSSD / sqrt(2)               —— 短时变异性（Poincare 椭圆短轴）
 *   SD2 = sqrt(2*SDNN^2 - SD1^2)        —— 长时变异性（Poincare 椭圆长轴）
 *   SD1/SD2 x100                         —— 0–100 比值（高比值→短时变异主导）
 *
 * rhythm_irregular (短窗口提示，非诊断):
 *   - 条件: RMSSD >= 120 ms 且 SD1/SD2 x100 >= 70。
 *   - 仅反映当前 32 拍窗口内的节律不规整趋势，
 *     不等于房颤诊断——没有心率触诊、Poincare 密度图或临床标准分类。
 *
 * 历史管理:
 *   - Reset (手指离开/测量重置): 清零 ring buffer + 所有输出。
 *   - Invalidate (信号中断/motion): 只标 valid=0，保留 buffer 与旧值。
 *
 * Frequency HRV:
 *   - Short-window LF/HF estimate from the same 32-beat IBI ring buffer.
 *   - 4 Hz linear resampling, mean removal, Hann window, CMSIS-DSP RFFT.
 *   - LF: 0.04-0.15 Hz, HF: 0.15-0.40 Hz.
 *   - VLF is intentionally not estimated: 32 beats are too short for
 *     useful <0.04 Hz resolution, and this is not standard 5-minute HRV.
 */

#include "app_hrv.h"

#include "arm_math.h"

#include <string.h>

#define APP_HRV_IBI_HISTORY_SIZE       32U
#define APP_HRV_MIN_VALID_IBI_COUNT    3U
#define APP_HRV_JUMP_FILTER_MIN_COUNT  4U
#define APP_HRV_IBI_JUMP_PERCENT       50U
/* rhythm_irregular 阈值——短窗口提示，非诊断。 */
#define APP_HRV_IRREGULAR_RMSSD_MS     120U
#define APP_HRV_IRREGULAR_SD1_SD2_X100 70U

#define APP_HRV_FREQ_RESAMPLE_HZ       4.0f
#define APP_HRV_FREQ_MIN_FFT_SIZE      64U
#define APP_HRV_FREQ_MAX_FFT_SIZE      256U
#define APP_HRV_FREQ_LF_MIN_HZ         0.04f
#define APP_HRV_FREQ_LF_MAX_HZ         0.15f
#define APP_HRV_FREQ_HF_MIN_HZ         0.15f
#define APP_HRV_FREQ_HF_MAX_HZ         0.40f
#define APP_HRV_FREQ_EPSILON           1.0e-6f
#define APP_HRV_PI_F32                 3.14159265f

static struct
{
  uint16_t ibi_ms[APP_HRV_IBI_HISTORY_SIZE];
  uint8_t write_index;
  uint8_t count;
  uint8_t last_peak_valid;
  uint32_t last_peak_sample;
} hrv_state;

static float32_t hrv_freq_time_s[APP_HRV_IBI_HISTORY_SIZE];
static float32_t hrv_freq_ibi_ms[APP_HRV_IBI_HISTORY_SIZE];
static float32_t hrv_freq_fft_in[APP_HRV_FREQ_MAX_FFT_SIZE];
static float32_t hrv_freq_fft_out[APP_HRV_FREQ_MAX_FFT_SIZE];
static arm_rfft_fast_instance_f32 hrv_freq_rfft_inst;
static uint16_t hrv_freq_rfft_size;

static uint32_t app_hrv_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static uint32_t app_hrv_isqrt_u64(uint64_t value);
static uint8_t app_hrv_accept_ibi(uint16_t ibi_ms);
static uint8_t app_hrv_order_to_index(uint8_t order);
static void app_hrv_update_outputs(AppState_t *app);
static void app_hrv_update_frequency_outputs(AppState_t *app);
static uint16_t app_hrv_select_frequency_fft_size(float32_t duration_s);
static uint8_t app_hrv_frequency_ensure_rfft(uint16_t fft_size);
static float32_t app_hrv_interpolate_ibi(float32_t sample_time_s,
                                         uint8_t *segment_index);
static uint32_t app_hrv_float_to_u32_x100(float32_t value);
static uint16_t app_hrv_ratio_to_u16_x100(float32_t numerator,
                                          float32_t denominator);

void app_hrv_reset(AppState_t *app)
{
  (void)memset(&hrv_state, 0, sizeof(hrv_state));

  if (app == NULL)
  {
    return;
  }

  app->ibi_valid = 0U;
  app->latest_ibi_ms = 0U;
  app->hrv_valid = 0U;
  app->hrv_mean_ibi_ms = 0U;
  app->hrv_sdnn_ms = 0U;
  app->hrv_rmssd_ms = 0U;
  app->hrv_sd1_ms = 0U;
  app->hrv_sd2_ms = 0U;
  app->hrv_sd1_sd2_x100 = 0U;
  app->hrv_freq_valid = 0U;
  app->hrv_lf_power_x100 = 0U;
  app->hrv_hf_power_x100 = 0U;
  app->hrv_lf_hf_x100 = 0U;
  app->rhythm_irregular = 0U;
}

void app_hrv_invalidate_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->ibi_valid = 0U;
  app->hrv_valid = 0U;
  app->hrv_freq_valid = 0U;
}

uint8_t app_hrv_mark_peak_seen(uint32_t peak_sample)
{
  if ((hrv_state.last_peak_valid != 0U) &&
      (hrv_state.last_peak_sample == peak_sample))
  {
    return 0U;
  }

  hrv_state.last_peak_valid = 1U;
  hrv_state.last_peak_sample = peak_sample;
  return 1U;
}

uint8_t app_hrv_is_peak_stale(uint32_t current_sample, uint32_t stale_samples)
{
  if ((hrv_state.last_peak_valid != 0U) &&
      (current_sample > (hrv_state.last_peak_sample + stale_samples)))
  {
    return 1U;
  }

  return 0U;
}

uint8_t app_hrv_add_ibi(AppState_t *app, uint16_t ibi_ms)
{
  if (app_hrv_accept_ibi(ibi_ms) == 0U)
  {
    return 0U;
  }

  hrv_state.ibi_ms[hrv_state.write_index] = ibi_ms;
  hrv_state.write_index = (uint8_t)((hrv_state.write_index + 1U) % APP_HRV_IBI_HISTORY_SIZE);
  if (hrv_state.count < APP_HRV_IBI_HISTORY_SIZE)
  {
    hrv_state.count++;
  }

  if (app != NULL)
  {
    app->ibi_valid = 1U;
    app->latest_ibi_ms = ibi_ms;
  }

  app_hrv_update_outputs(app);
  return 1U;
}

static uint8_t app_hrv_accept_ibi(uint16_t ibi_ms)
{
  uint16_t i;
  uint32_t sum;
  uint16_t mean_ibi;

  /* 绝对范围门禁。 */
  if ((ibi_ms < APP_HRV_IBI_MIN_MS) || (ibi_ms > APP_HRV_IBI_MAX_MS))
  {
    return 0U;
  }

  /* 相对跳变过滤：仅在 buffer 中已有 >=4 个 IBI 时启用。 */
  if (hrv_state.count < APP_HRV_JUMP_FILTER_MIN_COUNT)
  {
    return 1U;
  }

  sum = 0U;
  for (i = 0U; i < hrv_state.count; i++)
  {
    sum += hrv_state.ibi_ms[i];
  }

  mean_ibi = (uint16_t)((sum + (hrv_state.count / 2U)) / hrv_state.count);
  if (mean_ibi == 0U)
  {
    return 1U;
  }

  if (app_hrv_abs_diff_u32(ibi_ms, mean_ibi) >
      (((uint32_t)mean_ibi * APP_HRV_IBI_JUMP_PERCENT) / 100U))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t app_hrv_order_to_index(uint8_t order)
{
  uint8_t start_index;

  if (hrv_state.count < APP_HRV_IBI_HISTORY_SIZE)
  {
    start_index = 0U;
  }
  else
  {
    start_index = hrv_state.write_index;
  }

  return (uint8_t)((start_index + order) % APP_HRV_IBI_HISTORY_SIZE);
}

static void app_hrv_update_outputs(AppState_t *app)
{
  uint8_t i;
  uint32_t sum = 0U;
  uint16_t mean_ibi;
  uint64_t variance_sum = 0ULL;
  uint64_t rmssd_sum = 0ULL;
  int32_t diff;
  uint16_t previous_ibi = 0U;
  uint16_t ibi_ms;
  uint32_t sd1_ms;
  uint32_t sd2_ms;
  uint64_t sd2_variance;
  uint64_t sd2_subterm;
  uint32_t sd1_sd2_x100;

  if (app == NULL)
  {
    return;
  }

  if (hrv_state.count < APP_HRV_MIN_VALID_IBI_COUNT)
  {
    app->hrv_valid = 0U;
    app->hrv_freq_valid = 0U;
    return;
  }

  for (i = 0U; i < hrv_state.count; i++)
  {
    sum += hrv_state.ibi_ms[app_hrv_order_to_index(i)];
  }

  mean_ibi = (uint16_t)((sum + (hrv_state.count / 2U)) / hrv_state.count);

  for (i = 0U; i < hrv_state.count; i++)
  {
    ibi_ms = hrv_state.ibi_ms[app_hrv_order_to_index(i)];

    diff = (int32_t)ibi_ms - (int32_t)mean_ibi;
    variance_sum += (uint64_t)((int64_t)diff * (int64_t)diff);

    if (i != 0U)
    {
      diff = (int32_t)ibi_ms - (int32_t)previous_ibi;
      rmssd_sum += (uint64_t)((int64_t)diff * (int64_t)diff);
    }

    previous_ibi = ibi_ms;
  }

  app->hrv_valid = 1U;
  app->hrv_mean_ibi_ms = mean_ibi;
  app->hrv_sdnn_ms = (uint16_t)app_hrv_isqrt_u64(variance_sum / (uint64_t)(hrv_state.count - 1U));
  app->hrv_rmssd_ms = (uint16_t)app_hrv_isqrt_u64(rmssd_sum / (uint64_t)(hrv_state.count - 1U));

  /*
   * 从同一 32 拍 IBI 序列推导 Poincare SD1/SD2：
   * SD1 = sqrt(SDSD / 2) ≈ RMSSD / sqrt(2)  —— 短时变异性（椭圆短轴）
   * SD2 = sqrt(2*SDNN^2 - SD1^2)             —— 长时变异性（椭圆长轴）
   * SD1/SD2 x100 = Poincare 比值 × 100。
   */
  sd1_ms = app_hrv_isqrt_u64((((uint64_t)app->hrv_rmssd_ms * (uint64_t)app->hrv_rmssd_ms) + 1ULL) / 2ULL);
  if (sd1_ms > 0xFFFFUL)
  {
    sd1_ms = 0xFFFFUL;
  }

  sd2_variance = 2ULL * (uint64_t)app->hrv_sdnn_ms * (uint64_t)app->hrv_sdnn_ms;
  sd2_subterm = (((uint64_t)app->hrv_rmssd_ms * (uint64_t)app->hrv_rmssd_ms) + 1ULL) / 2ULL;
  if (sd2_variance > sd2_subterm)
  {
    sd2_ms = app_hrv_isqrt_u64(sd2_variance - sd2_subterm);
  }
  else
  {
    sd2_ms = 0U;
  }

  if (sd2_ms > 0xFFFFUL)
  {
    sd2_ms = 0xFFFFUL;
  }

  if (sd2_ms != 0U)
  {
    sd1_sd2_x100 = (uint32_t)(((uint64_t)sd1_ms * 100ULL + (sd2_ms / 2U)) / sd2_ms);
  }
  else if (sd1_ms != 0U)
  {
    sd1_sd2_x100 = 0xFFFFUL;
  }
  else
  {
    sd1_sd2_x100 = 0U;
  }

  if (sd1_sd2_x100 > 0xFFFFUL)
  {
    sd1_sd2_x100 = 0xFFFFUL;
  }

  app->hrv_sd1_ms = (uint16_t)sd1_ms;
  app->hrv_sd2_ms = (uint16_t)sd2_ms;
  app->hrv_sd1_sd2_x100 = (uint16_t)sd1_sd2_x100;

  /*
   * 短窗口节律不齐提示（非诊断）。
   * 两个条件在当前 32 拍窗口内同时满足时才置位：
   * - RMSSD >= 120 ms（逐拍变异性高）
   * - SD1/SD2 x100 >= 70（短时变异性主导）
   */
  app->rhythm_irregular =
      ((app->hrv_rmssd_ms >= APP_HRV_IRREGULAR_RMSSD_MS) &&
       (app->hrv_sd1_sd2_x100 >= APP_HRV_IRREGULAR_SD1_SD2_X100)) ? 1U : 0U;

  app_hrv_update_frequency_outputs(app);
}

static void app_hrv_update_frequency_outputs(AppState_t *app)
{
  uint16_t i;
  uint8_t segment_index;
  uint16_t fft_size;
  uint16_t k;
  float32_t duration_s;
  float32_t cumulative_s;
  float32_t sample_start_s;
  float32_t sample_time_s;
  float32_t mean_val;
  float32_t window;
  float32_t window_power;
  float32_t bin_hz;
  float32_t re;
  float32_t im;
  float32_t mag2;
  float32_t bin_power;
  float32_t lf_power;
  float32_t hf_power;

  if (app == NULL)
  {
    return;
  }

  /*
   * Frequency-domain HRV needs the complete 32-beat window. With fewer beats
   * the LF band is too under-resolved, so only the time-domain fields are valid.
   */
  if (hrv_state.count < APP_HRV_IBI_HISTORY_SIZE)
  {
    app->hrv_freq_valid = 0U;
    return;
  }

  /* Treat each IBI as the tachogram value at the middle of its beat interval. */
  cumulative_s = 0.0f;
  for (i = 0U; i < APP_HRV_IBI_HISTORY_SIZE; i++)
  {
    hrv_freq_ibi_ms[i] = (float32_t)hrv_state.ibi_ms[app_hrv_order_to_index((uint8_t)i)];
    hrv_freq_time_s[i] = cumulative_s + (hrv_freq_ibi_ms[i] * 0.0005f);
    cumulative_s += hrv_freq_ibi_ms[i] * 0.001f;
  }

  duration_s = cumulative_s;
  fft_size = app_hrv_select_frequency_fft_size(duration_s);
  if ((fft_size == 0U) || (app_hrv_frequency_ensure_rfft(fft_size) == 0U))
  {
    app->hrv_freq_valid = 0U;
    return;
  }

  sample_start_s = duration_s -
                   ((float32_t)(fft_size - 1U) / APP_HRV_FREQ_RESAMPLE_HZ);
  if (sample_start_s < 0.0f)
  {
    sample_start_s = 0.0f;
  }

  segment_index = 0U;
  mean_val = 0.0f;
  for (i = 0U; i < fft_size; i++)
  {
    sample_time_s = sample_start_s + ((float32_t)i / APP_HRV_FREQ_RESAMPLE_HZ);
    hrv_freq_fft_in[i] = app_hrv_interpolate_ibi(sample_time_s, &segment_index);
    mean_val += hrv_freq_fft_in[i];
  }
  mean_val /= (float32_t)fft_size;

  window_power = 0.0f;
  for (i = 0U; i < fft_size; i++)
  {
    window = 0.5f * (1.0f - arm_cos_f32((2.0f * APP_HRV_PI_F32 * (float32_t)i) /
                                        (float32_t)(fft_size - 1U)));
    hrv_freq_fft_in[i] = (hrv_freq_fft_in[i] - mean_val) * window;
    window_power += window * window;
  }

  if (window_power <= APP_HRV_FREQ_EPSILON)
  {
    app->hrv_freq_valid = 0U;
    return;
  }

  arm_rfft_fast_f32(&hrv_freq_rfft_inst, hrv_freq_fft_in, hrv_freq_fft_out, 0U);

  lf_power = 0.0f;
  hf_power = 0.0f;
  bin_hz = APP_HRV_FREQ_RESAMPLE_HZ / (float32_t)fft_size;
  for (k = 1U; k <= (fft_size / 2U); k++)
  {
    float32_t freq_hz = bin_hz * (float32_t)k;

    if (k == (fft_size / 2U))
    {
      re = hrv_freq_fft_out[1];
      im = 0.0f;
      mag2 = re * re;
    }
    else
    {
      re = hrv_freq_fft_out[2U * k];
      im = hrv_freq_fft_out[(2U * k) + 1U];
      mag2 = (re * re) + (im * im);
    }

    bin_power = ((k == (fft_size / 2U)) ? mag2 : (2.0f * mag2)) /
                ((float32_t)fft_size * window_power);

    if ((freq_hz >= APP_HRV_FREQ_LF_MIN_HZ) && (freq_hz < APP_HRV_FREQ_LF_MAX_HZ))
    {
      lf_power += bin_power;
    }
    else if ((freq_hz >= APP_HRV_FREQ_HF_MIN_HZ) && (freq_hz <= APP_HRV_FREQ_HF_MAX_HZ))
    {
      hf_power += bin_power;
    }
  }

  if ((lf_power <= APP_HRV_FREQ_EPSILON) && (hf_power <= APP_HRV_FREQ_EPSILON))
  {
    app->hrv_freq_valid = 0U;
    return;
  }

  app->hrv_lf_power_x100 = app_hrv_float_to_u32_x100(lf_power);
  app->hrv_hf_power_x100 = app_hrv_float_to_u32_x100(hf_power);
  app->hrv_lf_hf_x100 = app_hrv_ratio_to_u16_x100(lf_power, hf_power);
  app->hrv_freq_valid = 1U;
}

static uint16_t app_hrv_select_frequency_fft_size(float32_t duration_s)
{
  uint32_t available_samples;

  if (duration_s <= 0.0f)
  {
    return 0U;
  }

  available_samples = (uint32_t)(duration_s * APP_HRV_FREQ_RESAMPLE_HZ) + 1U;
  if (available_samples >= APP_HRV_FREQ_MAX_FFT_SIZE)
  {
    return APP_HRV_FREQ_MAX_FFT_SIZE;
  }

  if (available_samples >= 128U)
  {
    return 128U;
  }

  if (available_samples >= APP_HRV_FREQ_MIN_FFT_SIZE)
  {
    return APP_HRV_FREQ_MIN_FFT_SIZE;
  }

  return 0U;
}

static uint8_t app_hrv_frequency_ensure_rfft(uint16_t fft_size)
{
  if (hrv_freq_rfft_size == fft_size)
  {
    return 1U;
  }

  if (arm_rfft_fast_init_f32(&hrv_freq_rfft_inst, fft_size) != ARM_MATH_SUCCESS)
  {
    hrv_freq_rfft_size = 0U;
    return 0U;
  }

  hrv_freq_rfft_size = fft_size;
  return 1U;
}

static float32_t app_hrv_interpolate_ibi(float32_t sample_time_s,
                                         uint8_t *segment_index)
{
  uint8_t seg;
  float32_t t0;
  float32_t t1;
  float32_t ratio;

  if (segment_index == NULL)
  {
    return hrv_freq_ibi_ms[0];
  }

  seg = *segment_index;
  while ((seg < (APP_HRV_IBI_HISTORY_SIZE - 2U)) &&
         (sample_time_s > hrv_freq_time_s[seg + 1U]))
  {
    seg++;
  }
  *segment_index = seg;

  if (sample_time_s <= hrv_freq_time_s[0])
  {
    return hrv_freq_ibi_ms[0];
  }

  if (sample_time_s >= hrv_freq_time_s[APP_HRV_IBI_HISTORY_SIZE - 1U])
  {
    return hrv_freq_ibi_ms[APP_HRV_IBI_HISTORY_SIZE - 1U];
  }

  t0 = hrv_freq_time_s[seg];
  t1 = hrv_freq_time_s[seg + 1U];
  if (t1 <= t0)
  {
    return hrv_freq_ibi_ms[seg];
  }

  ratio = (sample_time_s - t0) / (t1 - t0);
  return hrv_freq_ibi_ms[seg] +
         ((hrv_freq_ibi_ms[seg + 1U] - hrv_freq_ibi_ms[seg]) * ratio);
}

static uint32_t app_hrv_float_to_u32_x100(float32_t value)
{
  if (value <= 0.0f)
  {
    return 0U;
  }

  if (value >= 42949672.0f)
  {
    return 0xFFFFFFFFUL;
  }

  return (uint32_t)((value * 100.0f) + 0.5f);
}

static uint16_t app_hrv_ratio_to_u16_x100(float32_t numerator,
                                          float32_t denominator)
{
  float32_t ratio_x100;

  if (denominator <= APP_HRV_FREQ_EPSILON)
  {
    return (numerator > APP_HRV_FREQ_EPSILON) ? 0xFFFFU : 0U;
  }

  ratio_x100 = (numerator * 100.0f) / denominator;
  if (ratio_x100 <= 0.0f)
  {
    return 0U;
  }

  if (ratio_x100 >= 65535.0f)
  {
    return 0xFFFFU;
  }

  return (uint16_t)(ratio_x100 + 0.5f);
}

static uint32_t app_hrv_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  if (lhs >= rhs)
  {
    return lhs - rhs;
  }

  return rhs - lhs;
}

static uint32_t app_hrv_isqrt_u64(uint64_t value)
{
  uint64_t bit = 1ULL << 62;
  uint64_t result = 0ULL;

  while (bit > value)
  {
    bit >>= 2U;
  }

  while (bit != 0ULL)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1U) + bit;
    }
    else
    {
      result >>= 1U;
    }

    bit >>= 2U;
  }

  if (result > 0xFFFFFFFFULL)
  {
    return 0xFFFFFFFFUL;
  }

  return (uint32_t)result;
}
