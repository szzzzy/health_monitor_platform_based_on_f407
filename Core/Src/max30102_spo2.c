/**
  ******************************************************************************
  * @file    max30102_spo2.c
  * @brief   MAX30102 SpO2 窗口管理、信号质量、血氧估算 + Butterworth 带通滤波器
  ******************************************************************************
  */

#include "max30102_spo2.h"
#include "max30102_algo_utils.h"
#include "arm_math.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * CMSIS-DSP 4th-order Butterworth bandpass (2 biquad stages)
 *
 * Designed for fs = 100 Hz, passband ≈ 0.5–5 Hz (30–300 BPM).
 * Coefficients are in CMSIS order: {b0, b1, b2, -a1, -a2}.
 * -------------------------------------------------------------------------- */
#define BIQUAD_STAGES 2U

static const float32_t biquad_coeffs[5U * BIQUAD_STAGES] = {
  /* Stage 1 – HPF, fc ≈ 0.5 Hz */
   0.97803048f, -1.95606096f, 0.97803048f,  1.95557824f, -0.95654368f,
  /* Stage 2 – LPF, fc ≈ 5 Hz */
   0.02008337f,  0.04016673f, 0.02008337f,  1.56101808f, -0.64135154f
};

static arm_biquad_casd_df1_inst_f32 biquad_red;
static arm_biquad_casd_df1_inst_f32 biquad_ir;
static float32_t biquad_state_red[4U * BIQUAD_STAGES];
static float32_t biquad_state_ir[4U * BIQUAD_STAGES];
static uint8_t biquad_initialized = 0U;

static void biquad_ensure_init(void)
{
  if (biquad_initialized != 0U)
  {
    return;
  }

  arm_biquad_cascade_df1_init_f32(&biquad_red, BIQUAD_STAGES,
                                  (float32_t *)biquad_coeffs, biquad_state_red);
  arm_biquad_cascade_df1_init_f32(&biquad_ir, BIQUAD_STAGES,
                                  (float32_t *)biquad_coeffs, biquad_state_ir);
  biquad_initialized = 1U;
}

static void biquad_reset(void)
{
  (void)memset(biquad_state_red, 0, sizeof(biquad_state_red));
  (void)memset(biquad_state_ir,  0, sizeof(biquad_state_ir));
  biquad_initialized = 0U;
  biquad_ensure_init();
}

/* ---- 公共 API ---- */

/*
 * 重置测量算法窗口。
 * 当前采用固定长度滑动窗口缓存最近一段 RED/IR 样本，
 * 供 BPM 峰值检测与 SpO2 AC/DC 比值计算共同使用。
 */
void max30102_spo2_reset(MAX30102_SpO2_t *spo2_state)
{
  uint16_t i;

  if (spo2_state == NULL)
  {
    return;
  }

  for (i = 0U; i < MAX30102_SPO2_WINDOW_SIZE; i++)
  {
    spo2_state->red_samples[i] = 0U;
    spo2_state->ir_samples[i] = 0U;
    spo2_state->red_filtered_samples[i] = 0;
    spo2_state->ir_filtered_samples[i] = 0;
  }

  spo2_state->write_index = 0U;
  spo2_state->sample_count = 0U;
  spo2_state->red_dc_estimate = 0U;
  spo2_state->ir_dc_estimate = 0U;
  spo2_state->red_filter_state = 0;
  spo2_state->ir_filter_state = 0;
  spo2_state->red_sum = 0ULL;
  spo2_state->ir_sum = 0ULL;
  spo2_state->red_square_sum = 0ULL;
  spo2_state->ir_square_sum = 0ULL;
  spo2_state->red_filtered_square_sum = 0ULL;
  spo2_state->ir_filtered_square_sum = 0ULL;
  spo2_state->total_samples = 0U;
  biquad_reset();
}

/* 向测量算法窗口中压入一个新的 RED/IR 样本。 */
void max30102_spo2_add_sample(MAX30102_SpO2_t *spo2_state, uint32_t red_value, uint32_t ir_value)
{
  int32_t old_red_filtered;
  int32_t old_ir_filtered;
  uint16_t sample_index;

  if (spo2_state == NULL)
  {
    return;
  }

  sample_index = spo2_state->write_index;
  if (spo2_state->sample_count >= MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->red_sum -= spo2_state->red_samples[sample_index];
    spo2_state->ir_sum -= spo2_state->ir_samples[sample_index];
    spo2_state->red_square_sum -= max30102_square_u32(spo2_state->red_samples[sample_index]);
    spo2_state->ir_square_sum -= max30102_square_u32(spo2_state->ir_samples[sample_index]);

    old_red_filtered = spo2_state->red_filtered_samples[sample_index];
    old_ir_filtered = spo2_state->ir_filtered_samples[sample_index];
    spo2_state->red_filtered_square_sum -= max30102_square_i32(old_red_filtered);
    spo2_state->ir_filtered_square_sum -= max30102_square_i32(old_ir_filtered);
  }

  /* 维护慢速 DC 估计（用于信号质量 PI 计算和外部诊断）。 */
  if (spo2_state->red_dc_estimate == 0U)
  {
    spo2_state->red_dc_estimate = red_value;
  }
  else
  {
    spo2_state->red_dc_estimate = max30102_slow_follow_u32(spo2_state->red_dc_estimate,
                                                           red_value,
                                                           MAX30102_FILTER_DC_SHIFT);
  }

  if (spo2_state->ir_dc_estimate == 0U)
  {
    spo2_state->ir_dc_estimate = ir_value;
  }
  else
  {
    spo2_state->ir_dc_estimate = max30102_slow_follow_u32(spo2_state->ir_dc_estimate,
                                                          ir_value,
                                                          MAX30102_FILTER_DC_SHIFT);
  }

  /* CMSIS-DSP 4th-order Butterworth bandpass isolates the pulse waveform. */
  {
    float32_t in, out;

    biquad_ensure_init();

    in = (float32_t)((int32_t)red_value - (int32_t)spo2_state->red_dc_estimate);
    arm_biquad_cascade_df1_f32(&biquad_red, &in, &out, 1U);
    spo2_state->red_filter_state = (int32_t)(out + (out >= 0.0f ? 0.5f : -0.5f));

    in = (float32_t)((int32_t)ir_value - (int32_t)spo2_state->ir_dc_estimate);
    arm_biquad_cascade_df1_f32(&biquad_ir, &in, &out, 1U);
    spo2_state->ir_filter_state = (int32_t)(out + (out >= 0.0f ? 0.5f : -0.5f));
  }

  spo2_state->red_samples[sample_index] = red_value;
  spo2_state->ir_samples[sample_index] = ir_value;
  spo2_state->red_filtered_samples[sample_index] = spo2_state->red_filter_state;
  spo2_state->ir_filtered_samples[sample_index] = spo2_state->ir_filter_state;
  spo2_state->red_sum += red_value;
  spo2_state->ir_sum += ir_value;
  spo2_state->red_square_sum += max30102_square_u32(red_value);
  spo2_state->ir_square_sum += max30102_square_u32(ir_value);
  spo2_state->red_filtered_square_sum += max30102_square_i32(spo2_state->red_filter_state);
  spo2_state->ir_filtered_square_sum += max30102_square_i32(spo2_state->ir_filter_state);

  spo2_state->write_index++;
  if (spo2_state->write_index >= MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->write_index = 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_WINDOW_SIZE)
  {
    spo2_state->sample_count++;
  }

  /* total_samples 永不重置的全局累加器，给流式脉冲检测提供绝对采样编号参考。 */
  if (spo2_state->total_samples < 0xFFFFFFFFUL)
  {
    spo2_state->total_samples++;
  }
}

uint8_t max30102_spo2_get_latest_filtered(const MAX30102_SpO2_t *spo2_state,
                                          int32_t *red_filtered,
                                          int32_t *ir_filtered)
{
  uint16_t sample_index;

  if ((spo2_state == NULL) || (red_filtered == NULL) || (ir_filtered == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count == 0U)
  {
    return 0U;
  }

  sample_index = (spo2_state->write_index == 0U) ?
                 (uint16_t)(MAX30102_SPO2_WINDOW_SIZE - 1U) :
                 (uint16_t)(spo2_state->write_index - 1U);
  *red_filtered = spo2_state->red_filtered_samples[sample_index];
  *ir_filtered = spo2_state->ir_filtered_samples[sample_index];
  return 1U;
}

uint8_t max30102_get_signal_metrics(const MAX30102_SpO2_t *spo2_state, MAX30102_SignalMetrics_t *metrics)
{
  uint16_t sample_count;
  uint64_t denominator;
  uint32_t red_pi_rms;
  uint32_t ir_pi_rms;

  if ((spo2_state == NULL) || (metrics == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  metrics->red_dc = (uint32_t)(spo2_state->red_sum / sample_count);
  metrics->ir_dc = (uint32_t)(spo2_state->ir_sum / sample_count);
  metrics->red_ac_rms = max30102_calculate_window_rms(spo2_state->red_filtered_square_sum, sample_count);
  metrics->ir_ac_rms = max30102_calculate_window_rms(spo2_state->ir_filtered_square_sum, sample_count);

  /*
   * PI uses max(filtered RMS, raw centered RMS) as the AC source.
   * Filtered RMS decays after the DC tracker settles on a steady finger,
   * but the raw centered RMS retains the true pulse amplitude.
   */
  red_pi_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                               spo2_state->red_square_sum,
                                               sample_count);
  ir_pi_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                              spo2_state->ir_square_sum,
                                              sample_count);
  if (red_pi_rms < metrics->red_ac_rms)  { red_pi_rms = metrics->red_ac_rms; }
  if (ir_pi_rms < metrics->ir_ac_rms)    { ir_pi_rms = metrics->ir_ac_rms; }

  metrics->red_pi_x1000 = 0U;
  metrics->ir_pi_x1000 = 0U;

  denominator = metrics->red_dc;
  if (denominator != 0ULL)
  {
    uint64_t red_pi = (((uint64_t)red_pi_rms * MAX30102_SPO2_RATIO_SCALE) +
                       (denominator / 2ULL)) / denominator;
    if (red_pi > 0xFFFFULL) { red_pi = 0xFFFFULL; }
    metrics->red_pi_x1000 = (uint16_t)red_pi;
  }

  denominator = metrics->ir_dc;
  if (denominator != 0ULL)
  {
    uint64_t ir_pi = (((uint64_t)ir_pi_rms * MAX30102_SPO2_RATIO_SCALE) +
                      (denominator / 2ULL)) / denominator;
    if (ir_pi > 0xFFFFULL) { ir_pi = 0xFFFFULL; }
    metrics->ir_pi_x1000 = (uint16_t)ir_pi;
  }

  return 1U;
}

uint8_t max30102_calculate_signal_quality(const MAX30102_SpO2_t *spo2_state,
                                          const MAX30102_SignalMetrics_t *metrics,
                                          uint8_t *signal_quality)
{
  uint32_t quality_score;
  uint32_t red_rms_for_quality;
  uint32_t ir_rms_for_quality;
  uint32_t red_centered_rms;
  uint32_t ir_centered_rms;
  uint32_t max_ac_rms;
  uint32_t min_ac_rms;
  uint16_t max_pi;
  uint16_t min_pi;

  if ((spo2_state == NULL) || (metrics == NULL) || (signal_quality == NULL))
  {
    return 0U;
  }

  red_centered_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                                     spo2_state->red_square_sum,
                                                     spo2_state->sample_count);
  ir_centered_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                                    spo2_state->ir_square_sum,
                                                    spo2_state->sample_count);
  red_rms_for_quality = (red_centered_rms > metrics->red_ac_rms) ?
                        red_centered_rms : metrics->red_ac_rms;
  ir_rms_for_quality = (ir_centered_rms > metrics->ir_ac_rms) ?
                       ir_centered_rms : metrics->ir_ac_rms;

  quality_score = 0U;
  quality_score += max30102_scale_score_u32(metrics->ir_pi_x1000, MAX30102_SIGNAL_QUALITY_IR_PI_GOOD, 30U);
  quality_score += max30102_scale_score_u32(metrics->red_pi_x1000, MAX30102_SIGNAL_QUALITY_RED_PI_GOOD, 20U);
  quality_score += max30102_scale_score_u32(ir_rms_for_quality, MAX30102_SIGNAL_QUALITY_IR_RMS_GOOD, 20U);
  quality_score += max30102_scale_score_u32(red_rms_for_quality, MAX30102_SIGNAL_QUALITY_RED_RMS_GOOD, 10U);
  quality_score += max30102_scale_score_u32(spo2_state->sample_count, MAX30102_SIGNAL_QUALITY_WINDOW_GOOD, 5U);

  max_pi = metrics->red_pi_x1000;
  min_pi = metrics->ir_pi_x1000;
  if (max_pi < min_pi) { max_pi = metrics->ir_pi_x1000; min_pi = metrics->red_pi_x1000; }

  if (max_pi != 0U)
  {
    quality_score += ((uint32_t)min_pi * 15U + (max_pi / 2U)) / max_pi;
  }
  else
  {
    max_ac_rms = red_rms_for_quality;
    min_ac_rms = ir_rms_for_quality;
    if (max_ac_rms < min_ac_rms) { max_ac_rms = ir_rms_for_quality; min_ac_rms = red_rms_for_quality; }

    if (max_ac_rms != 0U)
    {
      quality_score += ((uint32_t)min_ac_rms * 15U + (max_ac_rms / 2U)) / max_ac_rms;
    }
  }

  if (quality_score > 100U) { quality_score = 100U; }

  *signal_quality = (uint8_t)quality_score;
  return 1U;
}

/*
 * 根据窗口内 RED/IR 的 AC/DC 比值估算 SpO2。
 * 这里先求直流分量 DC，再求交流分量的 RMS，最后计算比值 R。
 * 与简单 max-min 相比，RMS 对偶发毛刺更不敏感，结果通常更稳。
 */
uint8_t max30102_calculate_spo2(const MAX30102_SpO2_t *spo2_state, uint8_t *spo2_value)
{
  uint16_t sample_count;
  uint32_t red_dc;
  uint32_t ir_dc;
  uint32_t red_ac_rms;
  uint32_t ir_ac_rms;
  uint32_t ratio_milli;
  int32_t spo2_milli;
  int64_t ratio_square_term;
  uint64_t denominator;

  if ((spo2_state == NULL) || (spo2_value == NULL))
  {
    return 0U;
  }

  if (spo2_state->sample_count < MAX30102_SPO2_MIN_VALID_SAMPLES)
  {
    return 0U;
  }

  sample_count = spo2_state->sample_count;
  red_dc = (uint32_t)(spo2_state->red_sum / sample_count);
  ir_dc = (uint32_t)(spo2_state->ir_sum / sample_count);
  red_ac_rms = max30102_calculate_centered_rms(spo2_state->red_sum,
                                               spo2_state->red_square_sum,
                                               sample_count);
  ir_ac_rms = max30102_calculate_centered_rms(spo2_state->ir_sum,
                                              spo2_state->ir_square_sum,
                                              sample_count);

  if ((red_dc <= MAX30102_SPO2_MIN_DC) ||
      (ir_dc <= MAX30102_SPO2_MIN_DC) ||
      (red_ac_rms < MAX30102_SPO2_MIN_AC_RMS) ||
      (ir_ac_rms < MAX30102_SPO2_MIN_AC_RMS))
  {
    return 0U;
  }

  /*
   * R = (ACred / DCred) / (ACir / DCir)
   * 这里用 RMS 作为 AC 幅值，比简单 max-min 更抗偶发毛刺。
   * 二次拟合系数来自 MAX3010x 系列常见开源示例，适合做演示版估算。
   */
  denominator = (uint64_t)ir_ac_rms * (uint64_t)red_dc;
  if (denominator == 0ULL)
  {
    return 0U;
  }

  ratio_milli = (uint32_t)((((uint64_t)red_ac_rms * (uint64_t)ir_dc * MAX30102_SPO2_RATIO_SCALE) +
                            (denominator / 2ULL)) / denominator);
  ratio_square_term = ((int64_t)ratio_milli * (int64_t)ratio_milli + 500LL) / 1000LL;
  spo2_milli = (int32_t)(94845LL + ((30354LL * (int64_t)ratio_milli + 500LL) / 1000LL) -
                         ((45060LL * ratio_square_term + 500LL) / 1000LL));

  if ((spo2_milli < 70000L) || (spo2_milli > 100000L))
  {
    return 0U;
  }

  *spo2_value = (uint8_t)((spo2_milli + 500L) / 1000L);
  return 1U;
}
