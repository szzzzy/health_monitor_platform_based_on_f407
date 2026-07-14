/**
  ******************************************************************************
  * @file    app_ppg_side_elgendi.c
  * @brief   Elgendi 风格 PPG 侧路检测器，用于 A/B 诊断。
  *
  * 本实现参考 Elgendi 双移动平均思路，但刻意保持为低成本侧路检测器：
  *   1. 对滤波后 IR AC 样本取绝对能量；
  *   2. 用短窗 MA 表征峰能量，用长窗 MA 表征 beat 背景；
  *   3. 峰窗高于动态阈值时形成 block，并取 block 内最大绝对能量点作为侧路峰；
  *   4. 侧路峰只和生产检测器峰做匹配/漏检/误检统计。
  *
  * 该模块不写 BPM/SpO2/PTT 正式输出，因此适合面包板静态场景下解释
  * 手指覆盖不足、瞬态接触变化和异常峰检测，而不改变原有测量路径。
  ******************************************************************************
  */

#include "app_ppg_side_elgendi.h"

#include <string.h>

#if (APP_PPG_SIDE_ELGENDI != 0U)

/* 双移动平均参数：短窗看局部峰，长窗看一拍尺度背景。 */
#define APP_SIDE_MA_PEAK_SAMPLES       11U  /* 约 110 ms，峰宽窗口 */
#define APP_SIDE_MA_BEAT_SAMPLES       67U  /* 约 670 ms，一拍背景窗口 */
/* block/IBI 约束：防止窄尖峰、重复峰和不合理心率进入 A/B 统计。 */
#define APP_SIDE_MIN_BLOCK_SAMPLES      8U  /* 约 80 ms，收缩峰宽下限 */
#define APP_SIDE_REFRACTORY_SAMPLES    30U  /* 约 300 ms，不应期 */
#define APP_SIDE_IBI_MIN_SAMPLES       30U
#define APP_SIDE_IBI_MAX_SAMPLES      200U
/* 与生产检测器的匹配窗口；窗口内认为两者检测到同一搏动。 */
#define APP_SIDE_MATCH_WINDOW_SAMPLES  15U  /* +/-150 ms */
/* 动态阈值 = beat_ma + beat_ma/8，并设置最小 offset 避免零能量抖动。 */
#define APP_SIDE_OFFSET_DIV             8U
#define APP_SIDE_OFFSET_MIN             2U
#define APP_SIDE_ENERGY_CAP         65535UL

/* 侧路检测器私有状态：固定数组 + O(1) 环形更新，适合 100 Hz 实时调用。 */
typedef struct
{
  uint32_t peak_ring[APP_SIDE_MA_PEAK_SAMPLES];
  uint32_t beat_ring[APP_SIDE_MA_BEAT_SAMPLES];
  uint32_t peak_sum;
  uint32_t beat_sum;
  uint8_t peak_index;
  uint8_t beat_index;
  uint8_t peak_count;
  uint8_t beat_count;
  uint8_t in_block;
  uint16_t block_len;
  uint32_t block_start_sample;
  uint32_t block_max_energy;
  uint32_t block_peak_sample;
  uint32_t last_side_peak_sample;
  uint8_t pending_side_valid;
  uint32_t pending_side_sample;
  uint8_t pending_current_valid;
  uint32_t pending_current_sample;
} AppPpgSideState_t;

static AppPpgSideState_t side_state;

/* === 内部辅助函数 ========================================================== */
static uint32_t app_side_abs_i32(int32_t value);
static void app_side_push_energy(uint32_t energy);
static uint8_t app_side_ready(void);
static void app_side_accept_peak(AppState_t *app,
                                 uint32_t peak_sample,
                                 uint16_t block_len);
static void app_side_try_match(AppState_t *app);
static void app_side_expire_pending(AppState_t *app, uint32_t latest_sample);
static uint32_t app_side_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static void app_side_inc_u32(uint32_t *value);
static uint16_t app_side_samples_to_ms(uint32_t samples);

#endif

/**
 *******************************************************************************
 * @brief  重置侧路检测器内部环形窗口、block 状态和诊断计数。
 * @param  app AppState 指针（可为 NULL）。
 * @note   NULL 用于只清内部静态状态；非 NULL 时同步清 ppg_side_* 输出字段。
 *******************************************************************************
 */
void app_ppg_side_elgendi_reset(AppState_t *app)
{
#if (APP_PPG_SIDE_ELGENDI != 0U)
  (void)memset(&side_state, 0, sizeof(side_state));

  if (app == NULL)
  {
    return;
  }

  app->ppg_side_peak_count = 0UL;
  app->ppg_side_current_peak_count = 0UL;
  app->ppg_side_match_count = 0UL;
  app->ppg_side_missed_current_count = 0UL;
  app->ppg_side_unmatched_count = 0UL;
  app->ppg_side_reject_short_count = 0UL;
  app->ppg_side_reject_refractory_count = 0UL;
  app->ppg_side_reject_range_count = 0UL;
  app->ppg_side_last_ibi_ms = 0U;
  app->ppg_side_last_hr = 0U;
  app->ppg_side_last_delta_ms = 0;
  app->ppg_side_last_block_ms = 0U;
#else
  (void)app;
#endif
}

/**
 *******************************************************************************
 * @brief  处理一个滤波后 IR 样本，必要时形成侧路候选峰。
 * @param  app             AppState 指针。
 * @param  filtered_sample 带通滤波后的 IR AC 样本。
 * @param  total_samples   当前样本编号。
 * @note   算法只在手指就位且 contact_settle_samples==0 时运行。
 *         低于最小 block 宽度的片段会计入 short reject。
 *******************************************************************************
 */
void app_ppg_side_elgendi_update(AppState_t *app,
                                 int32_t filtered_sample,
                                 uint32_t current_sample)
{
#if (APP_PPG_SIDE_ELGENDI != 0U)
  uint32_t energy;
  uint32_t peak_ma;
  uint32_t beat_ma;
  uint32_t threshold;
  uint8_t above;

  if (app == NULL)
  {
    return;
  }

  if ((app->finger_present == 0U) || (app->contact_settle_samples > 0U))
  {
    app_ppg_side_elgendi_reset(app);
    return;
  }

  energy = app_side_abs_i32(filtered_sample);
  if (energy > APP_SIDE_ENERGY_CAP)
  {
    energy = APP_SIDE_ENERGY_CAP;
  }

  app_side_push_energy(energy);
  if (app_side_ready() == 0U)
  {
    return;
  }

  peak_ma = side_state.peak_sum / APP_SIDE_MA_PEAK_SAMPLES;
  beat_ma = side_state.beat_sum / APP_SIDE_MA_BEAT_SAMPLES;
  threshold = beat_ma + (beat_ma / APP_SIDE_OFFSET_DIV);
  if (threshold < (beat_ma + APP_SIDE_OFFSET_MIN))
  {
    threshold = beat_ma + APP_SIDE_OFFSET_MIN;
  }

  above = (peak_ma > threshold) ? 1U : 0U;

  if (above != 0U)
  {
    if (side_state.in_block == 0U)
    {
      side_state.in_block = 1U;
      side_state.block_len = 0U;
      side_state.block_start_sample = current_sample;
      side_state.block_max_energy = energy;
      side_state.block_peak_sample = current_sample;
    }

    if (side_state.block_len < 0xFFFFU)
    {
      side_state.block_len++;
    }
    if (energy > side_state.block_max_energy)
    {
      side_state.block_max_energy = energy;
      side_state.block_peak_sample = current_sample;
    }
  }
  else if (side_state.in_block != 0U)
  {
    if (side_state.block_len >= APP_SIDE_MIN_BLOCK_SAMPLES)
    {
      app_side_accept_peak(app,
                           side_state.block_peak_sample,
                           side_state.block_len);
    }
    else
    {
      app_side_inc_u32(&app->ppg_side_reject_short_count);
    }

    side_state.in_block = 0U;
    side_state.block_len = 0U;
  }

  app_side_expire_pending(app, current_sample);
#else
  (void)app;
  (void)filtered_sample;
  (void)current_sample;
#endif
}

/**
 *******************************************************************************
 * @brief  记录生产检测器已接受的 PPG 峰，并尝试和侧路峰匹配。
 * @param  app         AppState 指针。
 * @param  peak_sample 主检测器接受峰的样本编号。
 * @param  ibi_ms      主检测器 IBI（保留给未来扩展；当前统计不使用）。
 * @note   若上一个主峰在窗口内没有被侧路峰匹配，会计入 missed_current。
 *******************************************************************************
 */
void app_ppg_side_elgendi_note_current_peak(AppState_t *app,
                                            uint32_t peak_sample,
                                            uint16_t ibi_ms)
{
#if (APP_PPG_SIDE_ELGENDI != 0U)
  (void)ibi_ms;

  if ((app == NULL) || (peak_sample == 0UL))
  {
    return;
  }

  app_side_inc_u32(&app->ppg_side_current_peak_count);

  if (side_state.pending_current_valid != 0U)
  {
    app_side_inc_u32(&app->ppg_side_missed_current_count);
  }

  side_state.pending_current_valid = 1U;
  side_state.pending_current_sample = peak_sample;
  app_side_try_match(app);
  app_side_expire_pending(app, peak_sample);
#else
  (void)app;
  (void)peak_sample;
  (void)ibi_ms;
#endif
}

#if (APP_PPG_SIDE_ELGENDI != 0U)

/* ---- int32 绝对值：避开 INT32_MIN 直接取负溢出 ---- */
static uint32_t app_side_abs_i32(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-(value + 1)) + 1UL;
  }

  return (uint32_t)value;
}

/* ---- O(1) 更新短窗/长窗能量移动平均 ---- */
static void app_side_push_energy(uint32_t energy)
{
  if (side_state.peak_count >= APP_SIDE_MA_PEAK_SAMPLES)
  {
    side_state.peak_sum -= side_state.peak_ring[side_state.peak_index];
  }
  else
  {
    side_state.peak_count++;
  }
  side_state.peak_ring[side_state.peak_index] = energy;
  side_state.peak_sum += energy;
  side_state.peak_index = (uint8_t)((side_state.peak_index + 1U) %
                                    APP_SIDE_MA_PEAK_SAMPLES);

  if (side_state.beat_count >= APP_SIDE_MA_BEAT_SAMPLES)
  {
    side_state.beat_sum -= side_state.beat_ring[side_state.beat_index];
  }
  else
  {
    side_state.beat_count++;
  }
  side_state.beat_ring[side_state.beat_index] = energy;
  side_state.beat_sum += energy;
  side_state.beat_index = (uint8_t)((side_state.beat_index + 1U) %
                                    APP_SIDE_MA_BEAT_SAMPLES);
}

/* ---- 两个移动平均窗口都填满后才允许输出侧路候选峰 ---- */
static uint8_t app_side_ready(void)
{
  return ((side_state.peak_count >= APP_SIDE_MA_PEAK_SAMPLES) &&
          (side_state.beat_count >= APP_SIDE_MA_BEAT_SAMPLES)) ? 1U : 0U;
}

/* ---- 接受一个侧路 block 峰，并更新 IBI/HR 与 pending 匹配状态 ---- */
static void app_side_accept_peak(AppState_t *app,
                                 uint32_t peak_sample,
                                 uint16_t block_len)
{
  uint32_t ibi_samples;
  uint16_t ibi_ms;
  uint32_t hr;

  if ((app == NULL) || (peak_sample == 0UL))
  {
    return;
  }

  if (side_state.last_side_peak_sample != 0UL)
  {
    if (peak_sample <= side_state.last_side_peak_sample)
    {
      app_side_inc_u32(&app->ppg_side_reject_refractory_count);
      return;
    }

    ibi_samples = peak_sample - side_state.last_side_peak_sample;
    if (ibi_samples < APP_SIDE_REFRACTORY_SAMPLES)
    {
      app_side_inc_u32(&app->ppg_side_reject_refractory_count);
      return;
    }
    if ((ibi_samples < APP_SIDE_IBI_MIN_SAMPLES) ||
        (ibi_samples > APP_SIDE_IBI_MAX_SAMPLES))
    {
      app_side_inc_u32(&app->ppg_side_reject_range_count);
      return;
    }

    ibi_ms = app_side_samples_to_ms(ibi_samples);
    app->ppg_side_last_ibi_ms = ibi_ms;
    if (ibi_ms != 0U)
    {
      hr = (60000UL + ((uint32_t)ibi_ms / 2UL)) / (uint32_t)ibi_ms;
      app->ppg_side_last_hr = (hr > 255UL) ? 255U : (uint8_t)hr;
    }
  }

  side_state.last_side_peak_sample = peak_sample;
  app->ppg_side_last_block_ms = app_side_samples_to_ms(block_len);
  app_side_inc_u32(&app->ppg_side_peak_count);

  if (side_state.pending_side_valid != 0U)
  {
    app_side_inc_u32(&app->ppg_side_unmatched_count);
  }
  side_state.pending_side_valid = 1U;
  side_state.pending_side_sample = peak_sample;
  app_side_try_match(app);
}

/* ---- 若侧路峰与主检测器峰落在同一匹配窗口，记录 delta 并清 pending ---- */
static void app_side_try_match(AppState_t *app)
{
  uint32_t diff;
  int32_t delta_samples;
  int32_t delta_ms;

  if ((app == NULL) ||
      (side_state.pending_side_valid == 0U) ||
      (side_state.pending_current_valid == 0U))
  {
    return;
  }

  diff = app_side_abs_diff_u32(side_state.pending_side_sample,
                               side_state.pending_current_sample);
  if (diff > APP_SIDE_MATCH_WINDOW_SAMPLES)
  {
    return;
  }

  delta_samples = (int32_t)side_state.pending_side_sample -
                  (int32_t)side_state.pending_current_sample;
  delta_ms = delta_samples * (int32_t)APP_SAMPLE_PERIOD_MS;
  if (delta_ms > 32767L)
  {
    delta_ms = 32767L;
  }
  else if (delta_ms < -32768L)
  {
    delta_ms = -32768L;
  }

  app->ppg_side_last_delta_ms = (int16_t)delta_ms;
  app_side_inc_u32(&app->ppg_side_match_count);
  side_state.pending_side_valid = 0U;
  side_state.pending_current_valid = 0U;
}

/* ---- 超过匹配窗口仍未配对的 pending 峰转为 unmatched/missed 计数 ---- */
static void app_side_expire_pending(AppState_t *app, uint32_t latest_sample)
{
  if (app == NULL)
  {
    return;
  }

  if ((side_state.pending_side_valid != 0U) &&
      (latest_sample > (side_state.pending_side_sample + APP_SIDE_MATCH_WINDOW_SAMPLES)))
  {
    app_side_inc_u32(&app->ppg_side_unmatched_count);
    side_state.pending_side_valid = 0U;
  }

  if ((side_state.pending_current_valid != 0U) &&
      (latest_sample > (side_state.pending_current_sample + APP_SIDE_MATCH_WINDOW_SAMPLES)))
  {
    app_side_inc_u32(&app->ppg_side_missed_current_count);
    side_state.pending_current_valid = 0U;
  }
}

/* ---- uint32 绝对差 ---- */
static uint32_t app_side_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  return (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
}

/* ---- 饱和递增计数器，避免长时间运行后回绕误导日志 ---- */
static void app_side_inc_u32(uint32_t *value)
{
  if ((value != NULL) && (*value < 0xFFFFFFFFUL))
  {
    (*value)++;
  }
}

/* ---- 样本数换算为 ms，并钳位到 uint16 ---- */
static uint16_t app_side_samples_to_ms(uint32_t samples)
{
  uint32_t ms = samples * APP_SAMPLE_PERIOD_MS;
  return (ms > 0xFFFFUL) ? 0xFFFFU : (uint16_t)ms;
}

#endif
