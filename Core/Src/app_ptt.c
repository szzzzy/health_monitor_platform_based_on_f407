/**
  ******************************************************************************
  * @file    app_ptt.c
  * @brief   PTT 计算：PPG 脉搏波峰时刻 − ECG R 峰时刻
  *
  * 维护最近 4 个 ECG R 峰时间戳环形缓冲区。
  * PPG 波峰到达时查找其之前最近的 ECG R 峰，计算时间差。
  * PTT ∈ [60, 600] ms，越界结果标记无效并清零。
  * 导联脱落、手指离开、ECG/PPG 复位时清空 PTT 状态。
  ******************************************************************************
  */

#include "app_ptt.h"
#include "app_ppg_sqi.h"
#include <string.h>

#define APP_PTT_MIN_MS                60U
#define APP_PTT_MAX_MS                600U
#define APP_PTT_TRUSTED_MIN_MS        80U
#define APP_PTT_TRUSTED_MAX_MS        350U
#define APP_PTT_MIN_ECG_SQ            35U
#define APP_PTT_MIN_PPG_SQ            35U
#define APP_PTT_ECG_PEAK_HISTORY_SIZE 4U
#define APP_PTT_VALUE_HISTORY_SIZE    5U
#define APP_PTT_MAX_JUMP_MS           80U
#define APP_PTT_MAX_JUMP_PERCENT      35U

static struct
{
  uint32_t r_peak_ms[APP_PTT_ECG_PEAK_HISTORY_SIZE];
  uint8_t  count;
  uint8_t  write_index;
} ptt_ecg_state;

static struct
{
  uint16_t value_ms[APP_PTT_VALUE_HISTORY_SIZE];
  uint8_t count;
  uint8_t write_index;
} ptt_value_state;

/* PPG 样本序号 → 绝对 ms 时间戳 */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples);

/* 在 ECG 历史中查找 PPG 波峰之前最近的有效 R 峰 */
static uint8_t app_ptt_find_ref_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms);
static void app_ptt_add_value(uint16_t ptt_ms);
static uint16_t app_ptt_median_value(void);
static uint8_t app_ptt_jump_ok(uint16_t ptt_ms);
static void app_ptt_reject_ecg(AppState_t *app);
static void app_ptt_reject_ppg(AppState_t *app);
static void app_ptt_reject_range(AppState_t *app);
static void app_ptt_reject_jump(AppState_t *app);
static void app_ptt_inc_u32(uint32_t *value);

/**
 ******************************************************************************
 * @brief  重置 PTT 模块：清除 ECG R 峰历史和 AppState 字段。
 * @param  app AppState 指针（可为 NULL）。
 * @note   手指离开、导联脱落或 ECG/PPG 重置时调用。
 ******************************************************************************
 */
void app_ptt_reset(AppState_t *app)
{
  (void)memset(&ptt_ecg_state, 0, sizeof(ptt_ecg_state));
  (void)memset(&ptt_value_state, 0, sizeof(ptt_value_state));
  if (app != NULL)
  {
    app->ptt_valid = 0U;
    app->ptt_ms    = 0U;
  }
}

/**
 ******************************************************************************
 * @brief  记录 ECG R 峰时间戳用于 PTT 匹配窗口。
 * @param  r_peak_ms  检测到的 R 峰的绝对时间戳（ms）。
 * @note   写入 4 条目环形缓冲；零时间戳被忽略。
 ******************************************************************************
 */
void app_ptt_add_ecg_peak(uint32_t r_peak_ms)
{
  if (r_peak_ms == 0UL) return;

  ptt_ecg_state.r_peak_ms[ptt_ecg_state.write_index] = r_peak_ms;
  ptt_ecg_state.write_index = (uint8_t)((ptt_ecg_state.write_index + 1U) %
                                        APP_PTT_ECG_PEAK_HISTORY_SIZE);
  if (ptt_ecg_state.count < APP_PTT_ECG_PEAK_HISTORY_SIZE)
  {
    ptt_ecg_state.count++;
  }
}

/**
 ******************************************************************************
 * @brief  计算 PTT：查找最近的前驱 ECG R 峰并测量差值。
 * @param  app               AppState 指针。
 * @param  ppg_peak_sample   PPG 峰值样本索引。
 * @param  ppg_total_samples 自启动以来的 PPG 样本总数。
 * @note   前置条件：ecg_valid=1、无导联脱落、ecg_r_peak_ms 非零。
 *         应用 FIFO 积压时限检查（<=30 ms）。结果钳位于
 *         [60, 600] ms。任何失败则设置 ptt_valid=0。
 ******************************************************************************
 */
void app_ptt_update_from_ppg_peak(AppState_t *app,
                                  uint32_t ppg_peak_sample,
                                  uint32_t ppg_total_samples)
{
  uint32_t ppg_peak_ms, r_peak_ms, ptt_ms, ppg_age_ms;
  uint16_t ptt_candidate;

  if (app == NULL) return;

  /* 前置条件 */
  if ((app->ecg_valid == 0U) ||
      (app->ecg_lead_off != 0U) ||
      (app->ecg_r_peak_ms == 0UL) ||
      (app->ecg_signal_quality < APP_PTT_MIN_ECG_SQ) ||
      (app->ecg_invalid_reason == ECG_INVALID_ADC_SAT) ||
      (app->ecg_invalid_reason == ECG_INVALID_DMA_OVERFLOW))
  {
    app_ptt_reject_ecg(app);
    return;
  }

  if ((app->signal_quality < APP_PTT_MIN_PPG_SQ) ||
      (app_ppg_sqi_allows_ptt(app) == 0U) ||
      (app->motion_artifact != 0U) ||
      (app->ibi_valid == 0U))
  {
    app_ptt_reject_ppg(app);
    return;
  }

  ppg_peak_ms = app_ptt_ppg_sample_to_ms(app, ppg_peak_sample, ppg_total_samples);

  /*
   * 时间基准可靠性检查：
   * PPG 样本在 FIFO 积压后批量补读时，sensor_last_sample_tick 是
   * 消费时刻而非真实采样时刻，导致 ppg_peak_ms 推算值偏晚。
   * 若推算的峰值延迟 > 30 ms，说明存在 FIFO 积压延迟，PTT 不可信。
   */
  ppg_age_ms = (HAL_GetTick() >= ppg_peak_ms) ? (HAL_GetTick() - ppg_peak_ms) : 0U;
  if (ppg_age_ms > 30U)
  {
    app_ptt_reject_ppg(app);
    return;
  }

  if (app_ptt_find_ref_r_peak(ppg_peak_ms, &r_peak_ms) == 0U)
  {
    app_ptt_reject_ecg(app);
    return;
  }

  ptt_ms = ppg_peak_ms - r_peak_ms;

  /* 越界检查：PTT 必须在 [MIN, MAX] ms 内 */
  if ((ptt_ms < APP_PTT_MIN_MS) || (ptt_ms > APP_PTT_MAX_MS))
  {
    app_ptt_reject_range(app);
    return;
  }

  if ((ptt_ms < APP_PTT_TRUSTED_MIN_MS) || (ptt_ms > APP_PTT_TRUSTED_MAX_MS))
  {
    app_ptt_reject_range(app);
    return;
  }

  ptt_candidate = (uint16_t)ptt_ms;
  if (app_ptt_jump_ok(ptt_candidate) == 0U)
  {
    app_ptt_reject_jump(app);
    return;
  }

  app_ptt_add_value(ptt_candidate);
  app->ptt_ms    = app_ptt_median_value();
  app->ptt_valid = 1U;
}

/* ---- 将 PPG 样本索引转换为绝对毫秒时间戳 ---- */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples)
{
  uint32_t last_sample, delta_samples, delta_ms;

  if ((app == NULL) || (total_samples == 0U)) return HAL_GetTick();

  last_sample = total_samples - 1U;
  if (peak_sample > last_sample) return app->sensor_last_sample_tick;

  delta_samples = last_sample - peak_sample;
  delta_ms = delta_samples * APP_SAMPLE_PERIOD_MS;

  if (delta_ms > app->sensor_last_sample_tick) return 0UL;

  return app->sensor_last_sample_tick - delta_ms;
}

/* ---- 查找 PPG 峰值之前最近的 ECG R 峰 ---- */
static uint8_t app_ptt_find_ref_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms)
{
  uint8_t  i, index;
  uint32_t candidate;

  if ((r_peak_ms == NULL) || (ptt_ecg_state.count == 0U)) return 0U;

  for (i = 0U; i < ptt_ecg_state.count; i++)
  {
    index = (uint8_t)((ptt_ecg_state.write_index + APP_PTT_ECG_PEAK_HISTORY_SIZE - 1U - i) %
                      APP_PTT_ECG_PEAK_HISTORY_SIZE);
    candidate = ptt_ecg_state.r_peak_ms[index];

    if ((candidate != 0UL) && (candidate <= ppg_peak_ms))
    {
      *r_peak_ms = candidate;
      return 1U;
    }
  }
  return 0U;
}

static void app_ptt_add_value(uint16_t ptt_ms)
{
  ptt_value_state.value_ms[ptt_value_state.write_index] = ptt_ms;
  ptt_value_state.write_index = (uint8_t)((ptt_value_state.write_index + 1U) %
                                          APP_PTT_VALUE_HISTORY_SIZE);
  if (ptt_value_state.count < APP_PTT_VALUE_HISTORY_SIZE)
  {
    ptt_value_state.count++;
  }
}

static uint16_t app_ptt_median_value(void)
{
  uint16_t sorted[APP_PTT_VALUE_HISTORY_SIZE];
  uint16_t tmp;
  uint8_t i;
  uint8_t j;
  uint8_t n;

  n = ptt_value_state.count;
  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < n; i++)
  {
    sorted[i] = ptt_value_state.value_ms[i];
  }

  for (i = 1U; i < n; i++)
  {
    tmp = sorted[i];
    j = i;
    while ((j > 0U) && (sorted[j - 1U] > tmp))
    {
      sorted[j] = sorted[j - 1U];
      j--;
    }
    sorted[j] = tmp;
  }

  return sorted[n / 2U];
}

static uint8_t app_ptt_jump_ok(uint16_t ptt_ms)
{
  uint16_t median;
  uint32_t diff;
  uint32_t limit;

  if (ptt_value_state.count < 3U)
  {
    return 1U;
  }

  median = app_ptt_median_value();
  if (median == 0U)
  {
    return 1U;
  }

  diff = (ptt_ms >= median) ? ((uint32_t)ptt_ms - (uint32_t)median) :
                              ((uint32_t)median - (uint32_t)ptt_ms);
  limit = ((uint32_t)median * APP_PTT_MAX_JUMP_PERCENT) / 100U;
  if (limit < APP_PTT_MAX_JUMP_MS)
  {
    limit = APP_PTT_MAX_JUMP_MS;
  }

  return (diff <= limit) ? 1U : 0U;
}

static void app_ptt_reject_ecg(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app_ptt_inc_u32(&app->ptt_reject_ecg_count);
}

static void app_ptt_reject_ppg(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app_ptt_inc_u32(&app->ptt_reject_ppg_count);
}

static void app_ptt_reject_range(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app_ptt_inc_u32(&app->ptt_reject_range_count);
}

static void app_ptt_reject_jump(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app_ptt_inc_u32(&app->ptt_reject_jump_count);
}

static void app_ptt_inc_u32(uint32_t *value)
{
  if ((value != NULL) && (*value < 0xFFFFFFFFUL))
  {
    (*value)++;
  }
}
