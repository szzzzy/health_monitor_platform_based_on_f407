/**
  ******************************************************************************
  * @file    app_ptt.c
  * @brief   PTT 计算：PPG 脉搏波峰时刻 − ECG R 峰时刻
  *
  * 维护最近 4 个 ECG R 峰时间戳；PPG 波峰到达时查找其之前最近的 R 峰。
  * 候选先通过 60–600 ms 的硬件匹配边界，再通过 80–350 ms 的输出可信范围、
  * ECG/PPG 双侧质量、运动/IBI、FIFO 时延和历史突跳门控。接受值写入 5 点历史
  * 并输出中位数。拒绝时只清 ptt_valid 并保留旧值；完整 reset 才清空历史和值。
  ******************************************************************************
  */

#include "app_ptt.h"
#include "app_ppg_sqi.h"
#include <string.h>

/* 先用宽边界排除明显的跨搏动错配，再用更窄范围门控可发布输出。 */
#define APP_PTT_MIN_MS                60U
#define APP_PTT_MAX_MS                600U
#define APP_PTT_TRUSTED_MIN_MS        80U
#define APP_PTT_TRUSTED_MAX_MS        350U
/* ECG/PPG 两侧都需要足够质量，PTT 才有意义。 */
#define APP_PTT_MIN_ECG_SQ            35U
#define APP_PTT_MIN_PPG_SQ            35U
#define APP_PTT_ECG_PEAK_HISTORY_SIZE 4U
#define APP_PTT_VALUE_HISTORY_SIZE    5U
/* PTT 突跳门控：绝对和相对阈值取较宽者，避免单次错配污染输出。 */
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

/* PTT 输出短历史：同时用于中位数平滑和突跳门控参考。 */
/* PPG 样本序号 → 绝对 ms 时间戳 */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples);

/* 在 ECG 历史中查找 PPG 波峰之前最近的有效 R 峰 */
static uint8_t app_ptt_find_ref_r_peak(uint32_t ppg_peak_ms, uint32_t *r_peak_ms);
static void app_ptt_add_value(uint16_t ptt_ms);
static uint16_t app_ptt_median_value(void);
static uint8_t app_ptt_jump_ok(uint16_t ptt_ms);
/* 拒绝 helper 统一清 ptt_valid 并递增原因计数，便于上位机统计失败来源。 */
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
    app->ptt_last_update_tick = 0UL;
    app->ptt_match_age_ms = 0xFFFFU;
    app->ptt_invalid_reason = APP_OUTPUT_REASON_NO_FINGER;
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
 * @note   要求 ECG/PPG 质量均不低于 35、无导联脱落和运动门控、IBI 有效，
 *         且 PPG 峰回推时延不超过 30 ms。候选必须同时落入宽匹配边界
 *         [60, 600] ms 与可信输出范围 [80, 350] ms，并通过历史突跳检查。
 *         失败只设置 ptt_valid=0，不清除上次 ptt_ms。
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
  ppg_age_ms = HAL_GetTick() - ppg_peak_ms;
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

  /* 第一层宽边界排除负时序、跨搏动或时间基准异常造成的明显错配。 */
  if ((ptt_ms < APP_PTT_MIN_MS) || (ptt_ms > APP_PTT_MAX_MS))
  {
    app_ptt_reject_range(app);
    return;
  }

  /* 第二层可信范围决定候选能否进入输出历史。 */
  if ((ptt_ms < APP_PTT_TRUSTED_MIN_MS) || (ptt_ms > APP_PTT_TRUSTED_MAX_MS))
  {
    app_ptt_reject_range(app);
    return;
  }

  ptt_candidate = (uint16_t)ptt_ms;
  /* 与短历史中位数相比突跳过大时拒绝，避免 ECG/PPG 单次错配进入输出历史。 */
  if (app_ptt_jump_ok(ptt_candidate) == 0U)
  {
    app_ptt_reject_jump(app);
    return;
  }

  app_ptt_add_value(ptt_candidate);
  app->ptt_ms    = app_ptt_median_value();
  app->ptt_valid = 1U;
  app->ptt_last_update_tick = HAL_GetTick();
  app->ptt_match_age_ms = (ppg_age_ms > 0xFFFFUL) ? 0xFFFFU : (uint16_t)ppg_age_ms;
  app->ptt_invalid_reason = APP_OUTPUT_REASON_OK;
}

/* ---- 将 PPG 样本索引转换为绝对毫秒时间戳 ---- */
static uint32_t app_ptt_ppg_sample_to_ms(const AppState_t *app,
                                         uint32_t peak_sample,
                                         uint32_t total_samples)
{
  uint32_t last_sample, delta_samples, delta_ms;

  if (app == NULL) return HAL_GetTick();

  last_sample = total_samples - 1U;
  if (peak_sample > last_sample) return app->sensor_last_sample_tick;

  delta_samples = last_sample - peak_sample;
  delta_ms = delta_samples * APP_SAMPLE_PERIOD_MS;

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

    if ((candidate != 0UL) &&
        ((int32_t)(ppg_peak_ms - candidate) >= 0))
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

/* 启动初期直接接受；历史足够后，用中位数 + 宽松限幅判断 PTT 是否突跳。 */
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
  app->ptt_invalid_reason = APP_OUTPUT_REASON_ECG;
  app_ptt_inc_u32(&app->ptt_reject_ecg_count);
}

static void app_ptt_reject_ppg(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app->ptt_invalid_reason = APP_OUTPUT_REASON_STALE;
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_LOW_PERFUSION) != 0U)
  {
    app->ptt_invalid_reason = APP_OUTPUT_REASON_LOW_PERFUSION;
  }
  else if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_MOTION) != 0U)
  {
    app->ptt_invalid_reason = APP_OUTPUT_REASON_MOTION;
  }
  else if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_TRANSITION) != 0U)
  {
    app->ptt_invalid_reason = APP_OUTPUT_REASON_CONTACT;
  }
  else if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_BEAT_UNSTABLE) != 0U)
  {
    app->ptt_invalid_reason = APP_OUTPUT_REASON_BEAT_UNSTABLE;
  }
  app_ptt_inc_u32(&app->ptt_reject_ppg_count);
}

static void app_ptt_reject_range(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app->ptt_invalid_reason = APP_OUTPUT_REASON_RANGE;
  app_ptt_inc_u32(&app->ptt_reject_range_count);
}

static void app_ptt_reject_jump(AppState_t *app)
{
  if (app == NULL) { return; }
  app->ptt_valid = 0U;
  app->ptt_invalid_reason = APP_OUTPUT_REASON_JUMP;
  app_ptt_inc_u32(&app->ptt_reject_jump_count);
}

static void app_ptt_inc_u32(uint32_t *value)
{
  if ((value != NULL) && (*value < 0xFFFFFFFFUL))
  {
    (*value)++;
  }
}
