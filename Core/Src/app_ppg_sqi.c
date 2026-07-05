/**
  ******************************************************************************
  * @file    app_ppg_sqi.c
  * @brief   PPG SQI 实验门控 / Experimental PPG signal-quality gate
  *
  * 本模块把已有的窗口质量分、PI/AC 强度、RED/IR 平衡、运动标志、接触稳定期
  * 与 beat-to-beat 稳定性合成为一个保守 SQI。SQI 只向下钳制输出置信度，
  * 不改变原始采样、滤波或 PPG 峰值检测状态机。
  *
  * The SQI is intentionally a side gate: it can suppress HR/SpO2/PTT valid flags
  * when evidence is weak, while keeping the original detector path observable for
  * comparison in logs.
  ******************************************************************************
  */

#include "app_ppg_sqi.h"

#include <string.h>

/* ---- SQI 调参常量 / SQI tuning constants ---- */
/* 自适应基线慢速跟随：仅在无质量标志时更新，避免把坏窗口学习成正常基线。 */
#define APP_PPG_SQI_BASE_SHIFT              6U
#define APP_PPG_SQI_BASE_MIN_QUALITY        35U
/* 低灌注判据：同时保留相对跌落和绝对下限，兼顾个体差异与极弱信号。 */
#define APP_PPG_SQI_LOW_PERF_PERCENT        45U
#define APP_PPG_SQI_LOW_PERF_ABS_AC         3U
#define APP_PPG_SQI_LOW_PERF_ABS_PI         1U
/* 不同异常对应的最高可用质量分；SQI 只降分，不主动升分。 */
#define APP_PPG_SQI_LOW_PERF_CAP            24U
#define APP_PPG_SQI_MOTION_CAP              20U
#define APP_PPG_SQI_TRANSITION_CAP          15U
#define APP_PPG_SQI_BALANCE_CAP             45U
#define APP_PPG_SQI_BEAT_UNSTABLE_CAP       35U
/* HR/SpO2/PTT 使用不同门限：PTT 对峰时刻更敏感，因此门限最高。 */
#define APP_PPG_SQI_HR_MIN_SCORE            30U
#define APP_PPG_SQI_SPO2_MIN_SCORE          35U
#define APP_PPG_SQI_PTT_MIN_SCORE           45U
/* beat-to-beat 稳定性：短历史中位数用于拒绝突跳 IBI 和异常幅度。 */
#define APP_PPG_SQI_IBI_HISTORY_SIZE        5U
#define APP_PPG_SQI_IBI_JUMP_PERCENT        35U
#define APP_PPG_SQI_AMP_DROP_PERCENT        40U
#define APP_PPG_SQI_AMP_SPIKE_PERCENT       260U
#define APP_PPG_SQI_BEAT_HOLD_SAMPLES       50U

/* SQI 内部状态：不进 AppState，避免串口协议被临时基线细节污染。 */
typedef struct
{
  uint8_t  initialized;
  uint32_t ir_ac_baseline;
  uint32_t red_ac_baseline;
  uint16_t ir_pi_baseline;
  uint16_t red_pi_baseline;
  uint16_t ibi_history[APP_PPG_SQI_IBI_HISTORY_SIZE];
  uint8_t  ibi_count;
  uint8_t  ibi_write_index;
  uint32_t amp_ema;
  uint8_t  amp_ema_valid;
  uint8_t  beat_unstable_hold;
} AppPpgSqiState_t;

static AppPpgSqiState_t ppg_sqi_state;

/* ---- 内部工具函数 / Internal helpers ---- */
static uint32_t app_ppg_sqi_slow_follow_u32(uint32_t current,
                                            uint32_t target,
                                            uint8_t shift);
static uint16_t app_ppg_sqi_slow_follow_u16(uint16_t current,
                                            uint16_t target,
                                            uint8_t shift);
static uint8_t app_ppg_sqi_below_percent_u32(uint32_t current,
                                             uint32_t baseline,
                                             uint32_t percent);
static uint8_t app_ppg_sqi_below_percent_u16(uint16_t current,
                                             uint16_t baseline,
                                             uint32_t percent);
static void app_ppg_sqi_inc_u32(uint32_t *value);
static void app_ppg_sqi_cap_score(uint8_t *score, uint8_t cap);
static void app_ppg_sqi_update_baseline(const MAX30102_SignalMetrics_t *metrics);
static uint16_t app_ppg_sqi_median_ibi(void);
static void app_ppg_sqi_add_ibi(uint16_t ibi_ms);
static uint32_t app_ppg_sqi_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static void app_ppg_sqi_mark_beat_unstable(AppState_t *app, uint8_t ibi_reject);

/**
 ******************************************************************************
 * @brief  重置 SQI 运行状态和 AppState 中的 SQI 诊断计数。
 * @param  app AppState 指针；为 NULL 时只清内部自适应状态。
 * @note   手指状态切换、FIFO 溢出和测量链路重置时调用，防止旧基线污染新佩戴。
 ******************************************************************************
 */
void app_ppg_sqi_reset(AppState_t *app)
{
  (void)memset(&ppg_sqi_state, 0, sizeof(ppg_sqi_state));

  if (app == NULL)
  {
    return;
  }

  app->ppg_sqi_score = 0U;
  app->ppg_sqi_flags = 0U;
  app->ppg_sqi_low_perfusion_count = 0UL;
  app->ppg_sqi_motion_count = 0UL;
  app->ppg_sqi_balance_count = 0UL;
  app->ppg_sqi_transition_count = 0UL;
  app->ppg_sqi_ibi_reject_count = 0UL;
  app->ppg_sqi_amp_reject_count = 0UL;
  app->ppg_sqi_suppressed_count = 0UL;
}

/**
 ******************************************************************************
 * @brief  基于当前 PPG 窗口更新 SQI score/flags。
 * @param  app         AppState 指针。
 * @param  metrics     RED/IR AC、PI 和平衡状态输入；NULL 表示无有效 PPG 窗口。
 * @param  raw_quality 原始 PPG 窗口质量分，作为 SQI 的上限输入。
 * @note   先标记接触稳定期和运动，再根据本地基线判断低灌注；仅在无标志窗口
 *         中慢速更新基线。This function never raises raw_quality.
 ******************************************************************************
 */
void app_ppg_sqi_update_window(AppState_t *app,
                               const MAX30102_SignalMetrics_t *metrics,
                               uint8_t raw_quality)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U)
  uint8_t score;
  uint8_t flags = 0U;
  uint8_t low_perfusion = 0U;
  uint8_t balance_bad = 0U;

  if (app == NULL)
  {
    return;
  }

  if ((metrics == NULL) || (app->finger_present == 0U))
  {
    app->ppg_sqi_score = 0U;
    app->ppg_sqi_flags = 0U;
    return;
  }

  score = raw_quality;

  if (app->contact_settle_samples > 0U)
  {
    flags |= APP_PPG_SQI_FLAG_TRANSITION;
    app_ppg_sqi_cap_score(&score, APP_PPG_SQI_TRANSITION_CAP);
    app_ppg_sqi_inc_u32(&app->ppg_sqi_transition_count);
  }

  if (app->motion_artifact != 0U)
  {
    flags |= APP_PPG_SQI_FLAG_MOTION;
    app_ppg_sqi_cap_score(&score, APP_PPG_SQI_MOTION_CAP);
    app_ppg_sqi_inc_u32(&app->ppg_sqi_motion_count);
  }

  if (ppg_sqi_state.initialized != 0U)
  {
    if ((metrics->ir_ac_rms <= APP_PPG_SQI_LOW_PERF_ABS_AC) ||
        (metrics->ir_pi_x1000 <= APP_PPG_SQI_LOW_PERF_ABS_PI) ||
        app_ppg_sqi_below_percent_u32(metrics->ir_ac_rms,
                                      ppg_sqi_state.ir_ac_baseline,
                                      APP_PPG_SQI_LOW_PERF_PERCENT) ||
        app_ppg_sqi_below_percent_u16(metrics->ir_pi_x1000,
                                      ppg_sqi_state.ir_pi_baseline,
                                      APP_PPG_SQI_LOW_PERF_PERCENT))
    {
      low_perfusion = 1U;
    }
  }
  else if ((raw_quality >= APP_PPG_SQI_BASE_MIN_QUALITY) &&
           (app->contact_settle_samples == 0U) &&
           (app->motion_artifact == 0U))
  {
    ppg_sqi_state.ir_ac_baseline = metrics->ir_ac_rms;
    ppg_sqi_state.red_ac_baseline = metrics->red_ac_rms;
    ppg_sqi_state.ir_pi_baseline = metrics->ir_pi_x1000;
    ppg_sqi_state.red_pi_baseline = metrics->red_pi_x1000;
    ppg_sqi_state.initialized = 1U;
  }

  if (low_perfusion != 0U)
  {
    flags |= APP_PPG_SQI_FLAG_LOW_PERFUSION;
    app_ppg_sqi_cap_score(&score, APP_PPG_SQI_LOW_PERF_CAP);
    app_ppg_sqi_inc_u32(&app->ppg_sqi_low_perfusion_count);
  }

  if ((app->spo2_ratio_valid == 0U) ||
      (app->spo2_balance_status == APP_OXY_BALANCE_LOW) ||
      (app->spo2_balance_status == APP_OXY_BALANCE_HIGH))
  {
    balance_bad = 1U;
  }

  if (balance_bad != 0U)
  {
    flags |= APP_PPG_SQI_FLAG_BALANCE;
    app_ppg_sqi_cap_score(&score, APP_PPG_SQI_BALANCE_CAP);
    app_ppg_sqi_inc_u32(&app->ppg_sqi_balance_count);
  }

  if (ppg_sqi_state.beat_unstable_hold > 0U)
  {
    flags |= APP_PPG_SQI_FLAG_BEAT_UNSTABLE;
    app_ppg_sqi_cap_score(&score, APP_PPG_SQI_BEAT_UNSTABLE_CAP);
    ppg_sqi_state.beat_unstable_hold--;
  }

  if (score < raw_quality)
  {
    app_ppg_sqi_inc_u32(&app->ppg_sqi_suppressed_count);
  }

  app->ppg_sqi_score = score;
  app->ppg_sqi_flags = flags;

  if ((flags == 0U) && (raw_quality >= APP_PPG_SQI_BASE_MIN_QUALITY))
  {
    app_ppg_sqi_update_baseline(metrics);
  }
#else
  (void)app;
  (void)metrics;
  (void)raw_quality;
#endif
}

/**
 ******************************************************************************
 * @brief  将 SQI 侧评分向下钳制到 app->signal_quality。
 * @param  app AppState 指针。
 * @note   保持“只降不升”的原则，便于在日志中比较原始质量分与 SQI 后质量分。
 ******************************************************************************
 */
void app_ppg_sqi_apply_quality_gate(AppState_t *app)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U) && (APP_PPG_SQI_GATE_OUTPUTS != 0U)
  if ((app != NULL) && (app->ppg_sqi_score < app->signal_quality))
  {
    app->signal_quality = app->ppg_sqi_score;
  }
#else
  (void)app;
#endif
}

/**
 ******************************************************************************
 * @brief  记录已接受的 PPG beat，用于维护 IBI 和幅度稳定性历史。
 * @param  app            AppState 指针。
 * @param  ibi_ms         本次 beat 的 inter-beat interval，单位 ms。
 * @param  beat_amplitude 峰谷幅度。
 * @note   若新 beat 与短历史中位数差异过大，先标记短暂 beat unstable，
 *         但仍更新历史，让状态在真实节律变化时可以重新收敛。
 ******************************************************************************
 */
void app_ppg_sqi_note_accepted_beat(AppState_t *app,
                                    uint16_t ibi_ms,
                                    uint32_t beat_amplitude)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U)
  uint16_t median_ibi;

  if ((app == NULL) || (ibi_ms == 0U) || (beat_amplitude == 0UL))
  {
    return;
  }

  if (ppg_sqi_state.ibi_count >= 3U)
  {
    median_ibi = app_ppg_sqi_median_ibi();
    if ((median_ibi != 0U) &&
        (app_ppg_sqi_abs_diff_u32(ibi_ms, median_ibi) >
         (((uint32_t)median_ibi * APP_PPG_SQI_IBI_JUMP_PERCENT) / 100U)))
    {
      app_ppg_sqi_mark_beat_unstable(app, 1U);
    }
  }

  if (ppg_sqi_state.amp_ema_valid != 0U)
  {
    if (((uint64_t)beat_amplitude * 100ULL) <
        ((uint64_t)ppg_sqi_state.amp_ema * APP_PPG_SQI_AMP_DROP_PERCENT))
    {
      app_ppg_sqi_mark_beat_unstable(app, 0U);
    }
    else if (((uint64_t)beat_amplitude * 100ULL) >
             ((uint64_t)ppg_sqi_state.amp_ema * APP_PPG_SQI_AMP_SPIKE_PERCENT))
    {
      app_ppg_sqi_mark_beat_unstable(app, 0U);
    }
  }

  app_ppg_sqi_add_ibi(ibi_ms);
  if (ppg_sqi_state.amp_ema_valid != 0U)
  {
    ppg_sqi_state.amp_ema =
        ((ppg_sqi_state.amp_ema * 7UL) + beat_amplitude + 4UL) >> 3;
  }
  else
  {
    ppg_sqi_state.amp_ema = beat_amplitude;
    ppg_sqi_state.amp_ema_valid = 1U;
  }
#else
  (void)app;
  (void)ibi_ms;
  (void)beat_amplitude;
#endif
}

/**
 ******************************************************************************
 * @brief  PPG 峰值检测因 IBI 异常拒绝候选 beat 时调用。
 * @param  app AppState 指针。
 ******************************************************************************
 */
void app_ppg_sqi_note_ibi_reject(AppState_t *app)
{
  app_ppg_sqi_mark_beat_unstable(app, 1U);
}

/**
 ******************************************************************************
 * @brief  PPG 峰值检测因幅度异常拒绝候选 beat 时调用。
 * @param  app AppState 指针。
 ******************************************************************************
 */
void app_ppg_sqi_note_amp_reject(AppState_t *app)
{
  app_ppg_sqi_mark_beat_unstable(app, 0U);
}

/**
 ******************************************************************************
 * @brief  判断当前 SQI 是否允许 HR/BPM 输出继续有效。
 * @param  app AppState 指针。
 * @return 允许返回 1，否则返回 0。
 ******************************************************************************
 */
uint8_t app_ppg_sqi_allows_hr(const AppState_t *app)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U) && (APP_PPG_SQI_GATE_OUTPUTS != 0U)
  if ((app == NULL) ||
      (app->finger_present == 0U) ||
      (app->contact_settle_samples > 0U) ||
      ((app->ppg_sqi_flags & (APP_PPG_SQI_FLAG_MOTION |
                              APP_PPG_SQI_FLAG_TRANSITION)) != 0U) ||
      (app->ppg_sqi_score < APP_PPG_SQI_HR_MIN_SCORE))
  {
    return 0U;
  }
#else
  (void)app;
#endif
  return 1U;
}

/**
 ******************************************************************************
 * @brief  判断当前 SQI 是否允许 SpO2 输出继续有效。
 * @param  app AppState 指针。
 * @return 允许返回 1，否则返回 0。
 ******************************************************************************
 */
uint8_t app_ppg_sqi_allows_spo2(const AppState_t *app)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U) && (APP_PPG_SQI_GATE_OUTPUTS != 0U)
  if ((app == NULL) ||
      (app->finger_present == 0U) ||
      (app->contact_settle_samples > 0U) ||
      ((app->ppg_sqi_flags & (APP_PPG_SQI_FLAG_LOW_PERFUSION |
                              APP_PPG_SQI_FLAG_MOTION |
                              APP_PPG_SQI_FLAG_BALANCE |
                              APP_PPG_SQI_FLAG_TRANSITION |
                              APP_PPG_SQI_FLAG_BEAT_UNSTABLE)) != 0U) ||
      (app->ppg_sqi_score < APP_PPG_SQI_SPO2_MIN_SCORE))
  {
    return 0U;
  }
#else
  (void)app;
#endif
  return 1U;
}

/**
 ******************************************************************************
 * @brief  判断当前 SQI 是否允许 PTT 匹配使用 PPG 峰时刻。
 * @param  app AppState 指针。
 * @return 允许返回 1，否则返回 0。
 * @note   PTT 对峰值时刻偏移最敏感，因此低灌注、运动、平衡异常和 beat unstable
 *         都会阻断 PTT 更新。
 ******************************************************************************
 */
uint8_t app_ppg_sqi_allows_ptt(const AppState_t *app)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U) && (APP_PPG_SQI_GATE_OUTPUTS != 0U)
  if ((app == NULL) ||
      (app->finger_present == 0U) ||
      (app->contact_settle_samples > 0U) ||
      ((app->ppg_sqi_flags & (APP_PPG_SQI_FLAG_LOW_PERFUSION |
                              APP_PPG_SQI_FLAG_MOTION |
                              APP_PPG_SQI_FLAG_BALANCE |
                              APP_PPG_SQI_FLAG_TRANSITION |
                              APP_PPG_SQI_FLAG_BEAT_UNSTABLE)) != 0U) ||
      (app->ppg_sqi_score < APP_PPG_SQI_PTT_MIN_SCORE))
  {
    return 0U;
  }
#else
  (void)app;
#endif
  return 1U;
}

/* 慢速一阶跟随，使用 shift 代替除法以适配 MCU 运行环境。 */
static uint32_t app_ppg_sqi_slow_follow_u32(uint32_t current,
                                            uint32_t target,
                                            uint8_t shift)
{
  uint32_t delta;
  uint32_t step;

  if (current == target)
  {
    return current;
  }

  if (current == 0UL)
  {
    return target;
  }

  if (current < target)
  {
    delta = target - current;
    step = delta >> shift;
    if (step == 0UL) { step = 1UL; }
    return current + step;
  }

  delta = current - target;
  step = delta >> shift;
  if (step == 0UL) { step = 1UL; }
  return current - step;
}

/* 16-bit 包装版本，用于 PI 基线。 */
static uint16_t app_ppg_sqi_slow_follow_u16(uint16_t current,
                                            uint16_t target,
                                            uint8_t shift)
{
  return (uint16_t)app_ppg_sqi_slow_follow_u32((uint32_t)current,
                                               (uint32_t)target,
                                               shift);
}

/* 相对跌落检测：基线过低时只依赖绝对下限，避免除噪声误判。 */
static uint8_t app_ppg_sqi_below_percent_u32(uint32_t current,
                                             uint32_t baseline,
                                             uint32_t percent)
{
  if (baseline <= APP_PPG_SQI_LOW_PERF_ABS_AC)
  {
    return 0U;
  }

  return (((uint64_t)current * 100ULL) <
          ((uint64_t)baseline * (uint64_t)percent)) ? 1U : 0U;
}

/* 16-bit 相对跌落检测，用于 PI x1000。 */
static uint8_t app_ppg_sqi_below_percent_u16(uint16_t current,
                                             uint16_t baseline,
                                             uint32_t percent)
{
  if (baseline <= APP_PPG_SQI_LOW_PERF_ABS_PI)
  {
    return 0U;
  }

  return (((uint64_t)current * 100ULL) <
          ((uint64_t)baseline * (uint64_t)percent)) ? 1U : 0U;
}

/* 饱和递增诊断计数器，避免长时间运行后回卷。 */
static void app_ppg_sqi_inc_u32(uint32_t *value)
{
  if ((value != NULL) && (*value < 0xFFFFFFFFUL))
  {
    (*value)++;
  }
}

/* 将质量分钳制到异常对应上限。 */
static void app_ppg_sqi_cap_score(uint8_t *score, uint8_t cap)
{
  if ((score != NULL) && (*score > cap))
  {
    *score = cap;
  }
}

/* 只在干净窗口中更新本地 AC/PI 基线。 */
static void app_ppg_sqi_update_baseline(const MAX30102_SignalMetrics_t *metrics)
{
  if (metrics == NULL)
  {
    return;
  }

  ppg_sqi_state.ir_ac_baseline =
      app_ppg_sqi_slow_follow_u32(ppg_sqi_state.ir_ac_baseline,
                                  metrics->ir_ac_rms,
                                  APP_PPG_SQI_BASE_SHIFT);
  ppg_sqi_state.red_ac_baseline =
      app_ppg_sqi_slow_follow_u32(ppg_sqi_state.red_ac_baseline,
                                  metrics->red_ac_rms,
                                  APP_PPG_SQI_BASE_SHIFT);
  ppg_sqi_state.ir_pi_baseline =
      app_ppg_sqi_slow_follow_u16(ppg_sqi_state.ir_pi_baseline,
                                  metrics->ir_pi_x1000,
                                  APP_PPG_SQI_BASE_SHIFT);
  ppg_sqi_state.red_pi_baseline =
      app_ppg_sqi_slow_follow_u16(ppg_sqi_state.red_pi_baseline,
                                  metrics->red_pi_x1000,
                                  APP_PPG_SQI_BASE_SHIFT);
}

/* 短 IBI 历史中位数，抗单个误检 beat。 */
static uint16_t app_ppg_sqi_median_ibi(void)
{
  uint16_t sorted[APP_PPG_SQI_IBI_HISTORY_SIZE];
  uint16_t tmp;
  uint8_t i;
  uint8_t j;
  uint8_t n;

  n = ppg_sqi_state.ibi_count;
  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < n; i++)
  {
    sorted[i] = ppg_sqi_state.ibi_history[i];
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

/* 环形写入已接受 IBI。 */
static void app_ppg_sqi_add_ibi(uint16_t ibi_ms)
{
  ppg_sqi_state.ibi_history[ppg_sqi_state.ibi_write_index] = ibi_ms;
  ppg_sqi_state.ibi_write_index =
      (uint8_t)((ppg_sqi_state.ibi_write_index + 1U) %
                APP_PPG_SQI_IBI_HISTORY_SIZE);
  if (ppg_sqi_state.ibi_count < APP_PPG_SQI_IBI_HISTORY_SIZE)
  {
    ppg_sqi_state.ibi_count++;
  }
}

/* 无符号绝对差。 */
static uint32_t app_ppg_sqi_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  return (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
}

/* 标记短暂 beat unstable，并在后续若干样本窗口内压低 SQI。 */
static void app_ppg_sqi_mark_beat_unstable(AppState_t *app, uint8_t ibi_reject)
{
#if (APP_PPG_SQI_EXPERIMENTAL != 0U)
  if (app == NULL)
  {
    return;
  }

  if (ibi_reject != 0U)
  {
    app_ppg_sqi_inc_u32(&app->ppg_sqi_ibi_reject_count);
  }
  else
  {
    app_ppg_sqi_inc_u32(&app->ppg_sqi_amp_reject_count);
  }

  app->ppg_sqi_flags |= APP_PPG_SQI_FLAG_BEAT_UNSTABLE;
  if (app->ppg_sqi_score > APP_PPG_SQI_BEAT_UNSTABLE_CAP)
  {
    app->ppg_sqi_score = APP_PPG_SQI_BEAT_UNSTABLE_CAP;
  }
  ppg_sqi_state.beat_unstable_hold = APP_PPG_SQI_BEAT_HOLD_SAMPLES;
#else
  (void)app;
  (void)ibi_reject;
#endif
}
