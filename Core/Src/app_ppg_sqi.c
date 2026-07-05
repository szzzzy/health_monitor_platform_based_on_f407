#include "app_ppg_sqi.h"

#include <string.h>

#define APP_PPG_SQI_BASE_SHIFT              6U
#define APP_PPG_SQI_BASE_MIN_QUALITY        35U
#define APP_PPG_SQI_LOW_PERF_PERCENT        45U
#define APP_PPG_SQI_LOW_PERF_ABS_AC         3U
#define APP_PPG_SQI_LOW_PERF_ABS_PI         1U
#define APP_PPG_SQI_LOW_PERF_CAP            24U
#define APP_PPG_SQI_MOTION_CAP              20U
#define APP_PPG_SQI_TRANSITION_CAP          15U
#define APP_PPG_SQI_BALANCE_CAP             45U
#define APP_PPG_SQI_BEAT_UNSTABLE_CAP       35U
#define APP_PPG_SQI_HR_MIN_SCORE            30U
#define APP_PPG_SQI_SPO2_MIN_SCORE          35U
#define APP_PPG_SQI_PTT_MIN_SCORE           45U
#define APP_PPG_SQI_IBI_HISTORY_SIZE        5U
#define APP_PPG_SQI_IBI_JUMP_PERCENT        35U
#define APP_PPG_SQI_AMP_DROP_PERCENT        40U
#define APP_PPG_SQI_AMP_SPIKE_PERCENT       260U
#define APP_PPG_SQI_BEAT_HOLD_SAMPLES       50U

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

void app_ppg_sqi_note_ibi_reject(AppState_t *app)
{
  app_ppg_sqi_mark_beat_unstable(app, 1U);
}

void app_ppg_sqi_note_amp_reject(AppState_t *app)
{
  app_ppg_sqi_mark_beat_unstable(app, 0U);
}

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

static uint16_t app_ppg_sqi_slow_follow_u16(uint16_t current,
                                            uint16_t target,
                                            uint8_t shift)
{
  return (uint16_t)app_ppg_sqi_slow_follow_u32((uint32_t)current,
                                               (uint32_t)target,
                                               shift);
}

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

static void app_ppg_sqi_inc_u32(uint32_t *value)
{
  if ((value != NULL) && (*value < 0xFFFFFFFFUL))
  {
    (*value)++;
  }
}

static void app_ppg_sqi_cap_score(uint8_t *score, uint8_t cap)
{
  if ((score != NULL) && (*score > cap))
  {
    *score = cap;
  }
}

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

static uint32_t app_ppg_sqi_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  return (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
}

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
