/*
 * PPG-only motion artifact 检测——无加速度计，纯从 PPG 信号推断。
 *
 * 三条检测信号及其贡献分：
 *   1. AC RMS 尖峰 (25分/通道): IR 或 RED 当前 RMS ≥ baseline × 3.0 且增量 ≥ 80。
 *   2. RED/IR 平衡异常 (25分): AC ratio x1000 落在 [350, 2800] 以外，或 R/BAL 已偏。
 *   3. SQ 骤降 (35分): raw_quality 从 ≥45 的前值骤降 ≥25 点。
 *   总分截断至 100。
 *
 * 迟滞状态机 (防止边界抖动):
 *   - 进入: score ≥ 60 连续 5 拍确认 → motion_artifact = 1。
 *   - 退出: score ≤ 30 连续 30 拍确认 → motion_artifact = 0。
 *   - Motion 期间 AC RMS baseline 冻结（防止污染 baseline）。
 *   - 非 motion 期间 baseline 慢速跟踪 (shift=6)，维持自适应能力。
 *
 * Motion 对测量的影响 (由 app_measurement 层执行):
 *   - HR/SpO2/RR/IBI/HRV valid 清零，旧值保留（UI 显示旧值+"?"）。
 *   - HRV/IBI ring buffer 不清——motion 结束可立即恢复。
 *   - 流式脉冲检测状态重置（防止跨 motion 的假 IBI）。
 */

#include "app_motion.h"

#include <string.h>

#define APP_MOTION_RMS_BASE_SHIFT      6U
#define APP_MOTION_MIN_BASELINE_RMS    8U
/* RMS 尖峰：当前值 >= baseline × 4.0 且增量 >= 120。 */
#define APP_MOTION_RMS_SPIKE_X100      400U
#define APP_MOTION_RMS_SPIKE_MIN_DELTA 120U
#define APP_MOTION_RMS_SPIKE_SCORE     25U
/* RED/IR 平衡异常：比值落在 [0.35, 2.80] 以外。 */
#define APP_MOTION_BALANCE_LOW_X1000   350U
#define APP_MOTION_BALANCE_HIGH_X1000  2800U
#define APP_MOTION_BALANCE_SCORE       25U
/* SQ 骤降：从 >=45 的前一值单步下降 >=30。 */
#define APP_MOTION_SQ_DROP_MIN_PREV    45U
#define APP_MOTION_SQ_DROP_DELTA       30U
#define APP_MOTION_SQ_DROP_SCORE       25U
/* 迟滞阈值与确认计数。进入更严格：score≥75 连续 20 拍。 */
#define APP_MOTION_SCORE_ON            75U
#define APP_MOTION_SCORE_OFF           30U
#define APP_MOTION_ON_CONFIRM_SAMPLES  20U
#define APP_MOTION_OFF_CONFIRM_SAMPLES 30U

static struct
{
  uint8_t initialized;
  uint8_t last_quality_valid;
  uint8_t last_raw_quality;
  uint8_t on_count;
  uint8_t off_count;
  /* AC RMS 慢速跟踪基线——motion 期间冻结。 */
  uint32_t ir_ac_rms_baseline;
  uint32_t red_ac_rms_baseline;
} motion_state;

static uint32_t app_motion_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);
static uint8_t app_motion_ac_rms_spike(uint32_t current_rms, uint32_t baseline_rms);

void app_motion_reset(AppState_t *app)
{
  (void)memset(&motion_state, 0, sizeof(motion_state));

  if (app == NULL)
  {
    return;
  }

  app->motion_artifact = 0U;
  app->motion_score = 0U;
}

void app_motion_update_artifact(AppState_t *app,
                                const MAX30102_SignalMetrics_t *metrics,
                                uint8_t raw_quality)
{
  uint8_t score = 0U;
  uint8_t balance_abnormal = 0U;
  uint32_t ac_ratio_x1000;

  if (app == NULL)
  {
    return;
  }

  if ((metrics == NULL) || (app->finger_present == 0U))
  {
    app->motion_score = 0U;
    motion_state.last_quality_valid = 0U;
    if (app->motion_artifact != 0U)
    {
      if (motion_state.off_count < APP_MOTION_OFF_CONFIRM_SAMPLES)
      {
        motion_state.off_count++;
      }

      if (motion_state.off_count >= APP_MOTION_OFF_CONFIRM_SAMPLES)
      {
        app->motion_artifact = 0U;
        motion_state.off_count = 0U;
      }
    }
    return;
  }

  if (motion_state.initialized == 0U)
  {
    motion_state.ir_ac_rms_baseline = metrics->ir_ac_rms;
    motion_state.red_ac_rms_baseline = metrics->red_ac_rms;
    motion_state.initialized = 1U;
  }
  else
  {
    if (app_motion_ac_rms_spike(metrics->ir_ac_rms, motion_state.ir_ac_rms_baseline) != 0U)
    {
      score = (uint8_t)(score + APP_MOTION_RMS_SPIKE_SCORE);
    }

    if (app_motion_ac_rms_spike(metrics->red_ac_rms, motion_state.red_ac_rms_baseline) != 0U)
    {
      score = (uint8_t)(score + APP_MOTION_RMS_SPIKE_SCORE);
    }
  }

  if (metrics->ir_ac_rms != 0U)
  {
    ac_ratio_x1000 = (uint32_t)((((uint64_t)metrics->red_ac_rms * 1000ULL) +
                                 (metrics->ir_ac_rms / 2U)) / metrics->ir_ac_rms);
    if ((ac_ratio_x1000 < APP_MOTION_BALANCE_LOW_X1000) ||
        (ac_ratio_x1000 > APP_MOTION_BALANCE_HIGH_X1000))
    {
      balance_abnormal = 1U;
    }
  }

  if ((app->spo2_balance_status == APP_OXY_BALANCE_LOW) ||
      (app->spo2_balance_status == APP_OXY_BALANCE_HIGH))
  {
    balance_abnormal = 1U;
  }

  if (balance_abnormal != 0U)
  {
    score = (uint8_t)(score + APP_MOTION_BALANCE_SCORE);
  }

  if ((motion_state.last_quality_valid != 0U) &&
      (motion_state.last_raw_quality >= APP_MOTION_SQ_DROP_MIN_PREV) &&
      (raw_quality <= (uint8_t)(motion_state.last_raw_quality - APP_MOTION_SQ_DROP_DELTA)))
  {
    score = (uint8_t)(score + APP_MOTION_SQ_DROP_SCORE);
  }

  if (score > 100U)
  {
    score = 100U;
  }

  app->motion_score = score;

  /*
   * 迟滞：进入需 score >= 60 连续 5 拍确认；
   * 退出需 score <= 30 连续 30 拍确认。
   * 任何不符合的拍都会重置确认计数。
   */
  if (app->motion_artifact != 0U)
  {
    if (score <= APP_MOTION_SCORE_OFF)
    {
      if (motion_state.off_count < APP_MOTION_OFF_CONFIRM_SAMPLES)
      {
        motion_state.off_count++;
      }
    }
    else
    {
      motion_state.off_count = 0U;
    }

    if (motion_state.off_count >= APP_MOTION_OFF_CONFIRM_SAMPLES)
    {
      app->motion_artifact = 0U;
      motion_state.off_count = 0U;
      motion_state.on_count = 0U;
    }
  }
  else
  {
    if (score >= APP_MOTION_SCORE_ON)
    {
      if (motion_state.on_count < APP_MOTION_ON_CONFIRM_SAMPLES)
      {
        motion_state.on_count++;
      }
    }
    else
    {
      motion_state.on_count = 0U;
    }

    if (motion_state.on_count >= APP_MOTION_ON_CONFIRM_SAMPLES)
    {
      app->motion_artifact = 1U;
      motion_state.on_count = 0U;
      motion_state.off_count = 0U;
    }
  }

  /*
   * AC RMS baseline 在 motion 期间冻结（artifact RMS 会污染它）。
   * 仅当不在 motion 且 score 低于 ON 阈值时才更新。
   */
  if ((app->motion_artifact == 0U) && (score < APP_MOTION_SCORE_ON))
  {
    motion_state.ir_ac_rms_baseline = app_motion_slow_follow_u32(motion_state.ir_ac_rms_baseline,
                                                                 metrics->ir_ac_rms,
                                                                 APP_MOTION_RMS_BASE_SHIFT);
    motion_state.red_ac_rms_baseline = app_motion_slow_follow_u32(motion_state.red_ac_rms_baseline,
                                                                  metrics->red_ac_rms,
                                                                  APP_MOTION_RMS_BASE_SHIFT);
  }

  motion_state.last_raw_quality = raw_quality;
  motion_state.last_quality_valid = 1U;
}

static uint8_t app_motion_ac_rms_spike(uint32_t current_rms, uint32_t baseline_rms)
{
  if ((baseline_rms < APP_MOTION_MIN_BASELINE_RMS) ||
      (current_rms <= baseline_rms) ||
      ((current_rms - baseline_rms) < APP_MOTION_RMS_SPIKE_MIN_DELTA))
  {
    return 0U;
  }

  if (((uint64_t)current_rms * 100ULL) >=
      ((uint64_t)baseline_rms * (uint64_t)APP_MOTION_RMS_SPIKE_X100))
  {
    return 1U;
  }

  return 0U;
}

static uint32_t app_motion_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
{
  uint32_t delta;
  uint32_t step;

  if (current == target)
  {
    return current;
  }

  if (current < target)
  {
    delta = target - current;
    step = delta >> shift;
    if (step == 0U)
    {
      step = 1U;
    }

    return current + step;
  }

  delta = current - target;
  step = delta >> shift;
  if (step == 0U)
  {
    step = 1U;
  }

  return current - step;
}
