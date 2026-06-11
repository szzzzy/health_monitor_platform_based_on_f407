/**
  ******************************************************************************
  * @file    app_oxy_status.c
  * @brief   血氧信号质量评估与平衡状态判定
  *
  * 功能：
  *   1. 汇总 RED/IR 通道交流 RMS、灌注指数 (PI)、信号质量评分
  *   2. 计算 SpO2 比率 (Red/IR AC/DC 比)，并做 EMA 平滑
  *   3. 根据比率判断传感器平衡状态（正常/偏低/偏高）
  *
  * 信号质量平滑：
  *   采用非对称步进平滑，质量上升比下降快（上升步长更大），
  *   防止短暂扰动拉低质量评分。
  *
  * SpO2 比率 = (Red_AC / Red_DC) / (IR_AC / IR_DC) × 1000
  *   简化为: (Red_AC × IR_DC × 1000) / (IR_AC × Red_DC)
  *   需要对 AC 分量做最小阈值检查以防除零。
  ******************************************************************************
  */

#include "app_oxy_status.h"

/* 平衡状态判定阈值（比率 × 1000） */
#define APP_OXY_BAL_LOW_RATIO_X1000     500U   /* 低于此值 → 偏低 */
#define APP_OXY_BAL_HIGH_RATIO_X1000    2000U  /* 高于此值 → 偏高 */
#define APP_OXY_RATIO_MIN_AC_RMS        2U     /* AC RMS 最小有效值 */

/* 信号质量平滑右移位数（步长 = delta / 4） */
#define APP_SIGNAL_QUALITY_SMOOTH_SHIFT 2U

static void app_oxy_status_update_balance(AppState_t *app, const MAX30102_SignalMetrics_t *metrics);

/**
 ******************************************************************************
 * @brief  Reset all signal-quality and SpO2-ratio fields to zero/unknown.
 * @param  app Pointer to AppState (may be NULL).
 ******************************************************************************
 */
void app_oxy_status_reset(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->signal_ir_ac_rms = 0U;
  app->signal_red_ac_rms = 0U;
  app->signal_quality = 0U;
  app->signal_ir_pi_x1000 = 0U;
  app->signal_red_pi_x1000 = 0U;
  app->spo2_ratio_valid = 0U;
  app->spo2_ratio_x1000 = 0U;
  app->spo2_balance_status = APP_OXY_BALANCE_UNKNOWN;
}

/**
 ******************************************************************************
 * @brief  Instant clear of all signal-quality fields (same as reset).
 * @param  app Pointer to AppState.
 ******************************************************************************
 */
void app_oxy_status_clear_instant(AppState_t *app)
{
  app_oxy_status_reset(app);
}

/**
 ******************************************************************************
 * @brief  Update all signal-quality and SpO2 fields from sensor metrics.
 * @param  app         Pointer to AppState (may be NULL; cleared on NULL).
 * @param  metrics     Sensor metrics containing AC RMS, DC, PI values.
 * @param  raw_quality Raw signal quality score from the sensor driver.
 * @note  Copies IR/RED AC RMS, PI, updates quality smoothed value, and
 *        recomputes SpO2 ratio and balance status.
 ******************************************************************************
 */
void app_oxy_status_update_from_metrics(AppState_t *app,
                                        const MAX30102_SignalMetrics_t *metrics,
                                        uint8_t raw_quality)
{
  if ((app == NULL) || (metrics == NULL))
  {
    app_oxy_status_clear_instant(app);
    return;
  }

  app->signal_ir_ac_rms = metrics->ir_ac_rms;
  app->signal_red_ac_rms = metrics->red_ac_rms;
  app_oxy_status_update_quality(app, raw_quality);
  app->signal_ir_pi_x1000 = metrics->ir_pi_x1000;
  app->signal_red_pi_x1000 = metrics->red_pi_x1000;
  app_oxy_status_update_balance(app, metrics);
}

/**
 ******************************************************************************
 * @brief  Smooth signal quality using asymmetric step tracking.
 * @param  app         Pointer to AppState (may be NULL).
 * @param  raw_quality Raw quality score to track towards.
 * @note   Step = delta/4 (min 1) for both rising and falling quality.
 *         Prevents brief noise dips from collapsing the quality score.
 ******************************************************************************
 */
void app_oxy_status_update_quality(AppState_t *app, uint8_t raw_quality)
{
  uint8_t current_quality;
  uint8_t delta;
  uint8_t step;

  if (app == NULL)
  {
    return;
  }

  current_quality = app->signal_quality;
  if (current_quality == raw_quality)
  {
    return;
  }

  if (current_quality < raw_quality)
  {
    /* 质量上升 */
    delta = (uint8_t)(raw_quality - current_quality);
    step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
    if (step == 0U)
    {
      step = 1U;
    }

    app->signal_quality = (uint8_t)(current_quality + step);
    return;
  }

  /* 质量下降 */
  delta = (uint8_t)(current_quality - raw_quality);
  step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
  if (step == 0U)
  {
    step = 1U;
  }

  app->signal_quality = (uint8_t)(current_quality - step);
}

/* ---- SpO2 ratio computation + RED/IR balance status classification ---- */
static void app_oxy_status_update_balance(AppState_t *app, const MAX30102_SignalMetrics_t *metrics)
{
  uint32_t ratio_x1000;
  uint16_t next_ratio;

  /* 前置条件：DC 非零、AC 大于最小阈值 */
  if ((app == NULL) || (metrics == NULL) ||
      (metrics->red_dc == 0U) || (metrics->ir_dc == 0U) ||
      (metrics->red_ac_rms < APP_OXY_RATIO_MIN_AC_RMS) ||
      (metrics->ir_ac_rms < APP_OXY_RATIO_MIN_AC_RMS))
  {
    if (app != NULL)
    {
      app->spo2_ratio_valid = 0U;
      app->spo2_balance_status = APP_OXY_BALANCE_UNKNOWN;
    }
    return;
  }

  /*
   * 比率 = (Red_AC × IR_DC × 1000) / (IR_AC × Red_DC)
   * 使用 64 位中间变量防止溢出
   */
  ratio_x1000 = (uint32_t)((((uint64_t)metrics->red_ac_rms * (uint64_t)metrics->ir_dc * 1000ULL) +
                            (((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc) / 2ULL)) /
                           ((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc));
  if (ratio_x1000 > 0xFFFFUL)
  {
    ratio_x1000 = 0xFFFFUL;
  }

  next_ratio = (uint16_t)ratio_x1000;

  /* EMA 平滑（首帧除外） */
  if ((app->spo2_ratio_valid != 0U) || (app->spo2_ratio_x1000 != 0U))
  {
    next_ratio = (uint16_t)((((uint32_t)app->spo2_ratio_x1000 * 3U) + ratio_x1000 + 2U) / 4U);
  }

  app->spo2_ratio_valid = 1U;
  app->spo2_ratio_x1000 = next_ratio;

  /* 平衡状态判定 */
  if (next_ratio < APP_OXY_BAL_LOW_RATIO_X1000)
  {
    app->spo2_balance_status = APP_OXY_BALANCE_LOW;
  }
  else if (next_ratio > APP_OXY_BAL_HIGH_RATIO_X1000)
  {
    app->spo2_balance_status = APP_OXY_BALANCE_HIGH;
  }
  else
  {
    app->spo2_balance_status = APP_OXY_BALANCE_OK;
  }
}
