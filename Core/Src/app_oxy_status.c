#include "app_oxy_status.h"

#define APP_OXY_BAL_LOW_RATIO_X1000     500U
#define APP_OXY_BAL_HIGH_RATIO_X1000    2000U
#define APP_OXY_RATIO_MIN_AC_RMS        2U
#define APP_SIGNAL_QUALITY_SMOOTH_SHIFT 2U

static void app_oxy_status_update_balance(AppState_t *app, const MAX30102_SignalMetrics_t *metrics);

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

void app_oxy_status_clear_instant(AppState_t *app)
{
  app_oxy_status_reset(app);
}

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
    delta = (uint8_t)(raw_quality - current_quality);
    step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
    if (step == 0U)
    {
      step = 1U;
    }

    app->signal_quality = (uint8_t)(current_quality + step);
    return;
  }

  delta = (uint8_t)(current_quality - raw_quality);
  step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
  if (step == 0U)
  {
    step = 1U;
  }

  app->signal_quality = (uint8_t)(current_quality - step);
}

static void app_oxy_status_update_balance(AppState_t *app, const MAX30102_SignalMetrics_t *metrics)
{
  uint32_t ratio_x1000;
  uint16_t next_ratio;

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

  ratio_x1000 = (uint32_t)((((uint64_t)metrics->red_ac_rms * (uint64_t)metrics->ir_dc * 1000ULL) +
                            (((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc) / 2ULL)) /
                           ((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc));
  if (ratio_x1000 > 0xFFFFUL)
  {
    ratio_x1000 = 0xFFFFUL;
  }

  next_ratio = (uint16_t)ratio_x1000;
  if ((app->spo2_ratio_valid != 0U) || (app->spo2_ratio_x1000 != 0U))
  {
    next_ratio = (uint16_t)((((uint32_t)app->spo2_ratio_x1000 * 3U) + ratio_x1000 + 2U) / 4U);
  }

  app->spo2_ratio_valid = 1U;
  app->spo2_ratio_x1000 = next_ratio;

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
