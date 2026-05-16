#include "app_bpm_filter.h"

#include <string.h>

#define APP_BPM_CONFIRM_DELTA        8U
#define APP_BPM_SWITCH_DELTA         14U
#define APP_BPM_SPIKE_DELTA          24U
#define APP_BPM_START_CONFIRM_COUNT  2U
#define APP_BPM_SWITCH_CONFIRM_COUNT 3U
#define APP_BPM_SPIKE_CONFIRM_COUNT  5U
#define APP_BPM_INVALID_HOLD_TICKS   30U
#define APP_BPM_MAX_STEP_PER_UPDATE  5U

static uint8_t app_bpm_abs_diff_u8(uint8_t lhs, uint8_t rhs);
static uint8_t app_bpm_limit_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step);
static uint8_t app_bpm_median3_u8(uint8_t a, uint8_t b, uint8_t c);
static uint8_t app_bpm_filter_raw(AppState_t *app, uint8_t raw_bpm_value);

void app_bpm_filter_reset(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
}

uint8_t app_bpm_filter_update(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value)
{
  uint8_t diff;
  uint8_t filtered_bpm;
  uint8_t required_confirm_count;
  uint16_t blended_value;

  if (app == NULL)
  {
    return 0U;
  }

  if (raw_bpm_valid != 0U)
  {
    filtered_bpm = app_bpm_filter_raw(app, raw_bpm_value);
    app->bpm_invalid_hold_count = 0U;

    if (app->bpm_valid == 0U)
    {
      if ((app->bpm_candidate_count == 0U) ||
          (app_bpm_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_CONFIRM_DELTA))
      {
        app->bpm_candidate_value = filtered_bpm;
        app->bpm_candidate_count = 1U;
        return 0U;
      }

      if (app->bpm_candidate_count < 0xFFU)
      {
        app->bpm_candidate_count++;
      }

      blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
      app->bpm_candidate_value = (uint8_t)blended_value;

      if (app->bpm_candidate_count >= APP_BPM_START_CONFIRM_COUNT)
      {
        app->bpm_valid = 1U;
        app->bpm_value = app->bpm_candidate_value;
        app->bpm_candidate_count = 0U;
      }

      return 0U;
    }

    diff = app_bpm_abs_diff_u8(filtered_bpm, app->bpm_value);
    if (diff <= APP_BPM_CONFIRM_DELTA)
    {
      blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)filtered_bpm + 4U) / 8U;
      app->bpm_value = app_bpm_limit_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
      return 0U;
    }

    required_confirm_count = APP_BPM_SWITCH_CONFIRM_COUNT;
    if (diff > APP_BPM_SPIKE_DELTA)
    {
      required_confirm_count = APP_BPM_SPIKE_CONFIRM_COUNT;
    }

    if ((app->bpm_candidate_count == 0U) ||
        (app_bpm_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_SWITCH_DELTA))
    {
      app->bpm_candidate_value = filtered_bpm;
      app->bpm_candidate_count = 1U;
      return 0U;
    }

    if (app->bpm_candidate_count < 0xFFU)
    {
      app->bpm_candidate_count++;
    }

    blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
    app->bpm_candidate_value = (uint8_t)blended_value;

    if (app->bpm_candidate_count >= required_confirm_count)
    {
      if (diff > APP_BPM_SPIKE_DELTA)
      {
        blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)app->bpm_candidate_value + 4U) / 8U;
      }
      else
      {
        blended_value = (((uint16_t)app->bpm_value * 5U) +
                         ((uint16_t)app->bpm_candidate_value * 3U) + 4U) / 8U;
      }

      app->bpm_value = app_bpm_limit_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
    }

    return 0U;
  }

  if (app->bpm_valid != 0U)
  {
    if (app->bpm_invalid_hold_count < APP_BPM_INVALID_HOLD_TICKS)
    {
      app->bpm_invalid_hold_count++;
      return 0U;
    }
  }

  app_bpm_filter_reset(app);
  return 1U;
}

static uint8_t app_bpm_abs_diff_u8(uint8_t lhs, uint8_t rhs)
{
  if (lhs >= rhs)
  {
    return (uint8_t)(lhs - rhs);
  }

  return (uint8_t)(rhs - lhs);
}

static uint8_t app_bpm_limit_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step)
{
  if (target_bpm > current_bpm)
  {
    if ((uint8_t)(target_bpm - current_bpm) > max_step)
    {
      return (uint8_t)(current_bpm + max_step);
    }
  }
  else if (current_bpm > target_bpm)
  {
    if ((uint8_t)(current_bpm - target_bpm) > max_step)
    {
      return (uint8_t)(current_bpm - max_step);
    }
  }

  return target_bpm;
}

static uint8_t app_bpm_median3_u8(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t temp;

  if (a > b)
  {
    temp = a;
    a = b;
    b = temp;
  }

  if (b > c)
  {
    temp = b;
    b = c;
    c = temp;
  }

  if (a > b)
  {
    b = a;
  }

  return b;
}

static uint8_t app_bpm_filter_raw(AppState_t *app, uint8_t raw_bpm_value)
{
  uint8_t first_index;
  uint8_t second_index;

  if (app == NULL)
  {
    return raw_bpm_value;
  }

  app->raw_bpm_history[app->raw_bpm_history_index] = raw_bpm_value;
  app->raw_bpm_history_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  if (app->raw_bpm_history_count < 3U)
  {
    app->raw_bpm_history_count++;
  }

  if (app->raw_bpm_history_count == 1U)
  {
    return app->raw_bpm_history[0];
  }

  if (app->raw_bpm_history_count == 2U)
  {
    return (uint8_t)(((uint16_t)app->raw_bpm_history[0] +
                      (uint16_t)app->raw_bpm_history[1] + 1U) / 2U);
  }

  first_index = app->raw_bpm_history_index;
  second_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  return app_bpm_median3_u8(app->raw_bpm_history[first_index],
                            app->raw_bpm_history[second_index],
                            app->raw_bpm_history[(uint8_t)((app->raw_bpm_history_index + 2U) % 3U)]);
}
