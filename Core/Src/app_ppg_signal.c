#include "app_ppg_signal.h"

#include <string.h>

#define MAX30102_FINGER_ON_DELTA       6000UL
#define MAX30102_FINGER_OFF_DELTA      3000UL
#define MAX30102_FINGER_ON_NOISE_GAIN  4UL
#define MAX30102_FINGER_OFF_NOISE_GAIN 2UL
#define MAX30102_FINGER_ON_DELTA_MAX   18000UL
#define MAX30102_FINGER_OFF_DELTA_MAX  9000UL

static struct
{
  uint32_t ir_high;
  uint32_t ir_low;
  uint32_t red_high;
  uint32_t red_low;
} signal_envelope;

static uint32_t app_ppg_signal_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);

void app_ppg_signal_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;
}

void app_ppg_signal_reset_envelope(void)
{
  (void)memset(&signal_envelope, 0, sizeof(signal_envelope));
}

void app_ppg_signal_update_activity(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if ((signal_envelope.ir_high == 0U) || (app->ir_value >= signal_envelope.ir_high))
  {
    signal_envelope.ir_high = app->ir_value;
  }
  else
  {
    signal_envelope.ir_high = app_ppg_signal_slow_follow_u32(signal_envelope.ir_high, app->ir_value, 4U);
  }

  if ((signal_envelope.ir_low == 0U) || (app->ir_value <= signal_envelope.ir_low))
  {
    signal_envelope.ir_low = app->ir_value;
  }
  else
  {
    signal_envelope.ir_low = app_ppg_signal_slow_follow_u32(signal_envelope.ir_low, app->ir_value, 4U);
  }

  if ((signal_envelope.red_high == 0U) || (app->red_value >= signal_envelope.red_high))
  {
    signal_envelope.red_high = app->red_value;
  }
  else
  {
    signal_envelope.red_high = app_ppg_signal_slow_follow_u32(signal_envelope.red_high, app->red_value, 4U);
  }

  if ((signal_envelope.red_low == 0U) || (app->red_value <= signal_envelope.red_low))
  {
    signal_envelope.red_low = app->red_value;
  }
  else
  {
    signal_envelope.red_low = app_ppg_signal_slow_follow_u32(signal_envelope.red_low, app->red_value, 4U);
  }

  app->ir_signal_delta = (app->ir_value > app->baseline_ir) ?
                         (app->ir_value - app->baseline_ir) : 0U;
  app->ir_signal_span = (signal_envelope.ir_high >= signal_envelope.ir_low) ?
                        (signal_envelope.ir_high - signal_envelope.ir_low) : 0U;
  app->red_signal_span = (signal_envelope.red_high >= signal_envelope.red_low) ?
                         (signal_envelope.red_high - signal_envelope.red_low) : 0U;
}

void app_ppg_signal_update_adaptive_thresholds(AppState_t *app,
                                               const MAX30102_Baseline_t *baseline)
{
  uint32_t baseline_noise;
  uint32_t on_delta;
  uint32_t off_delta;

  if ((app == NULL) || (baseline == NULL))
  {
    return;
  }

  baseline_noise = max30102_baseline_get_noise_ir(baseline);
  on_delta = baseline_noise * MAX30102_FINGER_ON_NOISE_GAIN;
  off_delta = baseline_noise * MAX30102_FINGER_OFF_NOISE_GAIN;
  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;

  if (on_delta > app->adaptive_finger_on_delta)
  {
    app->adaptive_finger_on_delta = on_delta;
  }

  if (off_delta > app->adaptive_finger_off_delta)
  {
    app->adaptive_finger_off_delta = off_delta;
  }

  if (app->adaptive_finger_on_delta > MAX30102_FINGER_ON_DELTA_MAX)
  {
    app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA_MAX;
  }

  if (app->adaptive_finger_off_delta > MAX30102_FINGER_OFF_DELTA_MAX)
  {
    app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA_MAX;
  }
}

uint8_t app_ppg_signal_is_raw_present(const AppState_t *app)
{
  uint32_t finger_delta_threshold;
  uint32_t ir_delta;

  if (app == NULL)
  {
    return 0U;
  }

  if (app->ir_value <= app->baseline_ir)
  {
    return 0U;
  }

  finger_delta_threshold = (app->finger_present != 0U) ?
                           app->adaptive_finger_off_delta :
                           app->adaptive_finger_on_delta;
  ir_delta = app->ir_value - app->baseline_ir;

  return (ir_delta >= finger_delta_threshold) ? 1U : 0U;
}

void app_ppg_signal_track_background_ir(AppState_t *app, MAX30102_Baseline_t *baseline)
{
  if ((app == NULL) || (baseline == NULL))
  {
    return;
  }

  max30102_baseline_track_background(baseline, app->ir_value);
  app->baseline_ir = max30102_baseline_get_tracked_ir(baseline);
}

static uint32_t app_ppg_signal_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
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
