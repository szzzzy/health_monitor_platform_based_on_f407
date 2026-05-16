/**
  ******************************************************************************
  * @file    app_ppg_signal.c
  * @brief   PPG 信号包络追踪、自适应手指检测与背景基线维护
  *
  * 功能：
  *   1. 信号包络追踪 — 维护 IR/RED 通道的高低包络线（慢跟随衰减）
  *   2. 自适应手指检测阈值 — 基于背景噪声动态调整 on/off 阈值
  *   3. 原始信号存在性判定 — IR 增量是否超过自适应阈值
  *   4. IR 背景基线跟踪 — 将当前 IR 值送入基线跟踪器
  ******************************************************************************
  */

#include "app_ppg_signal.h"

#include <string.h>

/* 手指检测阈值参数 */
#define MAX30102_FINGER_ON_DELTA       6000UL   /* 默认手指就位阈值 (IR ADC LSB) */
#define MAX30102_FINGER_OFF_DELTA      3000UL   /* 默认手指离开阈值 (IR ADC LSB) */
#define MAX30102_FINGER_ON_NOISE_GAIN  4UL      /* 就位阈值 = 噪声 × 4 */
#define MAX30102_FINGER_OFF_NOISE_GAIN 2UL      /* 离开阈值 = 噪声 × 2 */
#define MAX30102_FINGER_ON_DELTA_MAX   18000UL  /* 就位阈值上限 */
#define MAX30102_FINGER_OFF_DELTA_MAX  9000UL   /* 离开阈值上限 */

/* 信号包络状态 */
static struct
{
  uint32_t ir_high;   /* IR 通道包络高值（慢衰减） */
  uint32_t ir_low;    /* IR 通道包络低值（慢上升） */
  uint32_t red_high;  /* RED 通道包络高值 */
  uint32_t red_low;   /* RED 通道包络低值 */
} signal_envelope;

/* 慢跟随辅助函数：当前值以 1/2^shift 速率向目标值衰减 */
static uint32_t app_ppg_signal_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);

/* 初始化自适应手指检测阈值为默认值 */
void app_ppg_signal_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;
}

/* 重置信号包络（手指状态切换时调用） */
void app_ppg_signal_reset_envelope(void)
{
  (void)memset(&signal_envelope, 0, sizeof(signal_envelope));
}

/*
 * 更新信号包络与活动指标
 *
 * 包络追踪策略：
 *   - 新样本突破当前包络 → 立即跟随（快速上升/下降）
 *   - 新样本在包络范围内 → 慢速向当前值衰减 (1/16 每拍)
 *
 * 输出字段：
 *   ir_signal_delta  = IR - 基线 (当前偏移量)
 *   ir_signal_span   = IR 包络峰谷差 (反映搏动幅度)
 *   red_signal_span  = RED 包络峰谷差
 */
void app_ppg_signal_update_activity(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  /* IR 高包络：突破则立即跟随，否则慢衰减 */
  if ((signal_envelope.ir_high == 0U) || (app->ir_value >= signal_envelope.ir_high))
  {
    signal_envelope.ir_high = app->ir_value;
  }
  else
  {
    signal_envelope.ir_high = app_ppg_signal_slow_follow_u32(signal_envelope.ir_high, app->ir_value, 4U);
  }

  /* IR 低包络：突破则立即跟随，否则慢上升 */
  if ((signal_envelope.ir_low == 0U) || (app->ir_value <= signal_envelope.ir_low))
  {
    signal_envelope.ir_low = app->ir_value;
  }
  else
  {
    signal_envelope.ir_low = app_ppg_signal_slow_follow_u32(signal_envelope.ir_low, app->ir_value, 4U);
  }

  /* RED 高包络 */
  if ((signal_envelope.red_high == 0U) || (app->red_value >= signal_envelope.red_high))
  {
    signal_envelope.red_high = app->red_value;
  }
  else
  {
    signal_envelope.red_high = app_ppg_signal_slow_follow_u32(signal_envelope.red_high, app->red_value, 4U);
  }

  /* RED 低包络 */
  if ((signal_envelope.red_low == 0U) || (app->red_value <= signal_envelope.red_low))
  {
    signal_envelope.red_low = app->red_value;
  }
  else
  {
    signal_envelope.red_low = app_ppg_signal_slow_follow_u32(signal_envelope.red_low, app->red_value, 4U);
  }

  /* 计算派生指标 */
  app->ir_signal_delta = (app->ir_value > app->baseline_ir) ?
                         (app->ir_value - app->baseline_ir) : 0U;
  app->ir_signal_span = (signal_envelope.ir_high >= signal_envelope.ir_low) ?
                        (signal_envelope.ir_high - signal_envelope.ir_low) : 0U;
  app->red_signal_span = (signal_envelope.red_high >= signal_envelope.red_low) ?
                         (signal_envelope.red_high - signal_envelope.red_low) : 0U;
}

/*
 * 基于背景噪声自适应更新手指检测阈值
 *
 * on_delta  = noise × 4  (就位需要更严格的阈值)
 * off_delta = noise × 2  (离开更敏感)
 *
 * 自适应值不低于默认值，不高于上限。
 */
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

  /* 从默认值出发，仅在噪声驱动的值更大时提升阈值 */
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

  /* 钳位上限 */
  if (app->adaptive_finger_on_delta > MAX30102_FINGER_ON_DELTA_MAX)
  {
    app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA_MAX;
  }

  if (app->adaptive_finger_off_delta > MAX30102_FINGER_OFF_DELTA_MAX)
  {
    app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA_MAX;
  }
}

/*
 * 原始 PPG 信号存在性判定
 *
 * 逻辑：
 *   手指就位时使用较低的离开阈值（避免频繁切换）
 *   手指离开时使用较高的就位阈值（防止噪声误触发）
 *   滞回特性提高了状态切换的稳定性。
 *
 * 返回值：1 = 信号存在（IR 偏移超过阈值），0 = 无信号
 */
uint8_t app_ppg_signal_is_raw_present(const AppState_t *app)
{
  uint32_t finger_delta_threshold;
  uint32_t ir_delta;

  if (app == NULL)
  {
    return 0U;
  }

  /* IR 值必须高于基线 */
  if (app->ir_value <= app->baseline_ir)
  {
    return 0U;
  }

  /* 滞回阈值选择 */
  finger_delta_threshold = (app->finger_present != 0U) ?
                           app->adaptive_finger_off_delta :
                           app->adaptive_finger_on_delta;
  ir_delta = app->ir_value - app->baseline_ir;

  return (ir_delta >= finger_delta_threshold) ? 1U : 0U;
}

/* 将当前 IR 值送入基线跟踪器，更新跟踪基线 */
void app_ppg_signal_track_background_ir(AppState_t *app, MAX30102_Baseline_t *baseline)
{
  if ((app == NULL) || (baseline == NULL))
  {
    return;
  }

  max30102_baseline_track_background(baseline, app->ir_value);
  app->baseline_ir = max30102_baseline_get_tracked_ir(baseline);
}

/*
 * 慢跟随函数
 *
 * 当前值以 1/2^shift 速率向目标值趋近。
 * 每拍步长 = |current - target| / 2^shift，最小 1。
 *
 * 用途：包络线在信号减弱时缓慢衰减而非骤降，保持包络的稳定性。
 */
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
