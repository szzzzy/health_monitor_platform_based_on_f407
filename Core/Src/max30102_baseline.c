/**
  ******************************************************************************
  * @file    max30102_baseline.c
  * @brief   MAX30102 背景基线统计 + 手指检测
  ******************************************************************************
  */

#include "max30102_baseline.h"
#include "max30102_algo_utils.h"

/*
 * 重置背景基线统计结构体。
 * 初始化后：
 * - ir_sum 用于求平均
 * - ir_min / ir_max 用于评估背景稳定性
 * - sample_count 用于判断是否采够目标样本数
 */
void max30102_baseline_reset(MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->ir_sum = 0U;
  baseline->ir_min = 0xFFFFFFFFUL;
  baseline->ir_max = 0U;
  baseline->tracked_ir = 0U;
  baseline->noise_ir = 0U;
  baseline->sample_count = 0U;
}

/*
 * 把一个新的 IR 样本纳入背景统计。
 * 该函数只用于"无手指阶段"的背景采样，不用于运行时的实时滤波。
 */
void max30102_baseline_add_ir(MAX30102_Baseline_t *baseline, uint32_t ir_value)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->ir_sum += ir_value;

  if (ir_value < baseline->ir_min)
  {
    baseline->ir_min = ir_value;
  }

  if (ir_value > baseline->ir_max)
  {
    baseline->ir_max = ir_value;
  }

  baseline->sample_count++;
}

/* 判断背景采样是否已经采够目标数量。 */
uint8_t max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return (baseline->sample_count >= target_samples) ? 1U : 0U;
}

/* 计算背景平均 IR。若尚无样本，则返回 0。 */
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_sum / baseline->sample_count);
}

/* 计算背景阶段的波动范围。若尚无样本，则返回 0。 */
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_max - baseline->ir_min);
}

/*
 * 判断背景采样阶段是否足够稳定。
 * 这里使用"最大值 - 最小值"的简单波动范围判断，优点是实现轻量、直观。
 */
uint8_t max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return ((baseline->ir_max - baseline->ir_min) <= stable_range) ? 1U : 0U;
}

/*
 * 用开机背景采样的结果为运行时基线跟踪器设定初值。
 * tracked_ir 表示当前认为的"无手指背景 IR"，noise_ir 表示背景波动量级。
 */
void max30102_baseline_seed_tracking(MAX30102_Baseline_t *baseline,
                                     uint32_t baseline_ir,
                                     uint32_t noise_ir)
{
  if (baseline == NULL)
  {
    return;
  }

  baseline->tracked_ir = baseline_ir;
  baseline->noise_ir = (noise_ir != 0U) ? noise_ir : 1U;
}

/*
 * 在"当前确认无手指"的阶段，持续跟踪背景 IR。
 * 这样即使环境光、供电或模块温漂导致背景缓慢变化，基线也能慢慢跟上，
 * 不会一直抱着开机那一次采样结果不放。
 */
void max30102_baseline_track_background(MAX30102_Baseline_t *baseline, uint32_t ir_value)
{
  uint32_t deviation;

  if (baseline == NULL)
  {
    return;
  }

  if (baseline->tracked_ir == 0U)
  {
    baseline->tracked_ir = ir_value;
    baseline->noise_ir = 1U;
    return;
  }

  if (ir_value >= baseline->tracked_ir)
  {
    deviation = ir_value - baseline->tracked_ir;
  }
  else
  {
    deviation = baseline->tracked_ir - ir_value;
  }

  /*
   * 背景基线本身跟随得更慢，避免把短时噪声直接吞进基线；
   * 背景波动量级跟随得稍快，用于后续自适应阈值。
   */
  baseline->tracked_ir = max30102_slow_follow_u32(baseline->tracked_ir, ir_value, 4U);
  baseline->noise_ir = max30102_slow_follow_u32(baseline->noise_ir, deviation, 3U);

  if (baseline->noise_ir == 0U)
  {
    baseline->noise_ir = 1U;
  }
}

/* 获取运行时动态背景基线。 */
uint32_t max30102_baseline_get_tracked_ir(const MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  if (baseline->tracked_ir != 0U)
  {
    return baseline->tracked_ir;
  }

  return max30102_baseline_get_average_ir(baseline);
}

/* 获取运行时背景噪声估计值。 */
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return baseline->noise_ir;
}

/*
 * 用双阈值滞回做手指状态切换，避免临界点附近因为噪声产生反复跳变。
 * current_state 表示调用前的状态：
 * - 0 表示当前认为"无手指"
 * - 1 表示当前认为"有手指"
 */
uint8_t max30102_detect_finger(uint32_t ir_value,
                               uint32_t baseline_ir,
                               uint8_t current_state,
                               uint32_t finger_on_delta,
                               uint32_t finger_off_delta)
{
  if (current_state == 0U)
  {
    if (ir_value > (baseline_ir + finger_on_delta))
    {
      return 1U;
    }
  }
  else
  {
    if (ir_value < (baseline_ir + finger_off_delta))
    {
      return 0U;
    }
  }

  return current_state;
}
