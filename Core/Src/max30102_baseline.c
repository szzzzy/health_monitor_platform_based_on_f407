/**
  ******************************************************************************
  * @file    max30102_baseline.c
  * @brief   MAX30102 背景基线统计 + 手指检测
  ******************************************************************************
  */

#include "max30102_baseline.h"
#include "max30102_algo_utils.h"

/**
 * @brief  将基线统计结构重置为初始状态。
 * @param  baseline 指向要重置的基线结构的指针。
 * @note   将 ir_sum、ir_min、ir_max、tracked_ir、noise_ir 以及
 *         sample_count 设置为其默认（零/反转）值。
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

/**
 * @brief  将新的 IR 样本纳入背景基线统计。
 * @param  baseline 指向基线结构的指针。
 * @param  ir_value 新的 IR 样本值。
 * @note   用于手指移开时的背景采集，而非实时滤波。
 *         更新累加和、最小值、最大值和样本计数。
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

/**
 * @brief  检查背景采样是否已达到目标数量。
 * @param  baseline       指向基线结构的指针。
 * @param  target_samples 期望的样本数。
 * @return 如果 sample_count >= target_samples 则返回 1，否则返回 0。
 */
uint8_t max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return (baseline->sample_count >= target_samples) ? 1U : 0U;
}

/**
 * @brief  获取背景采样中的平均 IR 值。
 * @param  baseline 指向基线结构的指针。
 * @return 平均 IR 值；如果未采集到样本则返回 0。
 */
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_sum / baseline->sample_count);
}

/**
 * @brief  获取背景采样期间的 IR 范围（最大值 - 最小值）。
 * @param  baseline 指向基线结构的指针。
 * @return 范围值；如果未采集到样本则返回 0。
 */
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return (baseline->ir_max - baseline->ir_min);
}

/**
 * @brief  检查背景噪声是否在可接受的稳定范围内。
 * @param  baseline     指向基线结构的指针。
 * @param  stable_range 允许的最大峰峰噪声。
 * @return 如果 (ir_max - ir_min) <= stable_range 则返回 1，否则返回 0。
 * @note   使用简单的基于范围的噪声检查，实现轻量化。
 */
uint8_t max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range)
{
  if ((baseline == NULL) || (baseline->sample_count == 0U))
  {
    return 0U;
  }

  return ((baseline->ir_max - baseline->ir_min) <= stable_range) ? 1U : 0U;
}

/**
 * @brief  用初始值设定运行时基线跟踪器。
 * @param  baseline    指向基线结构的指针。
 * @param  baseline_ir 初始跟踪的背景 IR 值。
 * @param  noise_ir    初始背景噪声估计值。
 * @note   在启动时背景采样完成后调用。
 *         noise_ir 被限制至少为 1，以避免除零错误。
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

/**
 * @brief  在确认手指移开后持续跟踪背景 IR。
 * @param  baseline 指向基线结构的指针。
 * @param  ir_value 当前 IR 样本值。
 * @note   使用慢跟踪滤波器：tracked_ir 缓慢跟随（右移 4 位），
 *         noise_ir 稍快跟随（右移 3 位）。这使得基线能够
 *         随环境变化漂移，而不会对短时突发噪声做出反应。
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

/**
 * @brief  获取当前运行时动态背景 IR 基线。
 * @param  baseline 指向基线结构的指针。
 * @return 跟踪的 IR 基线；如果尚未跟踪则返回平均 IR。
 */
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

/**
 * @brief  获取当前运行时背景噪声估计值。
 * @param  baseline 指向基线结构的指针。
 * @return 噪声 IR 估计值。
 */
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline)
{
  if (baseline == NULL)
  {
    return 0U;
  }

  return baseline->noise_ir;
}

/**
 * @brief  使用双阈值迟滞检测手指存在状态。
 * @param  ir_value         当前 IR 样本值。
 * @param  baseline_ir      当前背景 IR 基线。
 * @param  current_state    前一次手指状态（0 = 移开，1 = 按下）。
 * @param  finger_on_delta  手指按下的检测阈值增量。
 * @param  finger_off_delta 手指移开的检测阈值增量。
 * @return 新手指状态（0 = 移开，1 = 按下）。
 * @note   迟滞可防止在阈值边界附近因噪声导致
 *         反复的状态切换。使用独立的按下/移开增量值。
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
