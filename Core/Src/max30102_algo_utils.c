/**
  ******************************************************************************
  * @file    max30102_algo_utils.c
  * @brief   MAX30102 算法共享工具函数
  ******************************************************************************
  */

#include "max30102_algo_utils.h"
#include <math.h>
#include <string.h>

/**
 * @brief  CMSIS-DSP arm_bitreversal_32 的纯 C 替代实现。
 * @param  pSrc         指向交错复数缓冲区的指针（原地操作）。
 * @param  bitRevLength 位反转表的长度（条目数）。
 * @param  pBitRevTable 指向位反转查找表的指针。
 * @note   CMSIS-DSP 的汇编版本（GAS 语法）在 ARMCLANG v6 下编译失败。
 *         此 C 实现处理表中的字节偏移量，以交换复数对（每对 = 2 x float32 = 8 字节）。
 */
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLength, const uint16_t *pBitRevTable)
{
  uint16_t i;
  uint8_t *base;
  uint32_t *a;
  uint32_t *b;
  uint32_t tmp_re;
  uint32_t tmp_im;

  if ((pSrc == NULL) || (pBitRevTable == NULL))
  {
    return;
  }

  base = (uint8_t *)pSrc;
  for (i = 0U; (uint16_t)(i + 1U) < bitRevLength; i = (uint16_t)(i + 2U))
  {
    a = (uint32_t *)(base + pBitRevTable[i]);
    b = (uint32_t *)(base + pBitRevTable[i + 1U]);

    tmp_re = a[0];
    tmp_im = a[1];
    a[0] = b[0];
    a[1] = b[1];
    b[0] = tmp_re;
    b[1] = tmp_im;
  }
}

/**
 * @brief  整数运算的慢速一阶低通滤波器。
 * @param  current 当前滤波值。
 * @param  target  目标（输入）值。
 * @param  shift   步长移位量；值越大跟随越慢。
 * @return 更新后的滤波值。
 * @note   确保每次调用至少移动 1 个计数，以防止
 *         微小的持续误差导致更新停滞。
 */
uint32_t max30102_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
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

/**
 * @brief  64 位值的整数平方根（逐位算法）。
 * @param  value 输入值。
 * @return 值的整数平方根。
 * @note   纯整数实现，无需浮点运算。
 */
uint32_t max30102_isqrt_u64(uint64_t value)
{
  uint64_t bit = 1ULL << 62;
  uint64_t result = 0ULL;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0ULL)
  {
    if (value >= (result + bit))
    {
      value -= (result + bit);
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }

    bit >>= 2;
  }

  return (uint32_t)result;
}

/**
 * @brief  在可用时使用 FPU 硬件进行快速平方根运算。
 * @param  value 输入值。
 * @return 值的整数平方根。
 * @note   在 M4F 内核上使用硬件 sqrtf；当 FPU 不可用时
 *         回退到整数平方根以保证可移植性。
 */
uint32_t max30102_fast_sqrt_u64(uint64_t value)
{
  if (value == 0ULL)
  {
    return 0U;
  }

#if defined(__FPU_PRESENT) && defined(__FPU_USED) && (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  return (uint32_t)(sqrtf((float)value) + 0.5f);
#else
  return max30102_isqrt_u64(value);
#endif
}

/**
 * @brief  将值缩放到满分为 full_score 的分数，上限为 full_score。
 * @param  value      输入值。
 * @param  target     达到满分 full_score 的目标值。
 * @param  full_score 返回的最大分数。
 * @return 介于 0 和 full_score 之间的缩放分数。
 */
uint8_t max30102_scale_score_u32(uint32_t value, uint32_t target, uint8_t full_score)
{
  uint32_t scaled_score;

  if (full_score == 0U)
  {
    return 0U;
  }

  if (target == 0U)
  {
    return full_score;
  }

  scaled_score = ((value * full_score) + (target / 2U)) / target;
  if (scaled_score > full_score)
  {
    scaled_score = full_score;
  }

  return (uint8_t)scaled_score;
}

/**
 * @brief  计算 uint32_t 的平方，结果以 uint64_t 返回。
 * @param  value 输入值。
 * @return value * value（64 位）。
 */
uint64_t max30102_square_u32(uint32_t value)
{
  return (uint64_t)value * (uint64_t)value;
}

/**
 * @brief  计算 int32_t 的平方，结果以 uint64_t 返回。
 * @param  value 输入值。
 * @return value * value（64 位无符号）。
 */
uint64_t max30102_square_i32(int32_t value)
{
  return (uint64_t)((int64_t)value * (int64_t)value);
}

/**
 * @brief  从样本窗口的平方和计算 RMS。
 * @param  square_sum   样本值平方和。
 * @param  sample_count 窗口中的样本数。
 * @return RMS 值；如果 sample_count 为 0 则返回 0。
 */
uint32_t max30102_calculate_window_rms(uint64_t square_sum, uint16_t sample_count)
{
  if (sample_count == 0U)
  {
    return 0U;
  }

  return max30102_fast_sqrt_u64(square_sum / sample_count);
}

/**
 * @brief  从累加和计算去中心化（AC 耦合）RMS。
 * @param  sum          样本值总和（用于均值相减）。
 * @param  square_sum   样本值平方和。
 * @param  sample_count 窗口中的样本数。
 * @return 去中心化 RMS 值；如果 sample_count 为 0 则返回 0。
 * @note   使用精确整数方差避免直接浮点运算导致的
 *         DC 消除误差。公式：
 *         RMS = sqrt((n * sum(x^2) - sum(x)^2)) / n
 */
uint32_t max30102_calculate_centered_rms(uint64_t sum, uint64_t square_sum, uint16_t sample_count)
{
  uint64_t variance_n2;
  uint64_t sum_square;

  if (sample_count == 0U)
  {
    return 0U;
  }

  variance_n2 = square_sum * (uint64_t)sample_count;
  sum_square = sum * sum;
  if (variance_n2 <= sum_square)
  {
    return 0U;
  }

  variance_n2 -= sum_square;

#if defined(__FPU_PRESENT) && defined(__FPU_USED) && (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  return (uint32_t)((sqrtf((float)variance_n2) + ((float)sample_count * 0.5f)) /
                    (float)sample_count);
#else
  return (max30102_isqrt_u64(variance_n2) + (sample_count / 2U)) / sample_count;
#endif
}
