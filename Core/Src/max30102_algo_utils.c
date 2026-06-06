/**
  ******************************************************************************
  * @file    max30102_algo_utils.c
  * @brief   MAX30102 算法共享工具函数
  ******************************************************************************
  */

#include "max30102_algo_utils.h"
#include <math.h>
#include <string.h>

/*
 * arm_bitreversal_32 的纯 C 替代实现。
 *
 * CMSIS-DSP 原版使用汇编（GAS 语法），ARMCLANG v6 / armasm 无法汇编。
 * 本函数提供等效功能：对交错的复数缓冲区进行 in-place 位逆序重排。
 *
 * 关键细节：CMSIS-DSP 的位逆序表 pBitRevTable 存储的是"字节偏移"而非
 * "复数样本索引"。因此需要用 uint8_t* 基址 + 字节偏移来定位待交换的
 * 复数对（每个复数 = 2 个 float32 = 8 字节）。
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

/*
 * 用整数形式做一个慢速一阶低通：
 * - shift 越大，跟随越慢
 * - 至少移动 1 个计数，避免小误差长期卡住不更新
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

/*
 * STM32F407 has an M4F core, so using `sqrtf` here is cheaper than walking a
 * full software integer square root for the medium-sized energy terms used by
 * the algorithm windows. Keep the integer fallback for portability.
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

uint64_t max30102_square_u32(uint32_t value)
{
  return (uint64_t)value * (uint64_t)value;
}

uint64_t max30102_square_i32(int32_t value)
{
  return (uint64_t)((int64_t)value * (int64_t)value);
}

uint32_t max30102_calculate_window_rms(uint64_t square_sum, uint16_t sample_count)
{
  if (sample_count == 0U)
  {
    return 0U;
  }

  return max30102_fast_sqrt_u64(square_sum / sample_count);
}

/*
 * Compute the centered RMS from the exact integer variance numerator
 * `n * sum(x^2) - sum(x)^2`, then let the FPU do the final square root. This
 * avoids the large-DC cancellation problem of direct float variance.
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
