/**
  ******************************************************************************
  * @file    max30102_algo_utils.h
  * @brief   MAX30102 算法共享工具函数（sqrt、RMS、慢跟随等）
  ******************************************************************************
  */

#ifndef __MAX30102_ALGO_UTILS_H__
#define __MAX30102_ALGO_UTILS_H__

#include <stdint.h>

/** @brief 以 1/2^shift 的步长让无符号值缓慢逼近目标。 */
uint32_t max30102_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);
/** @brief 返回 64 位无符号数平方根的向下取整值。 */
uint32_t max30102_isqrt_u64(uint64_t value);
/** @brief 平方根快速入口；当前实现保持与精确整数平方根相同的整数语义。 */
uint32_t max30102_fast_sqrt_u64(uint64_t value);
/** @brief 将 value/target 线性映射到不超过 full_score 的分数。 */
uint8_t  max30102_scale_score_u32(uint32_t value, uint32_t target, uint8_t full_score);
/** @brief 以 64 位结果计算无符号/有符号 32 位值的平方。 */
uint64_t max30102_square_u32(uint32_t value);
uint64_t max30102_square_i32(int32_t value);
/** @brief 根据平方和计算窗口 RMS。 */
uint32_t max30102_calculate_window_rms(uint64_t square_sum, uint16_t sample_count);
/** @brief 根据和、平方和计算去均值后的窗口 RMS。 */
uint32_t max30102_calculate_centered_rms(uint64_t sum, uint64_t square_sum, uint16_t sample_count);

/*
 * arm_bitreversal_32 的纯 C 替代实现。
 *
 * CMSIS-DSP 原版使用汇编（GAS 语法），ARMCLANG v6 / armasm 无法汇编。
 * 本函数提供等效功能：对交错的复数缓冲区进行原地位逆序重排。
 */
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLength, const uint16_t *pBitRevTable);

#endif /* __MAX30102_ALGO_UTILS_H__ */
