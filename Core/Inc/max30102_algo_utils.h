/**
  ******************************************************************************
  * @file    max30102_algo_utils.h
  * @brief   MAX30102 算法共享工具函数（sqrt、RMS、慢跟随等）
  ******************************************************************************
  */

#ifndef __MAX30102_ALGO_UTILS_H__
#define __MAX30102_ALGO_UTILS_H__

#include <stdint.h>

uint32_t max30102_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);
uint32_t max30102_isqrt_u64(uint64_t value);
uint32_t max30102_fast_sqrt_u64(uint64_t value);
uint8_t  max30102_scale_score_u32(uint32_t value, uint32_t target, uint8_t full_score);
uint64_t max30102_square_u32(uint32_t value);
uint64_t max30102_square_i32(int32_t value);
uint32_t max30102_calculate_window_rms(uint64_t square_sum, uint16_t sample_count);
uint32_t max30102_calculate_centered_rms(uint64_t sum, uint64_t square_sum, uint16_t sample_count);

/*
 * arm_bitreversal_32 的纯 C 替代实现。
 *
 * CMSIS-DSP 原版使用汇编（GAS 语法），ARMCLANG v6 / armasm 无法汇编。
 * 本函数提供等效功能：对交错的复数缓冲区进行 in-place 位逆序重排。
 */
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLength, const uint16_t *pBitRevTable);

#endif /* __MAX30102_ALGO_UTILS_H__ */
