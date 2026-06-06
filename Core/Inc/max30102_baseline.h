/**
  ******************************************************************************
  * @file    max30102_baseline.h
  * @brief   MAX30102 背景基线统计与手指检测
  ******************************************************************************
  */

#ifndef __MAX30102_BASELINE_H__
#define __MAX30102_BASELINE_H__

#include <stddef.h>
#include <stdint.h>

/* 用于统计背景 IR 基线 */
typedef struct
{
  uint32_t ir_sum;
  uint32_t ir_min;
  uint32_t ir_max;
  uint32_t tracked_ir;
  uint32_t noise_ir;
  uint16_t sample_count;
} MAX30102_Baseline_t;

void     max30102_baseline_reset(MAX30102_Baseline_t *baseline);
void     max30102_baseline_add_ir(MAX30102_Baseline_t *baseline, uint32_t ir_value);
uint8_t  max30102_baseline_is_ready(const MAX30102_Baseline_t *baseline, uint16_t target_samples);
uint32_t max30102_baseline_get_average_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_range_ir(const MAX30102_Baseline_t *baseline);
uint8_t  max30102_baseline_is_stable(const MAX30102_Baseline_t *baseline, uint32_t stable_range);
void     max30102_baseline_seed_tracking(MAX30102_Baseline_t *baseline,
                                         uint32_t baseline_ir,
                                         uint32_t noise_ir);
void     max30102_baseline_track_background(MAX30102_Baseline_t *baseline, uint32_t ir_value);
uint32_t max30102_baseline_get_tracked_ir(const MAX30102_Baseline_t *baseline);
uint32_t max30102_baseline_get_noise_ir(const MAX30102_Baseline_t *baseline);
uint8_t  max30102_detect_finger(uint32_t ir_value,
                                uint32_t baseline_ir,
                                uint8_t current_state,
                                uint32_t finger_on_delta,
                                uint32_t finger_off_delta);

#endif /* __MAX30102_BASELINE_H__ */
