/**
  ******************************************************************************
  * @file    max30102_spo2.h
  * @brief   MAX30102 SpO2 窗口、信号质量、血氧计算
  ******************************************************************************
  */

#ifndef __MAX30102_SPO2_H__
#define __MAX30102_SPO2_H__

#include <stdint.h>

#define MAX30102_SPO2_WINDOW_SIZE             256U
#define MAX30102_SPO2_MIN_VALID_SAMPLES       30U

/*
 * 算法默认按 100 Hz 采样设计。
 * 这个值应与 main.c 中的采样周期保持一致，否则 BPM 结果会按比例偏差。
 */
#define MAX30102_ALGO_SAMPLE_RATE_HZ          100U

/* SpO2 算法内部阈值 */
#define MAX30102_SPO2_MIN_DC                  1U
#define MAX30102_SPO2_MIN_AC_RMS              40U
#define MAX30102_SPO2_RATIO_SCALE             1000U
#define MAX30102_FILTER_DC_SHIFT              4U

/* 信号质量阈值 */
#define MAX30102_SIGNAL_QUALITY_IR_PI_GOOD     4U
#define MAX30102_SIGNAL_QUALITY_RED_PI_GOOD    3U
#define MAX30102_SIGNAL_QUALITY_IR_RMS_GOOD   24U
#define MAX30102_SIGNAL_QUALITY_RED_RMS_GOOD  18U
#define MAX30102_SIGNAL_QUALITY_WINDOW_GOOD   80U

/* 用于保存最近一段 RED / IR 样本，给 SpO2 算法计算 AC/DC 比值。 */
typedef struct
{
  uint32_t red_samples[MAX30102_SPO2_WINDOW_SIZE];
  uint32_t ir_samples[MAX30102_SPO2_WINDOW_SIZE];
  int32_t red_filtered_samples[MAX30102_SPO2_WINDOW_SIZE];
  int32_t ir_filtered_samples[MAX30102_SPO2_WINDOW_SIZE];
  uint16_t write_index;
  uint16_t sample_count;
  uint32_t red_dc_estimate;
  uint32_t ir_dc_estimate;
  int32_t red_filter_state;
  int32_t ir_filter_state;
  uint64_t red_sum;
  uint64_t ir_sum;
  uint64_t red_square_sum;
  uint64_t ir_square_sum;
  uint64_t red_filtered_square_sum;
  uint64_t ir_filtered_square_sum;
  uint32_t total_samples;
} MAX30102_SpO2_t;

typedef struct
{
  uint32_t red_dc;
  uint32_t ir_dc;
  uint32_t red_ac_rms;
  uint32_t ir_ac_rms;
  uint16_t red_pi_x1000;
  uint16_t ir_pi_x1000;
} MAX30102_SignalMetrics_t;

void     max30102_spo2_reset(MAX30102_SpO2_t *spo2_state);
void     max30102_spo2_add_sample(MAX30102_SpO2_t *spo2_state, uint32_t red_value, uint32_t ir_value);
uint8_t  max30102_spo2_get_latest_filtered(const MAX30102_SpO2_t *spo2_state,
                                           int32_t *red_filtered,
                                           int32_t *ir_filtered);
uint8_t  max30102_calculate_spo2(const MAX30102_SpO2_t *spo2_state, uint8_t *spo2_value);
uint8_t  max30102_get_signal_metrics(const MAX30102_SpO2_t *spo2_state, MAX30102_SignalMetrics_t *metrics);
uint8_t  max30102_calculate_signal_quality(const MAX30102_SpO2_t *spo2_state,
                                           const MAX30102_SignalMetrics_t *metrics,
                                           uint8_t *signal_quality);

#endif /* __MAX30102_SPO2_H__ */
