/**
  ******************************************************************************
  * @file    max30102_bpm.h
  * @brief   MAX30102 BPM 峰值检测 + 自相关心率估算
  ******************************************************************************
  */

#ifndef __MAX30102_BPM_H__
#define __MAX30102_BPM_H__

#include <stdint.h>
#include "max30102_spo2.h"  /* for MAX30102_SpO2_t */

#define MAX30102_BPM_MIN_VALID_SAMPLES        40U

/*
 * 流式脉冲检测的输出信息：
 * - 由 max30102_calculate_bpm_with_pulse 从窗口峰值中提取最近一次有效峰距，
 *   供上层 app_measurement 的 app_stream_pulse_update 做 IBI/HRV 输入和交叉校验。
 */
typedef struct
{
  uint8_t beat_valid;
  uint16_t interval_samples;
  uint16_t latest_ibi_ms;
  uint32_t latest_peak_sample;
  uint32_t beat_amplitude;
} MAX30102_PulseInfo_t;

uint8_t max30102_calculate_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm_value);
uint8_t max30102_calculate_bpm_with_pulse(const MAX30102_SpO2_t *spo2_state,
                                          uint8_t *bpm_value,
                                          MAX30102_PulseInfo_t *pulse_info);
uint8_t max30102_autocorr_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm);

#endif /* __MAX30102_BPM_H__ */
