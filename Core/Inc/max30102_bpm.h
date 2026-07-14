/**
  ******************************************************************************
  * @file    max30102_bpm.h
  * @brief   MAX30102 BPM 峰值检测 + 自相关心率估算
  ******************************************************************************
  */

#ifndef __MAX30102_BPM_H__
#define __MAX30102_BPM_H__

#include <stdint.h>
#include "max30102_spo2.h"  /* 用于 MAX30102_SpO2_t */

#define MAX30102_BPM_MIN_VALID_SAMPLES        40U

/*
 * 流式脉冲检测的输出信息：
 * - 由 max30102_calculate_bpm_with_pulse 从窗口峰值中提取最近一次有效峰距，
 *   供上层 app_measurement 的 app_stream_pulse_update 做 IBI/HRV 输入和交叉校验。
 */
typedef struct
{
  uint8_t beat_valid;          /* 最近峰间隔通过窗口算法门控 */
  uint16_t interval_samples;   /* 最近两个有效峰之间的样本数 */
  uint16_t latest_ibi_ms;      /* 按 100 Hz 换算的最近 IBI，单位：ms */
  uint32_t latest_peak_sample; /* 最近峰在全局样本序列中的编号 */
  uint32_t beat_amplitude;     /* 最近峰相对局部基线的幅度 */
} MAX30102_PulseInfo_t;

/** @brief 基于当前窗口峰间隔估算 BPM；成功时返回 1。 */
uint8_t max30102_calculate_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm_value);
/** @brief 同时返回窗口 BPM 与最近逐拍间隔/幅度信息。 */
uint8_t max30102_calculate_bpm_with_pulse(const MAX30102_SpO2_t *spo2_state,
                                          uint8_t *bpm_value,
                                          MAX30102_PulseInfo_t *pulse_info);
/** @brief 用 IR AC 窗口自相关提供独立 BPM 估计，供上层交叉校验。 */
uint8_t max30102_autocorr_bpm(const MAX30102_SpO2_t *spo2_state, uint8_t *bpm);

#endif /* __MAX30102_BPM_H__ */
