/**
  ******************************************************************************
  * @file    app_ppg_pulse.h
  * @brief   PPG 时域脉搏波峰检测
  *
  * 在带通滤波后的 PPG 信号上进行逐点峰值检测，
  * 提取逐拍间隔 (IBI)，推送至 HRV/RR 模块。
  * 采用自适应阈值、突出度检验和极性交替约束。
  ******************************************************************************
  */
#ifndef __APP_PPG_PULSE_H__
#define __APP_PPG_PULSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

/** @brief 清零流式峰/谷状态、幅度 EMA 与 IBI 短历史。 */
void app_ppg_pulse_reset(void);

/**
 * @brief  向状态机输入一个带通 PPG 样本，检测峰值并生成逐拍信息。
 * @return 通过间期、幅度与质量门控的脉搏返回 1，否则返回 0。
 */
uint8_t app_ppg_pulse_update(AppState_t *app,
                             int32_t filtered_sample,
                             uint32_t current_sample,
                             MAX30102_PulseInfo_t *pulse_info);

/**
 * @brief  对已检测脉搏执行 SQI 门控，并把 IBI/幅度送入 HRV、RR 和显示标记。
 * @return 本次脉搏指标被接受返回 1，被质量门控拒绝返回 0。
 */
uint8_t app_ppg_pulse_process_metrics(AppState_t *app,
                                      const MAX30102_PulseInfo_t *pulse_info,
                                      uint32_t current_sample);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_PULSE_H__ */
