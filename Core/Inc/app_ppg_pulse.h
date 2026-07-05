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

/* 重置峰值检测器内部状态 */
void app_ppg_pulse_reset(void);

/* 每收到一个滤波后样本调用一次，返回 1=检测到有效脉搏 */
uint8_t app_ppg_pulse_update(AppState_t *app,
                             int32_t filtered_sample,
                             uint32_t total_samples,
                             MAX30102_PulseInfo_t *pulse_info);

/* 脉搏后处理：推送 IBI 到 HRV、振幅到 RR、触发 OLED 标记 */
uint8_t app_ppg_pulse_process_metrics(AppState_t *app,
                                      const MAX30102_PulseInfo_t *pulse_info,
                                      uint32_t total_samples);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_PULSE_H__ */
