/**
  ******************************************************************************
  * @file    app_ptt.h
  * @brief   PTT (脉搏传导时间) — ECG R 峰 → PPG 脉搏波峰时间差
  *
  * PTT 反映动脉硬度趋势，用于无袖带血压估算的输入特征。
  * 正常成人 PTT 约 150–400 ms，本模块接受 60–600 ms。
  *
  * 统一时间基准：ECG R 峰时间戳 (HAL_GetTick ms) 与 PPG 波峰时间戳
  * (通过样本序号反推到 ms) 使用同一 HAL Tick 时钟。
  ******************************************************************************
  */
#ifndef __APP_PTT_H__
#define __APP_PTT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void app_ptt_reset(AppState_t *app);
void app_ptt_add_ecg_peak(uint32_t r_peak_ms);
void app_ptt_update_from_ppg_peak(AppState_t *app,
                                  uint32_t ppg_peak_sample,
                                  uint32_t ppg_total_samples);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PTT_H__ */
