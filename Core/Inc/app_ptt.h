/**
  ******************************************************************************
  * @file    app_ptt.h
  * @brief   PTT（脉搏传导时间）计算模块
  *
  * PTT 定义为 ECG R 峰到 PPG 脉搏波峰之间的时间差，反映动脉硬度与血压趋势。
  *
  * 本模块维护一个最近的 ECG R 峰时间戳环形缓冲区，
  * 当 PPG 模块检测到脉搏波峰时，查找对应的 ECG 参考 R 峰，
  * 计算 PTT = PPG 波峰时间 - ECG R 峰时间。
  *
  * PTT 有效范围：60-600 ms，超出范围的结果被标记为无效。
  ******************************************************************************
  */
#ifndef __APP_PTT_H__
#define __APP_PTT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 重置 PTT 状态与 ECG 峰值历史 */
void app_ptt_reset(AppState_t *app);

/* 将新的 ECG R 峰时间戳写入历史环形缓冲区 */
void app_ptt_add_ecg_peak(uint32_t r_peak_ms);

/* PPG 波峰到达时调用：查找匹配的 ECG R 峰，计算 PTT 并更新 AppState */
void app_ptt_update_from_ppg_peak(AppState_t *app, uint32_t ppg_peak_sample, uint32_t ppg_total_samples);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PTT_H__ */
