/**
  ******************************************************************************
  * @file    app_ppg_signal.h
  * @brief   PPG 信号包络追踪、自适应手指检测与背景基线维护
  *
  * 维护 IR/RED 通道的高低包络线（慢跟随衰减），
  * 基于背景噪声动态调整手指检测阈值（滞回控制），
  * 判定原始 PPG 信号的存在性，并将样本送入背景基线跟踪器。
  ******************************************************************************
  */
#ifndef __APP_PPG_SIGNAL_H__
#define __APP_PPG_SIGNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

/* 手指状态确认计数阈值 */
#define APP_PPG_SIGNAL_FINGER_ON_CONFIRM_COUNT  8U    /* 就位需连续确认 8 拍 */
#define APP_PPG_SIGNAL_FINGER_OFF_CONFIRM_COUNT 75U   /* 离开需连续确认 75 拍 (750ms) */
#define APP_PPG_SIGNAL_REACQUIRE_NOISE_IR       3000UL /* 重新采集背景的噪声阈值 */

/* 初始化自适应手指检测阈值为默认值 */
void app_ppg_signal_init_state(AppState_t *app);

/* 重置信号包络（手指状态切换时调用） */
void app_ppg_signal_reset_envelope(void);

/* 更新 IR/RED 通道包络与活动指标 (delta, span) */
void app_ppg_signal_update_activity(AppState_t *app);

/* 基于背景噪声动态更新手指检测 on/off 阈值 */
void app_ppg_signal_update_adaptive_thresholds(AppState_t *app,
                                               const MAX30102_Baseline_t *baseline);

/* 判定原始信号是否存在（IR 偏移 > 自适应阈值，带滞回） */
uint8_t app_ppg_signal_is_raw_present(const AppState_t *app);

/* 将当前 IR 值送入基线跟踪器，更新跟踪基线 */
void app_ppg_signal_track_background_ir(AppState_t *app, MAX30102_Baseline_t *baseline);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PPG_SIGNAL_H__ */
