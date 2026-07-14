/**
  ******************************************************************************
  * @file    app_bpm_filter.h
  * @brief   BPM 输出滤波器 — 多级确认状态机
  *
  * 对逐拍原始心率值进行中值滤波、候选确认与平滑输出。
  * 支持分层阈值判断（小幅/中等/大幅变化），防止干扰导致的 BPM 跳变。
  ******************************************************************************
  */
#ifndef __APP_BPM_FILTER_H__
#define __APP_BPM_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/** @brief 清零 BPM 历史、候选状态和有效标志；手指离开或持续无效时调用。 */
void app_bpm_filter_reset(AppState_t *app);

/**
 * @brief  对原始逐拍心率执行三点中值、候选确认、EMA 与步进限幅。
 * @param  app 共享应用状态，可为 NULL。
 * @param  raw_bpm_valid 原始值可信时为 1，否则进入短时保持路径。
 * @param  raw_bpm_value 原始心率，单位：bpm。
 * @return 正常跟踪/保持返回 0；连续无效达到保持上限并清零 BPM 后返回 1。
 */
uint8_t app_bpm_filter_update(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BPM_FILTER_H__ */
