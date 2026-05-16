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

/* 重置滤波状态（手指离开或持续无效时调用） */
void app_bpm_filter_reset(AppState_t *app);

/* 输入原始心率值，返回 0=正常跟踪, 1=BPM 已清零 */
uint8_t app_bpm_filter_update(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BPM_FILTER_H__ */
