/**
  ******************************************************************************
  * @file    app_spo2_filter.h
  * @brief   SpO2 输出 EMA 平滑滤波器接口。
 *
 * EMA 公式: 平滑值 = (旧值 * 3 + 新值 + 2) / 4（约 75% 旧值 + 25% 新值）。
 *
 * 关键语义：
 * - raw_spo2_valid==0 时不推进 EMA，app->spo2_valid 被清为 0，
 *   但 app->spo2_value 保留旧值（UI 显示旧值+"?"）。
 * - 低 SQ / 无效 / 运动均不污染 EMA 状态。
 * - app_spo2_filter_reset(): 清零内部平滑状态与 app->spo2_value（手指离开/测量重置）。
  ******************************************************************************
  */
#ifndef __APP_SPO2_FILTER_H__
#define __APP_SPO2_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 清零内部 EMA 状态与 app->spo2_value——手指离开/测量重置时调用。 */
void app_spo2_filter_reset(AppState_t *app);
/*
 * 更新 SpO2 EMA 并写入 app->spo2_value / spo2_valid。
 * raw_spo2_valid==0 -> 不推进平滑，spo2_valid 置 0，旧值保留。
 */
void app_spo2_filter_update_output(AppState_t *app,
                                   uint8_t raw_spo2_valid,
                                   uint8_t raw_spo2_value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SPO2_FILTER_H__ */
