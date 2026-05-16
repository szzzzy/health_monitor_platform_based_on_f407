/**
  ******************************************************************************
  * @file    app_oxy_status.h
  * @brief   血氧信号质量评估与平衡状态判定
  *
  * 汇总 AC RMS、灌注指数、信号质量评分，
  * 计算 SpO2 比率 (Red/IR AC/DC 比) 并做 EMA 平滑，
  * 根据比率判定传感器平衡状态（正常/偏低/偏高）。
  ******************************************************************************
  */
#ifndef __APP_OXY_STATUS_H__
#define __APP_OXY_STATUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"
#include "max30102.h"

/* 重置所有信号质量与平衡状态字段 */
void app_oxy_status_reset(AppState_t *app);

/* 即时清除（传感器异常时调用） */
void app_oxy_status_clear_instant(AppState_t *app);

/* 从传感器指标结构体更新所有信号质量字段 */
void app_oxy_status_update_from_metrics(AppState_t *app,
                                        const MAX30102_SignalMetrics_t *metrics,
                                        uint8_t raw_quality);

/* 信号质量平滑更新（非对称步进） */
void app_oxy_status_update_quality(AppState_t *app, uint8_t raw_quality);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OXY_STATUS_H__ */
