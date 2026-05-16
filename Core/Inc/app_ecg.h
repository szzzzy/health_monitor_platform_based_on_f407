/**
  ******************************************************************************
  * @file    app_ecg.h
  * @brief   ECG（心电图）信号采集与 QRS 检测模块
  *
  * 从 AD8232 模拟前端通过 ADC1 采集单导联 ECG 信号，软件实现：
  * - DC 漂移消除
  * - 滑动平均平滑
  * - 自适应阈值 QRS 波群检测（Pan-Tompkins 简化版）
  * - 导联脱落监测
  *
  * 检测到 R 峰后通过 AppEcgUpdate_t 上报瞬时 RR 间期与心率，
  * 同时通知 PTT 模块记录 ECG 参考时间戳。
  ******************************************************************************
  */
#ifndef __APP_ECG_H__
#define __APP_ECG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 导联脱落状态标志位：LA 电极脱落 / RA 电极脱落 */
#define APP_ECG_LEAD_OFF_MINUS 0x01U
#define APP_ECG_LEAD_OFF_PLUS  0x02U

/* ECG 更新结果：包含本次是否检测到 R 峰、峰值时刻、RR 间期与心率 */
typedef struct
{
  uint8_t r_peak_detected;   /* 1 = 本拍检测到有效 R 峰 */
  uint32_t r_peak_ms;        /* 当前 R 峰时间戳 (ms) */
  uint16_t rr_ms;            /* 与前一个 R 峰的 RR 间期 (ms) */
  uint8_t hr_bpm;            /* 瞬时心率 (bpm)，含 EMA 平滑 */
} AppEcgUpdate_t;

/* 重置 ECG 状态与检测器，回到上电初始状态 */
void app_ecg_reset(AppState_t *app);

/* 软件触发一次 ADC1 单次转换，读取 ADC 原始值 */
uint8_t app_ecg_read_adc_raw(uint16_t *raw_value);

/* 读取 AD8232 导联脱落指示引脚，返回 APP_ECG_LEAD_OFF_xxx 掩码 */
uint8_t app_ecg_read_lead_off(void);

/* 每拍调用一次：读取 ADC → 消除 DC → 平滑 → QRS 检测 → 更新心率 */
AppEcgUpdate_t app_ecg_update(AppState_t *app, uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ECG_H__ */
