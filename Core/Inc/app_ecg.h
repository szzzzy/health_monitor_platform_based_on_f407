/**
  ******************************************************************************
  * @file    app_ecg.h
  * @brief   ECG QRS 检测 — 250 Hz 定时器触发采样，逐样本消费
  *
  * AD8232 → PC0/ADC1_IN10 → TIM3 250 Hz → DMA 环形缓冲 → 本模块逐样本消费。
  * 简化 Pan-Tompkins：DC 漂移消除 + 低通平滑 + 自适应阈值 QRS 状态机。
  * 检测到 R 峰后通过 AppEcgUpdate_t 上报，同时通知 PTT 模块。
  * 此为工程观测/趋势提示，不声称临床诊断能力。
  ******************************************************************************
  */
#ifndef __APP_ECG_H__
#define __APP_ECG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

/* 导联脱落状态标志 */
#define APP_ECG_LEAD_OFF_MINUS 0x01U
#define APP_ECG_LEAD_OFF_PLUS  0x02U

/* AD8232 LO- / LO+ 引脚定义
 * LO- → PE5, LO+ → PE6。PE4 是翻页按钮，不得占用。 */
#define AD8232_LD_MINUS_Pin       GPIO_PIN_5
#define AD8232_LD_MINUS_GPIO_Port GPIOE
#define AD8232_LD_PLUS_Pin        GPIO_PIN_6
#define AD8232_LD_PLUS_GPIO_Port  GPIOE

/* ECG 更新结果 */
typedef struct
{
  uint8_t  r_peak_detected;
  uint32_t r_peak_ms;
  uint16_t rr_ms;
  uint8_t  hr_bpm;
} AppEcgUpdate_t;

/* 重置 ECG 检测器内部状态与 AppState ECG 字段 */
void app_ecg_reset(AppState_t *app);

/* 读取 AD8232 LO+/LO- 导联脱落引脚电平，返回掩码 */
uint8_t app_ecg_read_lead_off(void);

/* 消耗 DMA 缓冲区中所有待处理样本，推进 ECG 流水线。
 * 应在主循环每 10 ms 节拍调用一次。
 * 返回非零表示本轮检测到至少一个有效 R 峰。 */
uint8_t app_ecg_process_samples(AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ECG_H__ */
