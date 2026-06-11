/**
  ******************************************************************************
  * @file    app_ecg.h
  * @brief   ECG QRS 检测 — 250 Hz 定时器触发采样，逐样本消费
  *
  * AD8232 → PA5/ADC1_IN5 → TIM2 250 Hz → DMA 环形缓冲 → 本模块逐样本消费。
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

/* === ECG 调试显示模式 ====================================================== */
/* 设为 1 以在 OLED 上直接显示 raw ADC 波形 (raw_value - 2048)，绕过所有显示滤波。
 * 仅用于验证 AD8232 模拟前端输出是否正常到达 ADC。默认为 0（关闭）。 */
#define APP_ECG_DEBUG_DISPLAY_RAW      0U
/* 设为 1 以在 OLED 上显示滤波后的 ECG 波形 (app->ecg_filtered)，绕过显示端
 * 5 点滑动平均、振幅跟踪和软限幅。仅在 APP_ECG_DEBUG_DISPLAY_RAW=0 时生效。 */
#define APP_ECG_DEBUG_DISPLAY_FILTERED 0U
/* 设为 1 以使用独立视觉优化滤波链 (ecg_visual) 驱动 OLED 波形显示。
 * 仅在 RAW=0 且 FILTERED=0 时生效。默认开启，关闭则回退到原始 NORMAL 显示链路。 */
#define APP_ECG_DEBUG_DISPLAY_VISUAL   1U
/* 设为 1 以每 250 个 ECG 样本通过 huart2 输出一次调试统计行。
 * 注意：这会占用 UART2 约 4 ms/次，可能与串口协议帧冲突。默认为 0。 */
#define APP_ECG_DEBUG_PRINTF           0U

/* 导联脱落掩码：LO- = 红色电极，LO+ = 绿色电极。
 * 黄色 RL/RLD 电极未通过 LO 引脚连接——移除它
 * 不会改变导联脱落标志。此为硬件设计预期。 */
#define APP_ECG_LEAD_OFF_MINUS 0x01U
#define APP_ECG_LEAD_OFF_PLUS  0x02U

/* AD8232 LO- / LO+ 引脚分配。
 * LO- → PE5（红色电极），LO+ → PE6（绿色电极）。
 * PE4 为翻页按键——此处保持未使用。 */
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

/* ECG debug snapshot for D8 OLED page — read-only, no side effects.
 * Includes PPG fields for ECG-vs-PPG comparison on a single screen. */
typedef struct
{
  uint16_t raw_min;
  uint16_t raw_max;
  int16_t  filt_min;
  int16_t  filt_max;
  uint8_t  lead_raw;
  uint8_t  ecg_valid;
  uint8_t  ecg_hr;
  uint16_t ecg_rr_ms;
  uint8_t  ppg_valid;
  uint8_t  ppg_bpm;
  int16_t  hr_diff;    /* ecg_hr - ppg_bpm, valid only when both ecg_valid && ppg_valid */
  uint32_t sample_count;
  uint32_t dma_overflow_count;
  uint32_t adc_sat_count;
  uint32_t lead_off_count;
  uint32_t no_r_peak_timeout_count;
} AppEcgDebugSnapshot_t;

/* 重置 ECG 检测器内部状态与 AppState ECG 字段 */
void app_ecg_reset(AppState_t *app);

/* 读取 AD8232 LO+/LO- 导联脱落引脚电平，返回掩码 */
uint8_t app_ecg_read_lead_off(void);
uint8_t app_ecg_read_lead_off_raw(void);

/* 填充 ECG 调试快照，供 OLED debug 页面只读展示。
 * out 不可为 NULL。不阻塞，不 printf，不访问硬件寄存器。 */
void app_ecg_get_debug_snapshot(const AppState_t *app,
                                AppEcgDebugSnapshot_t *out);

/* 消耗 DMA 缓冲区中所有待处理样本，推进 ECG 流水线。
 * 应在主循环每 10 ms 节拍调用一次。
 * 返回非零表示本轮检测到至少一个有效 R 峰。 */
uint8_t app_ecg_process_samples(AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ECG_H__ */
