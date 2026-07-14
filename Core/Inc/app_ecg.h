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
/* 设为 1 以在 OLED 上直接显示原始 ADC 波形 (raw_value - 2048)，绕过所有显示滤波。
 * 仅用于验证 AD8232 模拟前端输出是否正常到达 ADC。默认为 0（关闭）。 */
#define APP_ECG_DEBUG_DISPLAY_RAW      0U
/* 设为 1 以在 OLED 上显示滤波后的 ECG 波形 (app->ecg_filtered)，绕过显示端
 * 5 点滑动平均、振幅跟踪和软限幅。仅在 APP_ECG_DEBUG_DISPLAY_RAW=0 时生效。 */
#define APP_ECG_DEBUG_DISPLAY_FILTERED 0U
/* 设为 1 以使用独立视觉优化滤波链 (ecg_visual) 驱动 OLED 波形显示。
 * 仅在 RAW=0 且 FILTERED=0 时生效。默认开启，关闭则回退到原始 NORMAL 显示链路。 */
#define APP_ECG_DEBUG_DISPLAY_VISUAL   1U
/* 设为 1 以启用 DSP 预处理：50 Hz 陷波 + 10-20 Hz 带通 biquad 级联。
 * 使用单精度浮点 Direct Form I，由目标 Cortex-M4F 的硬件 FPU 执行。
 * 设为 0 则回退到纯整数一阶 DC+LP 链路。 */
#define APP_ECG_DSP_PREPROCESS         1U

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

/* 二阶 IIR Biquad Direct Form I 结构（单精度浮点，Cortex-M4F FPU 优化）。
 * 差分方程：y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
 * 接口与 CMSIS-DSP arm_biquad_casd_df1_inst_f32 兼容，方便未来替换。 */
typedef struct
{
  float b0, b1, b2;   /* 前馈系数（已归一化，a0=1 隐含） */
  float a1, a2;        /* 反馈系数（标准符号，差分方程中系 -a1*y1 - a2*y2） */
  float x1, x2;        /* 输入延迟线 x[n-1], x[n-2] */
  float y1, y2;        /* 输出延迟线 y[n-1], y[n-2] */
} EcgBiquad_t;

/* D8 OLED 页面使用的 ECG 调试快照：只读、无副作用。
 * 包含 PPG 字段，便于在同一屏比较 ECG 与 PPG。 */
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
  int16_t  hr_diff;    /* ecg_hr - ppg_bpm，仅在 ecg_valid && ppg_valid 时有效 */
  uint32_t sample_count;
  uint32_t dma_overflow_count;
  uint32_t adc_sat_count;
  uint32_t lead_off_count;
  uint32_t no_r_peak_timeout_count;
  /* ECG 质量快照字段 */
  uint8_t  signal_quality;
  uint8_t  invalid_reason;
  uint16_t raw_span;
  uint16_t filtered_span;
  uint32_t noise_level;
  uint32_t qrs_threshold;
  uint16_t peak_snr_x100;
  uint16_t dma_available_high_watermark;
} AppEcgDebugSnapshot_t;

/* 重置 ECG 检测器内部状态与 AppState ECG 字段 */
void app_ecg_reset(AppState_t *app);

/* 读取 AD8232 LO+/LO- 导联脱落引脚电平，返回掩码 */
uint8_t app_ecg_read_lead_off(void);
uint8_t app_ecg_read_lead_off_raw(void);

/* 填充 ECG 调试快照，供 OLED 调试页面只读展示。
 * out 不可为 NULL。不阻塞，不 printf，不访问硬件寄存器。 */
void app_ecg_get_debug_snapshot(const AppState_t *app,
                                AppEcgDebugSnapshot_t *out);

/* 消耗 DMA 缓冲区中所有待处理样本，推进 ECG 流水线。
 * 由 MAXtask 在 TIM6 10 ms 节拍或通知超时唤醒后调用。
 * 返回非零表示本轮检测到至少一个有效 R 峰。 */
uint8_t app_ecg_process_samples(AppState_t *app);

/* 更新 AppState ECG quality 字段，基于当前检测器状态和调试统计。
 * 在每个 process 批次末尾调用一次。 */
void app_ecg_update_quality(AppState_t *app);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ECG_H__ */
