/**
  ******************************************************************************
  * @file    adc.h
  * @brief   ADC1 配置：TIM3 触发 + DMA 环形缓冲，250 Hz ECG 采样
  *
  * PC0 (ADC1_IN10) — AD8232 ECG 模拟前端输出。
  * TIM3 TRGO 250 Hz 上升沿触发 ADC1，DMA2_Stream0 循环搬移。
  * 主循环每 10 ms 从环形缓冲消费累计样本，不阻塞轮询。
  *
  * 覆盖检测通过 produced/consumed 绝对计数实现，不依赖环形缓冲指针相等比较。
  ******************************************************************************
  */
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* DMA 环形缓冲区长度 (uint16_t × BUF_SIZE)，250 Hz ≈ 1.0 s 窗口 */
#define APP_ECG_ADC_BUF_SIZE  256U

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);

/* 启动 TIM3 和 ADC1 DMA，开始连续采样 */
void app_ecg_adc_start(void);

/* 当前 DMA 写入位置 (0..BUF_SIZE-1) */
uint16_t app_ecg_adc_get_write_index(void);

/* 当前可供消费的样本数 */
uint16_t app_ecg_adc_get_available_count(void);

/*
 * 从环形缓冲区消费一个样本及其推算时间戳。
 * now_ms: 调用者在批量消费开始前取的 HAL_GetTick()。
 * avail_remaining: 批量消费开始前的可用样本数（调用者自行递减）。
 * timestamp_ms: 输出推算的样本绝对时间戳 (ms, HAL Tick 基准)。
 * 返回 1=成功读取，0=无新数据或被覆盖的旧数据已丢弃。
 */
uint8_t app_ecg_adc_read_sample(uint16_t *raw_value,
                                uint32_t *timestamp_ms,
                                uint32_t now_ms,
                                uint16_t avail_remaining);

/* 检查 DMA 是否发生过溢出/覆盖 */
uint8_t app_ecg_adc_had_overflow(void);
void    app_ecg_adc_clear_overflow(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
