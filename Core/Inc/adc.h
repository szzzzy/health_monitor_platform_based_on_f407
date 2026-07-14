/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define APP_ECG_ADC_BUF_SIZE 256U  /* ECG DMA 环形缓冲容量：约 1.024 s @ 250 Hz */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
/** @brief 启动 TIM2 触发的 ADC1 环形 DMA 采集。 */
void app_ecg_adc_start(void);
/** @brief 返回 DMA 当前写索引，并同步软件生产计数。 */
uint16_t app_ecg_adc_get_write_index(void);
/** @brief 返回 ECG 算法尚未消费的样本数。 */
uint16_t app_ecg_adc_get_available_count(void);
/**
 * @brief  读取一个待处理样本并按批次位置回推采样时间。
 * @return 成功返回 1，无数据或 raw_value 为 NULL 返回 0。
 */
uint8_t app_ecg_adc_read_sample(uint16_t *raw_value,
                                uint32_t *timestamp_ms,
                                uint32_t now_ms,
                                uint16_t avail_remaining);
/** @brief 查询/清除 DMA 数据覆盖锁存标志。 */
uint8_t app_ecg_adc_had_overflow(void);
void app_ecg_adc_clear_overflow(void);
/** @brief 获取 ADC/DMA 调试寄存器快照；仅用于诊断页面。 */
uint16_t app_ecg_adc_debug_ndtr(void);
uint32_t app_ecg_adc_debug_adc_state(void);
uint32_t app_ecg_adc_debug_adc_cr2(void);
uint32_t app_ecg_adc_debug_dma_cr(void);
uint32_t app_ecg_adc_debug_dma_lisr(void);
uint32_t app_ecg_adc_debug_dma_errcode(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
