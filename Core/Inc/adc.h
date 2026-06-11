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
#define APP_ECG_ADC_BUF_SIZE 256U

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void app_ecg_adc_start(void);
uint16_t app_ecg_adc_get_write_index(void);
uint16_t app_ecg_adc_get_available_count(void);
uint8_t app_ecg_adc_read_sample(uint16_t *raw_value,
                                uint32_t *timestamp_ms,
                                uint32_t now_ms,
                                uint16_t avail_remaining);
uint8_t app_ecg_adc_had_overflow(void);
void app_ecg_adc_clear_overflow(void);
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
