/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include <string.h>

#define APP_ECG_ADC_SAMPLE_PERIOD_MS 4U

static uint16_t app_ecg_adc_buf[APP_ECG_ADC_BUF_SIZE];
static volatile uint16_t app_ecg_adc_consume_idx = 0U;
static volatile uint8_t app_ecg_adc_overflow_flag = 0U;

static uint32_t ecg_produced = 0U;
static uint32_t ecg_consumed = 0U;
static uint16_t ecg_prev_write_idx = 0U;

static void app_ecg_adc_track_produced(uint16_t write_idx)
{
  uint16_t delta;

  if (write_idx >= ecg_prev_write_idx)
  {
    delta = (uint16_t)(write_idx - ecg_prev_write_idx);
  }
  else
  {
    delta = (uint16_t)(APP_ECG_ADC_BUF_SIZE - ecg_prev_write_idx + write_idx);
  }

  ecg_produced += (uint32_t)delta;
  ecg_prev_write_idx = write_idx;
}

static void app_ecg_adc_drop_overwritten(void)
{
  uint32_t overwritten;

  if (ecg_produced <= (ecg_consumed + (uint32_t)APP_ECG_ADC_BUF_SIZE))
  {
    return;
  }

  overwritten = ecg_produced - ecg_consumed - (uint32_t)APP_ECG_ADC_BUF_SIZE;
  ecg_consumed += overwritten;
  app_ecg_adc_consume_idx =
      (uint16_t)((app_ecg_adc_consume_idx + (uint16_t)(overwritten % APP_ECG_ADC_BUF_SIZE)) %
                 APP_ECG_ADC_BUF_SIZE);
  app_ecg_adc_overflow_flag = 1U;
}

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void app_ecg_adc_start(void)
{
  (void)memset(app_ecg_adc_buf, 0, sizeof(app_ecg_adc_buf));
  app_ecg_adc_consume_idx = 0U;
  app_ecg_adc_overflow_flag = 0U;
  ecg_produced = 0U;
  ecg_consumed = 0U;
  ecg_prev_write_idx = 0U;

  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)app_ecg_adc_buf, APP_ECG_ADC_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint16_t app_ecg_adc_get_write_index(void)
{
  uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
  uint16_t write_idx;

  if (ndtr >= APP_ECG_ADC_BUF_SIZE)
  {
    write_idx = 0U;
  }
  else
  {
    write_idx = (uint16_t)(APP_ECG_ADC_BUF_SIZE - ndtr);
  }

  app_ecg_adc_track_produced(write_idx);
  app_ecg_adc_drop_overwritten();

  return write_idx;
}

uint16_t app_ecg_adc_get_available_count(void)
{
  uint32_t avail;

  (void)app_ecg_adc_get_write_index();

  if (ecg_produced >= ecg_consumed)
  {
    avail = ecg_produced - ecg_consumed;
  }
  else
  {
    avail = 0U;
  }

  if (avail > (uint32_t)APP_ECG_ADC_BUF_SIZE)
  {
    avail = (uint32_t)APP_ECG_ADC_BUF_SIZE;
  }

  return (uint16_t)avail;
}

uint8_t app_ecg_adc_read_sample(uint16_t *raw_value,
                                uint32_t *timestamp_ms,
                                uint32_t now_ms,
                                uint16_t avail_remaining)
{
  if (raw_value == NULL)
  {
    return 0U;
  }

  (void)app_ecg_adc_get_write_index();

  if (ecg_consumed >= ecg_produced)
  {
    return 0U;
  }

  if (timestamp_ms != NULL)
  {
    uint32_t age_ms = 0U;

    if (avail_remaining > 0U)
    {
      age_ms = ((uint32_t)avail_remaining - 1U) * APP_ECG_ADC_SAMPLE_PERIOD_MS;
    }

    *timestamp_ms = (now_ms >= age_ms) ? (now_ms - age_ms) : 0UL;
  }

  *raw_value = app_ecg_adc_buf[app_ecg_adc_consume_idx];
  app_ecg_adc_consume_idx = (uint16_t)((app_ecg_adc_consume_idx + 1U) %
                                       APP_ECG_ADC_BUF_SIZE);
  ecg_consumed++;

  return 1U;
}

uint8_t app_ecg_adc_had_overflow(void)
{
  return app_ecg_adc_overflow_flag;
}

void app_ecg_adc_clear_overflow(void)
{
  app_ecg_adc_overflow_flag = 0U;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
