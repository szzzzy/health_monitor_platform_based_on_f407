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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include <string.h>

/*
 * ECG 采集链路：TIM2 TRGO（250 Hz）→ ADC1_IN5 → DMA2_Stream0 环形缓冲 →
 * MAXtask 批量消费。DMA 只推进硬件写指针；软件以 produced/consumed 单调计数
 * 区分“缓冲恰好为空”和“写指针绕回原位”，并在落后超过一圈时丢弃已覆盖样本。
 */
#define APP_ECG_ADC_SAMPLE_PERIOD_MS 4U  /* 250 Hz 采样周期，单位：ms */

static uint16_t app_ecg_adc_buf[APP_ECG_ADC_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(4), used));
static volatile uint16_t app_ecg_adc_consume_idx = 0U; /* 下一个待读样本的环形索引 */
static volatile uint8_t app_ecg_adc_overflow_flag = 0U; /* 至少发生过一次 DMA 覆盖 */

static uint32_t ecg_produced = 0U;       /* 根据 DMA 写指针累计的生产样本数 */
static uint32_t ecg_consumed = 0U;       /* 已交付给 ECG 算法的样本数 */
static uint16_t ecg_prev_write_idx = 0U; /* 上次观察到的 DMA 写索引 */

/** @brief 根据本次 DMA 写索引与上次索引的环形差值累计生产量。 */
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

/**
 * @brief  检测软件消费者落后是否超过一整圈，并跳过已被 DMA 覆盖的数据。
 * @note   覆盖后保留缓冲中最新的一圈样本，同时锁存溢出标志供 ECG 检测器复位。
 */
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
  sConfig.Channel = ADC_CHANNEL_5;
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

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA5     ------> ADC1_IN5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    PA5     ------> ADC1_IN5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief  清零 ECG DMA 软件状态，启动 ADC1 环形 DMA 与 TIM2 触发源。
 * @note   必须先启动 DMA 再启动定时器，避免首个触发到来时没有接收缓冲。
 */
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

/**
 * @brief  读取 DMA 剩余传输数并换算为当前硬件写索引。
 * @return DMA 下一次写入位置，范围为 [0, APP_ECG_ADC_BUF_SIZE-1]。
 * @note   调用同时更新累计生产量并执行覆盖检测。
 */
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

/** @brief 返回尚未交付给 ECG 算法的样本数，最大不超过一圈缓冲容量。 */
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

/**
 * @brief  按时间顺序取出一个 ECG 原始样本，并重建其采样时间戳。
 * @param  raw_value 输出 12 位 ADC 原始值，不可为 NULL。
 * @param  timestamp_ms 输出估算采样时刻；为 NULL 时不计算。
 * @param  now_ms 本批消费开始时的系统时间，单位：ms。
 * @param  avail_remaining 本样本在当前批次中的剩余数量，用于按 4 ms 间隔回推时间。
 * @return 成功取样返回 1；参数无效或当前无样本返回 0。
 */
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

    *timestamp_ms = now_ms - age_ms;
  }

  *raw_value = app_ecg_adc_buf[app_ecg_adc_consume_idx];
  app_ecg_adc_consume_idx = (uint16_t)((app_ecg_adc_consume_idx + 1U) %
                                       APP_ECG_ADC_BUF_SIZE);
  ecg_consumed++;

  return 1U;
}

/** @brief 查询自上次清除后是否发生过 DMA 覆盖。 */
uint8_t app_ecg_adc_had_overflow(void)
{
  return app_ecg_adc_overflow_flag;
}

/** @brief 清除 DMA 覆盖锁存标志；不会改动生产/消费位置。 */
void app_ecg_adc_clear_overflow(void)
{
  app_ecg_adc_overflow_flag = 0U;
}

/* 以下接口只暴露底层寄存器快照，供调试页面定位 ADC/DMA 停摆或错误标志。 */
uint16_t app_ecg_adc_debug_ndtr(void)
{
  return (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_adc1);
}

uint32_t app_ecg_adc_debug_adc_state(void)
{
  return HAL_ADC_GetState(&hadc1);
}

uint32_t app_ecg_adc_debug_adc_cr2(void)
{
  return (uint32_t)ADC1->CR2;
}

uint32_t app_ecg_adc_debug_dma_cr(void)
{
  return (uint32_t)DMA2_Stream0->CR;
}

uint32_t app_ecg_adc_debug_dma_lisr(void)
{
  return (uint32_t)(DMA2->LISR & 0x3DU); /* Stream0: TCIF0|HTIF0|TEIF0|DMEIF0|FEIF0 */
}

uint32_t app_ecg_adc_debug_dma_errcode(void)
{
  return (uint32_t)hdma_adc1.ErrorCode;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
