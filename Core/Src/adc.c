/**
  ******************************************************************************
  * @file    adc.c
  * @brief   ADC1 + TIM3 + DMA2 环形缓冲 ECG 采样驱动
  *
  * TIM3 → TRGO 250 Hz → ADC1 单通道转换 → DMA2_Stream0 循环写入
  * 环形缓冲区 app_ecg_adc_buf[256]，由主循环按 10 ms 节拍消费。
  *
  * 覆盖检测：通过 produced/consumed 绝对样本计数识别 DMA 回绕覆盖。
  * 每个消费样本附带 4 ms 间隔的推算时间戳，用于 PTT 时间基准。
  ******************************************************************************
  */

#include "adc.h"
#include "tim.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;

/* 环形缓冲区 + 消费跟踪 */
static uint16_t app_ecg_adc_buf[APP_ECG_ADC_BUF_SIZE];
static volatile uint16_t app_ecg_adc_consume_idx = 0U;
static volatile uint8_t  app_ecg_adc_overflow_flag = 0U;

/*
 * 绝对样本计数，用于可靠检测 DMA circular 覆盖。
 * produced: DMA 已写入的总样本数（由 get_write_index 内推演）。
 * consumed: 软件已消费的总样本数。
 * 覆盖条件：produced - consumed > BUF_SIZE。
 */
static uint32_t ecg_produced = 0U;
static uint32_t ecg_consumed = 0U;
static uint16_t ecg_prev_write_idx = 0U;

/* 更新 produced 计数：根据 write 指针变化推算新样本数 */
static void ecg_track_produced(uint16_t write_idx)
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

/* ADC1 init function — TIM3 TRGO 触发，DMA 循环模式 */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* PC0 — ADC1_IN10 模拟输入 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* DMA2_Stream0, Channel 0 — ADC1 数据搬运，循环模式 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

    /* DMA2_Stream0 中断用于检测传输错误 */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_DMA_DeInit(&hdma_adc1);
  }
}

/* 启动 TIM3 基底 + ADC1 DMA，开始硬件触发连续采样 */
void app_ecg_adc_start(void)
{
  (void)memset(app_ecg_adc_buf, 0, sizeof(app_ecg_adc_buf));
  app_ecg_adc_consume_idx = 0U;
  app_ecg_adc_overflow_flag = 0U;
  ecg_produced = 0U;
  ecg_consumed = 0U;
  ecg_prev_write_idx = 0U;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)app_ecg_adc_buf, APP_ECG_ADC_BUF_SIZE);
  HAL_TIM_Base_Start(&htim3);
}

/* DMA 写入位置 = BUF_SIZE - NDTR。同时更新 produced 计数。 */
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

  /* 推演 DMA 已写入的绝对样本数 */
  ecg_track_produced(write_idx);

  return write_idx;
}

/* 获取当前未消费样本数 */
uint16_t app_ecg_adc_get_available_count(void)
{
  uint32_t avail;

  /* 先更新 produced 计数 */
  (void)app_ecg_adc_get_write_index();

  if (ecg_produced >= ecg_consumed)
  {
    avail = ecg_produced - ecg_consumed;
  }
  else
  {
    avail = 0U; /* 异常情况 */
  }

  if (avail > (uint32_t)APP_ECG_ADC_BUF_SIZE)
  {
    avail = (uint32_t)APP_ECG_ADC_BUF_SIZE;
  }

  return (uint16_t)avail;
}

/*
 * 从环形缓冲消费一个样本，同时回填其推算时间戳。
 *
 * 时间戳逻辑：每个样本间隔 4 ms (250 Hz)。
 * 以当前 HAL_GetTick() 作为批量中最新样本的时刻，
 * 按样本在批量中的位置倒推其时间戳。
 *
 * 调用者应在进入批量消费循环前取一次 now_ms = HAL_GetTick()，
 * 并传入批量剩余样本数 avail_remaining（批量开始前调用
 * app_ecg_adc_get_available_count 获得初值，每消费一个递减）。
 *
 * 返回 1=成功，0=无新数据或发生覆盖丢失。
 */
uint8_t app_ecg_adc_read_sample(uint16_t *raw_value,
                                uint32_t *timestamp_ms,
                                uint32_t now_ms,
                                uint16_t avail_remaining)
{
  uint16_t write_idx;

  if (raw_value == NULL)
  {
    return 0U;
  }

  /* 检测覆盖：DMA 写入超出消费窗口 → 数据已丢失 */
  write_idx = app_ecg_adc_get_write_index();
  if (ecg_produced > (ecg_consumed + (uint32_t)APP_ECG_ADC_BUF_SIZE))
  {
    uint32_t overwritten = ecg_produced - ecg_consumed - (uint32_t)APP_ECG_ADC_BUF_SIZE;
    /* 丢弃已被覆盖的旧样本 */
    ecg_consumed += overwritten;
    app_ecg_adc_consume_idx = (uint16_t)((app_ecg_adc_consume_idx + overwritten) %
                                         APP_ECG_ADC_BUF_SIZE);
    app_ecg_adc_overflow_flag = 1U;
    return 0U; /* 本次不返回数据，调用者应查 overflow flag 后重置并重试 */
  }

  /* 无新数据 */
  if (ecg_consumed >= ecg_produced)
  {
    return 0U;
  }

  /* 推算时间戳：
   * avail_remaining 是调用者传入的批量总样本数（递减前）。
   * avail_remaining=1 → 这是最后一个（最新）样本 → age=0 → ts=now_ms。
   * avail_remaining=N → 这是第 (N-1) 个样本 → age=(N-1)*4 ms。 */
  if (timestamp_ms != NULL)
  {
    uint32_t age_ms = ((uint32_t)avail_remaining - 1U) * 4U;
    *timestamp_ms = (now_ms >= age_ms) ? (now_ms - age_ms) : 0UL;
  }

  *raw_value = app_ecg_adc_buf[app_ecg_adc_consume_idx];
  app_ecg_adc_consume_idx = (uint16_t)((app_ecg_adc_consume_idx + 1U) % APP_ECG_ADC_BUF_SIZE);
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

/* HAL ADC injected conversion callback stub — required by linker */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
}

/* DMA2_Stream0 中断：记录传输错误供主循环观测 */
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
  if (hdma_adc1.ErrorCode != HAL_DMA_ERROR_NONE)
  {
    app_ecg_adc_overflow_flag = 1U;
    hdma_adc1.ErrorCode = HAL_DMA_ERROR_NONE;
  }
}
