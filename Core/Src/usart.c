/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>

/* --------------------------------------------------------------------------
 * USART2 DMA 循环接收缓冲区
 *
 * DMA1_Stream5_Channel4 以 CIRCULAR 模式连续接收，半传输、整圈完成和空闲线
 * 事件只置“有数据待查”标志，实际命令拆帧由 Uitask 完成。wrap_count 与 NDTR
 * 共同组成单调生产计数，可识别协议层落后超过一圈的情况。
 * -------------------------------------------------------------------------- */
static DMA_HandleTypeDef hdma_usart2_rx;
static uint8_t  uart_dma_rx_buf[UART_DMA_RX_BUF_SIZE]
    __attribute__((section(".dma_buffer"), aligned(4), used));
static volatile uint16_t uart_dma_last_pos = 0U;    /* 兼容的环形消费索引 */
static volatile uint8_t  uart_dma_idle_flag = 0U;   /* ISR 置位、协议任务清除的数据到达提示 */
static volatile uint32_t uart_dma_wrap_count = 0U;  /* DMA 完整写满 64 字节的累计圈数 */
static volatile uint32_t uart_error_count = 0U;     /* UART 错误及恢复失败累计次数 */
static volatile uint32_t uart_dma_restart_count = 0U; /* DMA 接收启动/重启累计次数 */

uint8_t *usart_get_dma_rx_buf(void)       { return uart_dma_rx_buf; }
uint16_t usart_get_dma_last_pos(void)      { return uart_dma_last_pos; }
void     usart_set_dma_last_pos(uint16_t p) { uart_dma_last_pos = (uint16_t)(p % UART_DMA_RX_BUF_SIZE); }
uint8_t  usart_get_dma_idle_flag(void)     { return uart_dma_idle_flag; }
void     usart_clear_dma_idle_flag(void)   { uart_dma_idle_flag = 0U; }
void     usart_set_dma_idle_flag(void)     { uart_dma_idle_flag = 1U; }

uint32_t usart_get_uart_error_count(void) { return uart_error_count; }
uint32_t usart_get_dma_restart_count(void) { return uart_dma_restart_count; }

/**
 * @brief  获取 DMA 自最近一次重启以来累计写入的字节数。
 * @return 单调生产计数；DMA 尚未挂接时返回 0。
 * @note   读取 NDTR 前后各取一次 wrap_count，若期间恰好绕圈则重读，避免把
 *         新一圈的 NDTR 与旧一圈计数组合成不一致快照。
 */
uint32_t usart_get_dma_produced_count(void)
{
  uint32_t wraps_before;
  uint32_t wraps_after;
  uint16_t remaining;

  if (huart2.hdmarx == NULL)
  {
    return 0U;
  }

  do
  {
    wraps_before = uart_dma_wrap_count;
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
    wraps_after = uart_dma_wrap_count;
  } while (wraps_before != wraps_after);

  if (remaining == 0U)
  {
    return (wraps_before + 1UL) * UART_DMA_RX_BUF_SIZE;
  }

  return (wraps_before * UART_DMA_RX_BUF_SIZE) +
         (uint32_t)((UART_DMA_RX_BUF_SIZE - remaining) % UART_DMA_RX_BUF_SIZE);
}

/**
 * @brief  中止并重新启动 USART2 环形 DMA 接收，同时复位软件消费位置。
 * @return HAL_UART_Receive_DMA() 的状态。
 * @note   UART 错误回调也会调用本函数；协议层通过 restart_count 识别重启，
 *         丢弃重启前尚未组成完整行的内容。
 */
HAL_StatusTypeDef usart_restart_dma_rx(void)
{
  HAL_StatusTypeDef status;

  (void)HAL_UART_AbortReceive(&huart2);
  uart_dma_last_pos = 0U;
  uart_dma_wrap_count = 0U;
  status = HAL_UART_Receive_DMA(&huart2, uart_dma_rx_buf, UART_DMA_RX_BUF_SIZE);
  uart_dma_restart_count++;
  if (status == HAL_OK)
  {
    uart_dma_idle_flag = 1U;
  }

  return status;
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* 启动 USART2 IDLE 中断 + DMA 循环接收。
   * IDLE 中断在总线空闲（>=1 个字符时间无数据）时触发，
   * 用于检测变长帧结束，免去逐字节轮询。 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  if (usart_restart_dma_rx() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */
  /* USART2_RX DMA: DMA1_Stream5, Channel4, 循环模式 */
  {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart2_rx);

    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart2_rx);

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  }
  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    uart_dma_idle_flag = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    uart_dma_wrap_count++;
    uart_dma_idle_flag = 1U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    uart_error_count++;
    if (usart_restart_dma_rx() != HAL_OK)
    {
      uart_error_count++;
    }
  }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
