/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define UART_DMA_RX_BUF_SIZE  64U  /* USART2 环形 DMA 接收缓冲容量，单位：字节 */
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/** @brief 获取 DMA 接收缓冲首地址；调用方只读其中已生产的区间。 */
uint8_t *usart_get_dma_rx_buf(void);
/** @brief 读取/更新兼容的环形消费索引，设置值会按缓冲容量取模。 */
uint16_t usart_get_dma_last_pos(void);
void     usart_set_dma_last_pos(uint16_t p);
/** @brief 查询、清除或置位“接收数据待处理”提示标志。 */
uint8_t  usart_get_dma_idle_flag(void);
void     usart_clear_dma_idle_flag(void);
void     usart_set_dma_idle_flag(void);
/** @brief 返回自最近一次 DMA 重启以来累计生产的字节数。 */
uint32_t usart_get_dma_produced_count(void);
/** @brief 返回 UART 错误计数与 DMA 启动/重启计数。 */
uint32_t usart_get_uart_error_count(void);
uint32_t usart_get_dma_restart_count(void);
/** @brief 中止并重新启动 USART2 环形 DMA 接收。 */
HAL_StatusTypeDef usart_restart_dma_rx(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
