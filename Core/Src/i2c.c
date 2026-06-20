/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
/* I2C1 DMA 句柄（文件级静态，供 HAL 回调引用）。
 * TX: DMA1_Stream6_Channel1 — 发送设备地址+寄存器地址
 * RX: DMA1_Stream0_Channel1 — 接收 FIFO 数据 */
static DMA_HandleTypeDef hdma_i2c1_tx;
static DMA_HandleTypeDef hdma_i2c1_rx;

/*
 * I2C1_InitHandle — 独立出来的 I2C 初始化，供正常启动和总线恢复复用
 * I2C1_RecoveryDelay — 总线恢复时的位脉冲间隔延时
 */
static HAL_StatusTypeDef I2C1_InitHandle(void);
static void I2C1_RecoveryDelay(void);
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */
  /* 配置 I2C1 TX/RX DMA 通道以及 I2C 事件/错误中断。
   * HAL_I2C_Mem_Read_DMA 使用 TX DMA 发送寄存器地址阶段，
   * 使用 RX DMA 接收数据阶段；TX/RX DMA 都必须启用。 */
  {
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* I2C1_TX：DMA1 Stream6，Channel1 */
    hdma_i2c1_tx.Instance = DMA1_Stream6;
    hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_i2c1_tx);
    __HAL_LINKDMA(i2cHandle, hdmatx, hdma_i2c1_tx);

    /* I2C1_RX：DMA1 Stream0，Channel1 */
    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_i2c1_rx);
    __HAL_LINKDMA(i2cHandle, hdmarx, hdma_i2c1_rx);

    /* DMA 中断 */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* I2C 事件/错误中断（HAL DMA 传输需要） */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */
    /*
     * 总线上残留事务可能导致 DMA 流和中断处于挂起状态。
     * 反初始化时必须：
     * 1. 停止并释放 DMA 流，防止下一次 Init 时 HAL 发现流已被占用
     * 2. 关闭 I2C 的 DMA 和事件/错误中断，防止后续 GPIO 操作中误触发
     */
    if (i2cHandle->hdmatx != NULL)
    {
      (void)HAL_DMA_DeInit(i2cHandle->hdmatx);
    }

    if (i2cHandle->hdmarx != NULL)
    {
      (void)HAL_DMA_DeInit(i2cHandle->hdmarx);
    }

    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
 * 独立出来的 I2C1 初始化函数，设置 400 kHz 快速模式。
 * 既供 MX_I2C1_Init 调用，也供 MX_I2C1_RecoverBus 恢复后重新初始化。
 */
static HAL_StatusTypeDef I2C1_InitHandle(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  return HAL_I2C_Init(&hi2c1);
}

/*
 * I2C 恢复期间的 GPIO 位脉冲延时：
 * 128 次 NOP 循环，在 168 MHz 的 CM4 上约 3 us。
 * 满足 I2C 标准模式 4.7 us SCL 高低电平的要求。
 */
static void I2C1_RecoveryDelay(void)
{
  volatile uint32_t wait = 128U;

  while (wait > 0U)
  {
    wait--;
    __NOP();
  }
}

/*
 * I2C1 总线恢复函数 —— 适用于 MAX30102 从机意外拉死 SDA 的场景。
 *
 * 故障现象：
 *   当主机正在读取从机数据时，传感器突然掉电或复位，
 *   从机可能在 SDA 拉低后停止响应。
 *   此时 I2C 外设检测到总线忙，HAL_I2C_Init 会直接返回 HAL_BUSY。
 *
 * 恢复协议（基于 I2C 规范的软件线清除流程）：
 *   1. 停止 DMA、反初始化 I2C 外设，强制复位 I2C1 寄存器
 *   2. 将 PB8 (SCL) 和 PB9 (SDA) 切为通用 GPIO 输出模式
 *   3. 在 SCL 上发送最多 9 个时钟脉冲，每个脉冲期间检测 SDA：
 *      - 如果从机释放 SDA（回到高电平），说明从机已响应，提前跳出
 *      - 如果 9 个脉冲后 SDA 仍为低，说明从机彻底挂死
 *   4. 发送 I2C STOP 条件（SDA 从低变高，且 SCL 为高）
 *   5. 重新初始化 I2C 外设，恢复 400kHz 快速模式
 *
 * 返回值：
 *   HAL_OK — SDA 回到高电平且 I2C 重新初始化成功，总线可用
 *   HAL_ERROR — SDA 仍为低（从机未释放）或 I2C 初始化失败
 *
 * 注意：此函数会临时将 GPIO 从 AF_OD 重新配置为 OUTPUT_OD。
 *        重新初始化 I2C 时 HAL_I2C_MspInit 会将 GPIO 恢复为 AF_OD。
 */
HAL_StatusTypeDef MX_I2C1_RecoverBus(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t pulse_count;
  GPIO_PinState sda_state;

  /* 第一步：彻底关闭 I2C 外设、DMA 和寄存器状态。 */
  if (hi2c1.hdmatx != NULL)
  {
    (void)HAL_DMA_Abort(hi2c1.hdmatx);
  }

  if (hi2c1.hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(hi2c1.hdmarx);
  }

  (void)HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 第二步：将 SCL/SDA 切为 GPIO 开漏输出，准备手动打钟 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 初始状态：两条线都拉高 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  I2C1_RecoveryDelay();

  /* 第三步：在 SCL 上打出最多 9 个时钟脉冲。
   * 每次 SCL 上升沿后读取一次 SDA：如果 SDA 已变高，说明从机已释放总线，
   * 不需要继续打钟，这既遵循 I2C 规范，又避免不必要的总线翻转。 */
  for (pulse_count = 0U; pulse_count < 9U; pulse_count++)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
    {
      break;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    I2C1_RecoveryDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    I2C1_RecoveryDelay();
  }

  /* 第四步：生成 I2C STOP 条件 —— 在 SCL=H 时，SDA: L → H
   * SDA=H ← SDA=L + 延时 → SCL=H + 延时 → SDA=H + 延时 = STOP */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  I2C1_RecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  I2C1_RecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  I2C1_RecoveryDelay();

  /* 第五步：验证 SDA 已恢复高电平，并重新初始化 I2C 外设 */
  sda_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

  if (I2C1_InitHandle() != HAL_OK)
  {
    return HAL_ERROR;
  }

  return (sda_state == GPIO_PIN_SET) ? HAL_OK : HAL_ERROR;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
