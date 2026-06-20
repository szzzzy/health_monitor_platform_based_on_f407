/**
  ******************************************************************************
  * @file    sdio.c
  * @brief   SDIO 外设初始化与 GPIO MSP 配置。
  *
  * MX_SDIO_SD_Init() 保留为 CubeMX 兼容的安全占位函数，
  * 只填充 hsd 句柄字段。真正的 SD 卡初始化延后到
  * APP_SD_Card_Init()（懒启动路径）中执行，因此无卡或坏卡不会阻塞
  * 启动流程（OLED / MAX30102 / RTC / RTOS）。
  *
  * HAL_SD_MspInit() 配置 SDIO GPIO 引脚：
  *   PC8-PC11 (DAT0-DAT3) : AF12, PULLUP  — 4 位数据线，空闲高电平
  *   PC12     (CLK)        : AF12, NOPULL  — 时钟线，PCB 串联 100R
  *   PD2      (CMD)        : AF12, PULLUP  — 命令/响应线，空闲高电平
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
#include "sdio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SD_HandleTypeDef hsd;

/* SDIO init function */

/**
 * @brief  CubeMX 兼容的 SDIO 占位函数：只填充句柄字段，不访问硬件。
 * @note   应用通过 APP_SD_Card_Init() 接管 SD 卡拉起
 *         （APP_DataLog_ServiceBudget → f_mount → disk_initialize 懒启动）。
 *         此函数不得调用 HAL_SD_Init() 或 HAL_SD_ConfigWideBusOperation()，
 *         以保证未插入 SD 卡时系统仍可继续启动。
 */
void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */
  /*
   * 应用通过 app_sd_card/app_sd_file 接管 SD 卡拉起，
   * 未插卡时启动流程仍可继续。此 CubeMX 入口仅保留为无副作用的句柄占位，
   * 不要在这里调用 HAL_SD_Init()。
   */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* HAL_SD_Init() 和 HAL_SD_ConfigWideBusOperation() 延后到
   * APP_SD_Card_Init() 中执行，避免无卡/坏卡阻塞启动。
   * MX_SDIO_SD_Init() 仅作为 CubeMX 兼容占位，初始化句柄字段而不访问硬件。 */
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
 * @brief  SDIO MSP 初始化：GPIO 引脚配置。
 * @param  sdHandle SD 句柄（Instance 必须为 SDIO）。
 * @note   使能 SDIO、GPIOC 和 GPIOD 时钟。DAT0-DAT3 与 CMD 配置内部上拉，
 *         避免 4 位模式下输入悬空；CLK 保持 NOPULL（板上已串联 100R）。
 */
void HAL_SD_MspInit(SD_HandleTypeDef* sdHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(sdHandle->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspInit 0 */

  /* USER CODE END SDIO_MspInit 0 */
    /* SDIO clock enable */
    __HAL_RCC_SDIO_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0   (PULLUP)
    PC9     ------> SDIO_D1   (PULLUP)
    PC10    ------> SDIO_D2   (PULLUP)
    PC11    ------> SDIO_D3   (PULLUP)
    PC12    ------> SDIO_CK   (NOPULL — clock, 100R series on PCB)
    PD2     ------> SDIO_CMD  (PULLUP)
    */
    /* D0-D3：数据线，内部上拉以保持 4 位模式空闲状态 */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CLK：单独使用 NOPULL，上拉会影响时钟边沿 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CMD：命令线内部上拉，保持空闲高电平 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN SDIO_MspInit 1 */

  /* USER CODE END SDIO_MspInit 1 */
  }
}

/**
 * @brief  SDIO MSP 反初始化：关闭时钟并反初始化 GPIO 引脚。
 * @param  sdHandle SD 句柄（Instance 必须为 SDIO）。
 * @note   关闭 SDIO 外设时钟，并反初始化全部 SDIO GPIO
 *         （PC8-PC12、PD2）。
 */
void HAL_SD_MspDeInit(SD_HandleTypeDef* sdHandle)
{

  if(sdHandle->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspDeInit 0 */

  /* USER CODE END SDIO_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDIO_CLK_DISABLE();

    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* USER CODE BEGIN SDIO_MspDeInit 1 */

  /* USER CODE END SDIO_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
