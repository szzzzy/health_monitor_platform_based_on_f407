/**
  ******************************************************************************
  * @file    sdio.c
  * @brief   SDIO peripheral initialization and GPIO MSP configuration.
  *
  * MX_SDIO_SD_Init() is kept as a CubeMX-compatible safe placeholder that
  * only populates the hsd handle fields.  The real SD-card initialisation is
  * deferred to APP_SD_Card_Init() (lazy-start path), so a missing or bad
  * card does not block the boot sequence (OLED / MAX30102 / RTC / RTOS).
  *
  * HAL_SD_MspInit() configures the SDIO GPIO pins:
  *   PC8-PC11 (DAT0-DAT3) : AF12, PULLUP  — 4-bit data lines, idle-high
  *   PC12     (CLK)        : AF12, NOPULL  — clock, 100R series on PCB
  *   PD2      (CMD)        : AF12, PULLUP  — command/response, idle-high
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
 * @brief  CubeMX-compatible SDIO placeholder — handle fields only, no HW access.
 * @note   The application owns card bring-up through APP_SD_Card_Init()
 *         (lazy-start via APP_DataLog_ServiceBudget → f_mount → disk_initialize).
 *         This function must NOT call HAL_SD_Init() or HAL_SD_ConfigWideBus-
 *         Operation(), so boot can proceed when no SD card is inserted.
 */
void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */
  /*
   * The application owns SD card bring-up through app_sd_card/app_sd_file so
   * boot can continue when no card is inserted. Keep this CubeMX entry as a
   * harmless handle placeholder; do not call HAL_SD_Init() here.
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
  /* HAL_SD_Init() and HAL_SD_ConfigWideBusOperation() are deferred to
   * APP_SD_Card_Init() so a missing/bad card does not block boot.
   * MX_SDIO_SD_Init() is kept as a CubeMX-compatible placeholder that
   * only initializes the handle fields — no hardware access. */
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
 * @brief  SDIO MSP Initialization — GPIO pin configuration.
 * @param  sdHandle SD handle (must have Instance == SDIO).
 * @note   Enables SDIO, GPIOC, and GPIOD clocks.  Configures DAT0-DAT3 and CMD
 *         with internal pull-ups to prevent floating inputs in 4-bit mode;
 *         CLK is left NOPULL (board has a 100R series resistor).
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
    /* D0-D3: data lines with internal pull-up for 4-bit idle state */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CLK: separate NOPULL — pull-up would distort the clock edge */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CMD: internal pull-up for idle-high command line */
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
 * @brief  SDIO MSP De-Initialization — disable clock and de-init GPIO pins.
 * @param  sdHandle SD handle (must have Instance == SDIO).
 * @note   Disables the SDIO peripheral clock and de-initializes all SDIO GPIOs
 *         (PC8-PC12, PD2).
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
