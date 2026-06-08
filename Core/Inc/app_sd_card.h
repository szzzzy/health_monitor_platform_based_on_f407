/**
  ******************************************************************************
  * @file    app_sd_card.h
  * @brief   MicroSD card abstraction layer (SDIO 4-bit mode)
  ******************************************************************************
  */

#ifndef __APP_SD_CARD_H__
#define __APP_SD_CARD_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {
    APP_SD_CARD_OK           = 0,
    APP_SD_CARD_NOT_PRESENT  = 1,
    APP_SD_CARD_INIT_FAILED  = 2,
    APP_SD_CARD_ERROR        = 3,
    APP_SD_CARD_BUSY         = 4,
    APP_SD_CARD_NO_INIT      = 5
} AppSdCardStatus_t;

void               APP_SD_Card_InitHardware(void);
AppSdCardStatus_t  APP_SD_Card_Init(void);
void               APP_SD_Card_Deinit(void);
AppSdCardStatus_t  APP_SD_Card_Read(uint8_t *buf, uint32_t sector, uint32_t count);
AppSdCardStatus_t  APP_SD_Card_Write(const uint8_t *buf, uint32_t sector, uint32_t count);
bool               APP_SD_Card_IsPresent(void);
uint64_t           APP_SD_Card_GetCapacity(void);
uint32_t           APP_SD_Card_GetBlockCount(void);
SD_HandleTypeDef  *APP_SD_Card_GetHandle(void);
uint8_t            APP_SD_Card_GetMode(void);       /* 0=1-bit, 1=4-bit, 2=4bit-failed-permanent */
uint32_t           APP_SD_Card_GetLastError(void);  /* 最近一次 hsd.ErrorCode */

#endif
