/**
  ******************************************************************************
  * @file    app_sd_card.h
  * @brief   MicroSD 块设备抽象层（SDIO 1 位启动、4 位一次尝试与故障降级）
  ******************************************************************************
  */

#ifndef __APP_SD_CARD_H__
#define __APP_SD_CARD_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {
    APP_SD_CARD_OK           = 0, /* 操作成功 */
    APP_SD_CARD_NOT_PRESENT  = 1, /* 未检测到可用卡 */
    APP_SD_CARD_INIT_FAILED  = 2, /* HAL 初始化或总线宽度配置失败 */
    APP_SD_CARD_ERROR        = 3, /* 块读写失败 */
    APP_SD_CARD_BUSY         = 4, /* 等待 TRANSFER 状态超时 */
    APP_SD_CARD_NO_INIT      = 5  /* 尚未初始化即请求读写 */
} AppSdCardStatus_t;

/** @brief 只初始化 SDIO 句柄的软件字段，不执行卡识别。 */
void               APP_SD_Card_InitHardware(void);
/** @brief 懒初始化卡；先以 1 位识别，再按本次上电降级策略尝试 4 位。 */
AppSdCardStatus_t  APP_SD_Card_Init(void);
/** @brief 反初始化 SDIO，并清除卡容量与就绪状态。 */
void               APP_SD_Card_Deinit(void);
/** @brief 读取连续 512 字节扇区；完成后等待卡回到 TRANSFER 状态。 */
AppSdCardStatus_t  APP_SD_Card_Read(uint8_t *buf, uint32_t sector, uint32_t count);
/** @brief 写入连续 512 字节扇区；完成后等待卡内部编程结束。 */
AppSdCardStatus_t  APP_SD_Card_Write(const uint8_t *buf, uint32_t sector, uint32_t count);
/** @brief 查询卡已初始化且当前报告 TRANSFER 状态。 */
bool               APP_SD_Card_IsPresent(void);
/** @brief 返回卡容量（字节）和 512 字节块数；未初始化时返回 0。 */
uint64_t           APP_SD_Card_GetCapacity(void);
uint32_t           APP_SD_Card_GetBlockCount(void);
/** @brief 返回内部 HAL SD 句柄，供 SDIO ISR 分发使用。 */
SD_HandleTypeDef  *APP_SD_Card_GetHandle(void);
uint8_t            APP_SD_Card_GetMode(void);       /* 0=1 位，1=4 位，2=4 位永久失败 */
uint32_t           APP_SD_Card_GetLastError(void);  /* 最近一次 hsd.ErrorCode */

#endif
