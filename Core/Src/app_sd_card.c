/**
  ******************************************************************************
  * @file    app_sd_card.c
  * @brief   MicroSD card driver using SDIO 4-bit mode
  *
  * 初始化流程：
  *   1. 400kHz 低速时钟 → HAL_SD_Init + HAL_SD_InitCard
  *   2. 切换到 4-bit 宽总线
  *   3. 切换到 21MHz 高速时钟
  *
  * 异常策略：
  *   - 卡检测失败 → APP_SD_CARD_NOT_PRESENT
  *   - 超时/CRC 错误 → APP_SD_CARD_ERROR，需重新 Init
  ******************************************************************************
  */

#include "app_sd_card.h"
#include <string.h>

#define APP_SD_INIT_CLK_DIV  208U    /* SDIOCLK / (208+2) ≈ 400kHz */
#define APP_SD_FAST_CLK_DIV    2U    /* SDIOCLK / (2+2)   =  21MHz */
#define APP_SD_TIMEOUT      2000U    /* 2 秒 */

static SD_HandleTypeDef        hsd;
static HAL_SD_CardInfoTypeDef  card_info;
static bool                    card_initialized = false;
static uint32_t                block_count = 0;

/* -------------------------------------------------------------------------- */
/*  公共 API                                                                  */
/* -------------------------------------------------------------------------- */

void APP_SD_Card_InitHardware(void)
{
    (void)memset(&hsd, 0, sizeof(hsd));
    hsd.Instance = SDIO;
    card_initialized = false;
    block_count = 0;
}

AppSdCardStatus_t APP_SD_Card_Init(void)
{
    HAL_StatusTypeDef hal_ret;

    if (hsd.Instance != SDIO)
    {
        return APP_SD_CARD_NO_INIT;
    }

    if (card_initialized)
    {
        APP_SD_Card_Deinit();
    }

    /* 第一阶段：低速模式初始化 */
    hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide             = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv            = APP_SD_INIT_CLK_DIV;

    hal_ret = HAL_SD_Init(&hsd);
    if (hal_ret != HAL_OK)
    {
        APP_SD_Card_Deinit();
        return APP_SD_CARD_INIT_FAILED;
    }

    /* 第二阶段：识别卡并获取信息 */
    hal_ret = HAL_SD_InitCard(&hsd);
    if (hal_ret != HAL_OK)
    {
        APP_SD_Card_Deinit();
        return APP_SD_CARD_NOT_PRESENT;
    }

    HAL_SD_GetCardInfo(&hsd, &card_info);

    /* 第三阶段：切换到 4-bit 模式和高速时钟 */
    hal_ret = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
    if (hal_ret != HAL_OK)
    {
        HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_1B);
    }

    hsd.Init.ClockDiv = APP_SD_FAST_CLK_DIV;
    (void)HAL_SD_Init(&hsd);

    card_initialized = true;
    block_count      = card_info.LogBlockNbr *
                       (card_info.LogBlockSize / 512U);

    return APP_SD_CARD_OK;
}

void APP_SD_Card_Deinit(void)
{
    if (card_initialized)
    {
        HAL_SD_DeInit(&hsd);
    }
    card_initialized = false;
    block_count = 0;
}

AppSdCardStatus_t APP_SD_Card_Read(uint8_t *buf, uint32_t sector, uint32_t count)
{
    HAL_StatusTypeDef hal_ret;

    if (!card_initialized || (buf == NULL) || (count == 0U))
    {
        return APP_SD_CARD_NO_INIT;
    }

    if ((sector + count) > block_count)
    {
        return APP_SD_CARD_ERROR;
    }

    /* HAL_SD_ReadBlocks 是阻塞调用，含超时参数 */
    hal_ret = HAL_SD_ReadBlocks(&hsd, buf, sector * 512U, count, APP_SD_TIMEOUT);
    if (hal_ret != HAL_OK)
    {
        return APP_SD_CARD_ERROR;
    }

    return APP_SD_CARD_OK;
}

AppSdCardStatus_t APP_SD_Card_Write(const uint8_t *buf, uint32_t sector,
                                     uint32_t count)
{
    HAL_StatusTypeDef hal_ret;

    if (!card_initialized || (buf == NULL) || (count == 0U))
    {
        return APP_SD_CARD_NO_INIT;
    }

    if ((sector + count) > block_count)
    {
        return APP_SD_CARD_ERROR;
    }

    /* HAL_SD_WriteBlocks 是阻塞调用，含超时参数 */
    hal_ret = HAL_SD_WriteBlocks(&hsd, (uint8_t *)buf, sector * 512U, count, APP_SD_TIMEOUT);
    if (hal_ret != HAL_OK)
    {
        return APP_SD_CARD_ERROR;
    }

    return APP_SD_CARD_OK;
}

bool APP_SD_Card_IsPresent(void)
{
    if (!card_initialized)
    {
        return false;
    }
    return (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER);
}

uint64_t APP_SD_Card_GetCapacity(void)
{
    if (!card_initialized)
    {
        return 0ULL;
    }
    return (uint64_t)card_info.LogBlockNbr * (uint64_t)card_info.LogBlockSize;
}

uint32_t APP_SD_Card_GetBlockCount(void)
{
    return block_count;
}

SD_HandleTypeDef *APP_SD_Card_GetHandle(void)
{
    return &hsd;
}
