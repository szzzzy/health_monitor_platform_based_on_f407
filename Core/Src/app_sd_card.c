/**
  ******************************************************************************
  * @file    app_sd_card.c
  * @brief   SDIO driver — single-attempt 4-bit @ 400 kHz, permanent fallback
  *
  * 初始化：
  *   1. 400kHz 1-bit → HAL_SD_Init（卡识别）
  *   2. 400kHz 1-bit → HAL_SD_ConfigWideBusOperation(4-bit)（仅一次）
  *   3. 失败 → 保持 1-bit；成功后不升速，直接以 400kHz 4-bit 运行
  *
  * 运行时：
  *   - 4-bit 下任何读写/WaitReady 错误 → 本次上电永久禁用 4-bit
  *   - 后续 Init/backoff 重试仅走 1-bit 路径，不再尝试 4-bit
  ******************************************************************************
  */

#include "app_sd_card.h"
#include <string.h>

#define APP_SD_CLK_DIV       118U    /* 48MHz / (118+2) ≈ 400kHz — 全程不升速 */
#define APP_SD_WIDE_DTIMER    0x00989680UL  /* ≈ 200ms @ 48MHz, 防止 SD_FindSCR 无限等 */
#define APP_SD_TIMEOUT        200U

#define APP_SD_MODE_1BIT       0U
#define APP_SD_MODE_4BIT       1U
#define APP_SD_MODE_4BIT_FAIL  2U

static SD_HandleTypeDef        hsd;
static HAL_SD_CardInfoTypeDef  card_info;
static bool                    card_initialized = false;
static uint32_t                block_count = 0;
static uint8_t                 sdio_mode = APP_SD_MODE_1BIT;
static uint8_t                 sdio_reported_mode = APP_SD_MODE_1BIT;
static uint32_t                sdio_last_error = 0U;
static bool                    sdio_4bit_permanently_off = false;

/*
 * 等待 SD 卡从编程/接收状态回到传输空闲状态。
 *
 * 为什么需要：
 *   HAL_SD_ReadBlocks / HAL_SD_WriteBlocks 返回 HAL_OK 只表示
 *   "命令已发送并被卡确认"，不代表数据实际写入完毕。
 *   卡在内部数据阶段可能仍处于编程状态，此时发起下一次读写
 *   会收到忙响应或被 CRC 错误打断。
 *   必须等卡回到 TRANSFER 状态后才能安全进行下一次访问。
 *
 * 超时策略：超时后立即 Deinit 卡，防止卡在异常状态时被重复访问，
 * 让上层调用者感知到 APP_SD_CARD_BUSY 并做重试或上报。 */
/* 4-bit 下任何错误 → 本次上电永久禁用 4-bit，后续 Init 只走 1-bit。 */
static void sdio_disable_4bit_permanently(void)
{
    if (sdio_mode == APP_SD_MODE_4BIT)
    {
        sdio_4bit_permanently_off = true;
        sdio_mode = APP_SD_MODE_4BIT_FAIL;
        sdio_reported_mode = APP_SD_MODE_4BIT_FAIL;
    }
}

static AppSdCardStatus_t APP_SD_Card_WaitReady(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            sdio_disable_4bit_permanently();
            APP_SD_Card_Deinit();
            return APP_SD_CARD_BUSY;
        }
    }

    return APP_SD_CARD_OK;
}

/* -------------------------------------------------------------------------- */
/*  公共 API                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * 软件层初始化：仅设置句柄字段，不访问 SDIO 外设。
 *
 * 由 APP_SdFile_StartSession() 在挂载前调用，确保 hsd.Instance 有效，
 * 以便 SDIO_IRQHandler 通过 APP_SD_Card_GetHandle() 获取合法句柄。
 * 真正的硬件初始化在 APP_SD_Card_Init() 中完成（懒启动）。 */
void APP_SD_Card_InitHardware(void)
{
    (void)memset(&hsd, 0, sizeof(hsd));
    hsd.Instance = SDIO;
    card_initialized = false;
    block_count = 0;
    sdio_mode = APP_SD_MODE_1BIT;
}

/*
 * SD 卡硬件初始化（懒启动），每次仅调用一次 HAL_SD_ConfigWideBusOperation。
 * 全程 400kHz；4-bit 失败或已被永久禁用则保持 1-bit。 */
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

    /* 第一阶段：400kHz / 1-bit 卡识别 */
    hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide             = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv            = APP_SD_CLK_DIV;

    hal_ret = HAL_SD_Init(&hsd);
    if (hal_ret != HAL_OK)
    {
        sdio_last_error = hsd.ErrorCode;
        APP_SD_Card_Deinit();
        return APP_SD_CARD_INIT_FAILED;
    }

    HAL_SD_GetCardInfo(&hsd, &card_info);

    /* 第二阶段：单次尝试 4-bit @ 400kHz。
     * 若此前 4-bit 下读写已失败则永久跳过。 */
    if (!sdio_4bit_permanently_off)
    {
        SDIO->DTIMER = APP_SD_WIDE_DTIMER;

        __HAL_SD_CLEAR_FLAG(&hsd, SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT
                                  | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR
                                  | SDIO_STA_STBITERR);

        hal_ret = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
        if (hal_ret == HAL_OK)
        {
            hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
            sdio_mode = APP_SD_MODE_4BIT;
            sdio_reported_mode = APP_SD_MODE_4BIT;
        }
        else
        {
            sdio_last_error = hsd.ErrorCode;
            hsd.ErrorCode   = HAL_SD_ERROR_NONE;
            hsd.Init.BusWide  = SDIO_BUS_WIDE_1B;
            hsd.Init.ClockDiv = APP_SD_CLK_DIV;
            sdio_mode = APP_SD_MODE_4BIT_FAIL;
            sdio_reported_mode = APP_SD_MODE_4BIT_FAIL;
        }

        SDIO->DTIMER = 0xFFFFFFFFUL;
    }
    else
    {
        sdio_reported_mode = APP_SD_MODE_4BIT_FAIL;
    }

    card_initialized = true;
    block_count      = card_info.LogBlockNbr *
                       (card_info.LogBlockSize / 512U);

    return APP_SD_CARD_OK;
}

void APP_SD_Card_Deinit(void)
{
    if (hsd.Instance == SDIO)
    {
        (void)HAL_SD_DeInit(&hsd);
    }
    card_initialized = false;
    block_count = 0;
    sdio_mode = APP_SD_MODE_1BIT;
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

    /*
     * HAL_SD_ReadBlocks 是阻塞调用，含超时参数。
     * 失败时调用 Deinit，释放 SDIO 资源和卡状态，
     * 以便上层通过定时重试机制（app_sd_file 60 秒间隔）重新 Init 卡。*/
    hal_ret = HAL_SD_ReadBlocks(&hsd, buf, sector, count, APP_SD_TIMEOUT);
    if (hal_ret != HAL_OK)
    {
        sdio_last_error = hsd.ErrorCode;
        sdio_disable_4bit_permanently();
        APP_SD_Card_Deinit();
        return APP_SD_CARD_ERROR;
    }

    return APP_SD_Card_WaitReady(APP_SD_TIMEOUT);
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

    hal_ret = HAL_SD_WriteBlocks(&hsd, (uint8_t *)buf, sector, count, APP_SD_TIMEOUT);
    if (hal_ret != HAL_OK)
    {
        sdio_last_error = hsd.ErrorCode;
        sdio_disable_4bit_permanently();
        APP_SD_Card_Deinit();
        return APP_SD_CARD_ERROR;
    }

    return APP_SD_Card_WaitReady(APP_SD_TIMEOUT);
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

uint8_t APP_SD_Card_GetMode(void)
{
    return sdio_reported_mode;
}

uint32_t APP_SD_Card_GetLastError(void)
{
    return sdio_last_error;
}
