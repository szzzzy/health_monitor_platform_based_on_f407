/**
  ******************************************************************************
  * @file    app_sd_card.c
  * @brief   MicroSD card driver using SDIO 4-bit mode
  *
  * 初始化流程：
  *   1. 400kHz 低速时钟 → HAL_SD_Init（HAL 内部完成卡识别）
  *   2. 设置 12MHz 时钟分频并切换到 4-bit 宽总线
  *
  * 异常策略：
  *   - 低速初始化/卡识别失败 → APP_SD_CARD_INIT_FAILED
  *   - 传输超时/CRC 错误 → APP_SD_CARD_ERROR 或 APP_SD_CARD_BUSY，需重新 Init
  ******************************************************************************
  */

#include "app_sd_card.h"
#include <string.h>

#define APP_SD_INIT_CLK_DIV  118U    /* 48MHz SDIOCLK / (118+2) ~= 400kHz */
#define APP_SD_FAST_CLK_DIV    2U    /* 48MHz SDIOCLK / (2+2)   =  12MHz */
#define APP_SD_TIMEOUT       200U    /* 200 ms — 典型 4 扇区写 <10 ms，20x 余量；超时即 Deinit 避免阻塞主循环 */

static SD_HandleTypeDef        hsd;
static HAL_SD_CardInfoTypeDef  card_info;
static bool                    card_initialized = false;
static uint32_t                block_count = 0;

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
static AppSdCardStatus_t APP_SD_Card_WaitReady(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
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
}

/*
 * 真正的 SD 卡硬件初始化（懒启动路径）。
 *
 * 此函数由 disk_initialize() → f_mount() 链路调用，仅在首次写日志时触发。
 * 不在 MX_SDIO_SD_Init() 中调用，确保无卡/坏卡不阻塞启动流程。
 *
 * 流程：400kHz 低速识别 → 读取卡信息 → 切换到 12MHz 4-bit 宽总线。
 * 4-bit 切换失败时降级为 1-bit，两者都失败则 Deinit 并返回错误。 */
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

    /* 第一阶段：低速模式初始化并识别卡。HAL_SD_Init 内部会调用 HAL_SD_InitCard。 */
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

    HAL_SD_GetCardInfo(&hsd, &card_info);

    /* ---- TEMPORARY TEST: skip 4-bit wide-bus switch ----
     * Hypothesis: HAL_SD_ConfigWideBusOperation(SDIO_BUS_WIDE_4B) or one of
     * the D1-D3 lines (pull-up, signal integrity, card compatibility) causes
     * the system to hang during the second PLACE-FINGER → measurement cycle.
     *
     * If the hang disappears with 1-bit-only operation, the root cause is in
     * the 4-bit SDIO path (hardware or HAL). Revert this block after test.
     *
     * Normal code (commented out for test):
     *   hsd.Init.ClockDiv = APP_SD_FAST_CLK_DIV;
     *   hal_ret = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
     *   if (hal_ret != HAL_OK) { ... fallback to 1-bit ... }
     *   else { hsd.Init.BusWide = SDIO_BUS_WIDE_4B; }
     */
    (void)hal_ret;
    /* Keep 400 kHz 1-bit — safe and sufficient for logging at 100 Hz. */

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
        APP_SD_Card_Deinit();
        return APP_SD_CARD_ERROR;
    }

    /* 读到数据不代表卡已空闲：等待卡回到 TRANSFER 状态再进行后续操作 */
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

    /*
     * HAL_SD_WriteBlocks 是阻塞调用，含超时参数。
     * 写失败时立即 Deinit，确保卡状态一致。*/
    hal_ret = HAL_SD_WriteBlocks(&hsd, (uint8_t *)buf, sector, count, APP_SD_TIMEOUT);
    if (hal_ret != HAL_OK)
    {
        APP_SD_Card_Deinit();
        return APP_SD_CARD_ERROR;
    }

    /* 写命令被接受，但卡可能在内部编程阶段：等卡重新闲下来 */
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
