/**
  ******************************************************************************
  * @file    app_sd_card.c
  * @brief   SDIO 驱动 — 单次尝试 4 位 @ 400 kHz，永久降级
  *
  * 初始化：
  *   1. 400kHz 1 位 → HAL_SD_Init（卡识别）
  *   2. 400kHz 1 位 → HAL_SD_ConfigWideBusOperation(4 位)（仅一次）
  *   3. 失败 → 保持 1 位；成功后不升速，直接以 400kHz 4 位运行
  *
  * 运行时：
  *   - 4 位下任何读写/等待就绪错误 → 本次上电永久禁用 4 位
  *   - 后续初始化/退避重试仅走 1 位路径，不再尝试 4 位
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

/**
 * @brief  等待 SD 卡返回 TRANSFER（空闲）状态。
 * @param  timeout_ms 最长等待时间，单位毫秒。
 * @return APP_SD_CARD_OK 当卡报告 TRANSFER 状态时返回，
 *         APP_SD_CARD_BUSY 超时返回（超时时卡会被反初始化）。
 * @note   HAL_SD_ReadBlocks / HAL_SD_WriteBlocks 返回 HAL_OK 仅表示
 *         命令已被卡确认。卡可能仍在内部编程中。
 *         此轮询确保下次访问是安全的。
 *         超时后卡会被反初始化，以便上层可以重试。
 */
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
 * 超时策略：超时后立即反初始化卡，防止卡在异常状态时被重复访问，
 * 让上层调用者感知到 APP_SD_CARD_BUSY 并做重试或上报。 */
/**
 * @brief  永久禁用本次上电周期的 4 位 SDIO 总线模式。
 * @note  在处于 4 位模式时，发生任何卡错误（读/写/等待就绪超时）后调用。
 *         后续初始化调用将只尝试 1 位模式，直到下次上电复位。
 */
/* 4 位下任何错误 → 本次上电永久禁用 4 位，后续初始化只走 1 位。 */
static void sdio_disable_4bit_permanently(void)
{
    if (sdio_mode == APP_SD_MODE_4BIT)
    {
        sdio_4bit_permanently_off = true;
        sdio_mode = APP_SD_MODE_4BIT_FAIL;
        sdio_reported_mode = APP_SD_MODE_4BIT_FAIL;
    }
}

/**
 * @brief  等待 SD 卡进入 TRANSFER 状态（阻塞轮询）。
 * @param  timeout_ms 最长等待时间，单位毫秒。
 * @return 卡就绪返回 APP_SD_CARD_OK，超时返回 APP_SD_CARD_BUSY。
 */
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

/**
 * @brief  软件层 SDIO 句柄初始化（不访问硬件）。
 * @note  这不是完整的硬件初始化。它仅将句柄清零并设置 hsd.Instance = SDIO，
 *         以便 SDIO_IRQHandler 和其他 HAL 函数能获取有效的句柄指针。
 *         实际的硬件初始化稍后在 APP_SD_Card_Init() 中完成（懒启动）。
 *         由 APP_SdFile_StartSession() 在挂载 FAT 卷之前调用。
 */
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

/**
 * @brief  初始化 SD 卡硬件（懒启动）。
 * @note  执行两阶段初始化：
 *         1. 通过 HAL_SD_Init 进行 400 kHz / 1 位卡识别。
 *         2. 通过 HAL_SD_ConfigWideBusOperation 单次尝试 4 位模式。
 *         全程时钟保持在 400 kHz；4 位成功后不升速。
 *         如果 4 位已被先前的错误永久禁用，则跳过阶段 2。
 * @return 成功返回 APP_SD_CARD_OK，句柄未设置返回 APP_SD_CARD_NO_INIT，
 *         HAL 错误返回 APP_SD_CARD_INIT_FAILED。
 */
/*
 * SD 卡硬件初始化（懒启动），每次仅调用一次 HAL_SD_ConfigWideBusOperation。
 * 全程 400kHz；4 位失败或已被永久禁用则保持 1 位。 */
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

    /* 第一阶段：400kHz / 1 位卡识别 */
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

    /* 第二阶段：单次尝试 4 位 @ 400kHz。
     * 若此前 4 位下读写已失败则永久跳过。 */
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

/**
 * @brief  反初始化 SD 卡并释放 SDIO 硬件资源。
 * @note  将 card_initialized 标志和块计数重置为零。
 *         调用 HAL_SD_DeInit 关闭 SDIO 接口电源。
 *         即使卡从未初始化也可安全调用。
 */
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

/**
 * @brief  从 SD 卡读取一个或多个 512 字节扇区。
 * @param  buf 目标缓冲区指针（大小 >= count * 512 字节）。
 * @param  sector 起始逻辑块地址（LBA）。
 * @param  count 要读取的连续 512 字节扇区数。
 * @return 成功返回 APP_SD_CARD_OK，失败返回 APP_SD_CARD_ERROR / APP_SD_CARD_BUSY。
 *         错误会触发 4 位永久降级和卡反初始化。
 * @note  这是一个阻塞调用。发生总线错误时，4 位模式在本上电周期内被永久禁用，
 *         且卡会被反初始化。
 */
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
     * 失败时调用反初始化，释放 SDIO 资源和卡状态，
     * 以便上层通过定时重试机制（app_sd_file 60 秒间隔）重新初始化卡。*/
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

/**
 * @brief  向 SD 卡写入一个或多个 512 字节扇区。
 * @param  buf 源数据缓冲区指针（大小 >= count * 512 字节）。
 * @param  sector 起始逻辑块地址（LBA）。
 * @param  count 要写入的连续 512 字节扇区数。
 * @return 成功返回 APP_SD_CARD_OK，失败返回 APP_SD_CARD_ERROR / APP_SD_CARD_BUSY。
 *         错误会触发 4 位永久降级和卡反初始化。
 * @note  这是一个阻塞调用。发生总线错误时，4 位模式在本上电周期内被永久禁用，
 *         且卡会被反初始化。
 */
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

/**
 * @brief  检查 SD 卡是否存在且处于 TRANSFER 状态。
 * @return 如果卡已初始化且可读写返回 true，
 *         如果未初始化或卡忙返回 false。
 */
bool APP_SD_Card_IsPresent(void)
{
    if (!card_initialized)
    {
        return false;
    }
    return (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER);
}

/**
 * @brief  获取 SD 卡的总存储容量。
 * @return 总容量（字节），如果卡未初始化则返回 0。
 */
uint64_t APP_SD_Card_GetCapacity(void)
{
    if (!card_initialized)
    {
        return 0ULL;
    }
    return (uint64_t)card_info.LogBlockNbr * (uint64_t)card_info.LogBlockSize;
}

/**
 * @brief  获取 SD 卡上 512 字节块的数量。
 * @return 以 512 字节为单位的块总数，如果未初始化则返回 0。
 */
uint32_t APP_SD_Card_GetBlockCount(void)
{
    return block_count;
}

/**
 * @brief  获取内部 SDIO HAL 句柄的指针。
 * @return 指向 SDIO 驱动程序使用的 SD_HandleTypeDef 的指针。
 * @note  句柄在调用 APP_SD_Card_InitHardware() 后有效。
 *         由 SDIO_IRQHandler 用于访问 SDIO 外设实例。
 */
SD_HandleTypeDef *APP_SD_Card_GetHandle(void)
{
    return &hsd;
}

/**
 * @brief  获取当前 SDIO 总线宽度模式。
 * @return 0 = 1 位，1 = 4 位，2 = 4 位永久禁用（降级）。
 */
uint8_t APP_SD_Card_GetMode(void)
{
    return sdio_reported_mode;
}

/**
 * @brief  返回最后记录的 SDIO HAL 错误代码。
 * @return 上次失败的 SDIO 操作的 HAL 错误代码。
 *         该值仅在发生错误后有意义。
 */
uint32_t APP_SD_Card_GetLastError(void)
{
    return sdio_last_error;
}
