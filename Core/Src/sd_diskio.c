/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   FatFs -> app_sd_card 磁盘 I/O 桥接层
  *
  * 实现 FatFs 要求的 5 个 disk_* 函数 + get_fattime()。
  * 所有 SD 卡访问通过 app_sd_card 抽象层，不直接操作 HAL。
  *
 * 调用链：main.c → APP_DataLog_ServiceBudget() → APP_SdFile_WriteBytes()
  *         → f_write() → disk_write() → APP_SD_Card_Write() → HAL_SD_WriteBlocks()
  *
  * 错误隔离：disk_read/disk_write 失败时自动设置 STA_NOINIT，
  * FatFs 后续 I/O 快速返回 RES_NOTRDY，不再尝试访问已断开的卡。
  ******************************************************************************
  */

#include "diskio.h"
#include "sd_diskio.h"
#include "app_sd_card.h"

/* 磁盘状态跟踪 */
static volatile DSTATUS disk_status_flags = STA_NOINIT;

/* -------------------------------------------------------------------------- */
/*  FatFs 磁盘 I/O 接口                                                       */
/* -------------------------------------------------------------------------- */

/**
 ******************************************************************************
 * @brief  FatFs disk_initialize：通过 app_sd_card 初始化 SD 卡。
 * @param  pdrv 物理驱动器号（必须为 0）。
 * @return 成功返回 0，失败或无效驱动器返回 STA_NOINIT。
 ******************************************************************************
 */
DSTATUS disk_initialize(BYTE pdrv)
{
    AppSdCardStatus_t ret;

    if (pdrv != 0)
    {
        return STA_NOINIT;
    }

    ret = APP_SD_Card_Init();

    if (ret == APP_SD_CARD_OK)
    {
        disk_status_flags = 0U;
        return 0U;
    }

    disk_status_flags = STA_NOINIT;
    return STA_NOINIT;
}

/**
 ******************************************************************************
 * @brief  FatFs disk_status：返回当前磁盘状态标志。
 * @param  pdrv 物理驱动器号（必须为 0）。
 * @return 当前 DSTATUS 标志（0 = 正常，STA_NOINIT = 未就绪）。
 ******************************************************************************
 */
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }
    return disk_status_flags;
}

/**
 ******************************************************************************
 * @brief  FatFs disk_read：从 SD 卡读取一个或多个扇区。
 * @param  pdrv   物理驱动器号（必须为 0）。
 * @param  buff   目标缓冲（BYTE *）。
 * @param  sector 起始扇区 LBA。
 * @param  count  要读取的扇区数。
 * @return 成功返回 RES_OK，失败返回 RES_ERROR 或 RES_NOTRDY / RES_PARERR。
 * @note   失败时设置 STA_NOINIT，导致后续 FatFs 调用立即返回 RES_NOTRDY。
 ******************************************************************************
 */
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    AppSdCardStatus_t ret;

    if (pdrv != 0U)
    {
        return RES_PARERR;
    }

    if (disk_status_flags & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (count == 0U)
    {
        return RES_PARERR;
    }

    ret = APP_SD_Card_Read(buff, (uint32_t)sector, (uint32_t)count);

    if (ret == APP_SD_CARD_OK)
    {
        return RES_OK;
    }

    disk_status_flags |= STA_NOINIT;
    return RES_ERROR;
}

/**
 ******************************************************************************
 * @brief  FatFs disk_write：向 SD 卡写入一个或多个扇区。
 * @param  pdrv   物理驱动器号（必须为 0）。
 * @param  buff   源数据缓冲。
 * @param  sector 起始扇区 LBA。
 * @param  count  要写入的扇区数。
 * @return 成功返回 RES_OK，失败返回 RES_ERROR 或 RES_NOTRDY / RES_PARERR。
 * @note   阻塞调用——返回时数据已在卡上。失败时设置 STA_NOINIT。
 ******************************************************************************
 */
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    AppSdCardStatus_t ret;

    if (pdrv != 0U)
    {
        return RES_PARERR;
    }

    if (disk_status_flags & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (count == 0U)
    {
        return RES_PARERR;
    }

    ret = APP_SD_Card_Write(buff, (uint32_t)sector, (uint32_t)count);

    if (ret == APP_SD_CARD_OK)
    {
        return RES_OK;
    }

    disk_status_flags |= STA_NOINIT;
    return RES_ERROR;
}

/**
 ******************************************************************************
 * @brief  将磁盘标记为不可用，使 FatFs 返回 RES_NOTRDY。
 * @note   APP_SD_Card_Deinit() 后，app_sd_card 层重置其自身的初始化标志，
 *         但这不会传播到 FatFs 的 disk_status_flags。
 *         sd_diskio_invalidate() 填补这个空隙，使后续 disk_read / disk_write
 *         跳过已反初始化卡的 HAL_SD I/O，避免二次阻塞调用。
 ******************************************************************************
 */
void sd_diskio_invalidate(void)
{
    disk_status_flags |= STA_NOINIT;
}

/**
 ******************************************************************************
 * @brief  FatFs disk_ioctl：设备控制命令（CTRL_SYNC、扇区数、
 *         扇区大小、块大小）。
 * @param  pdrv 物理驱动器号（必须为 0）。
 * @param  cmd  命令标识符。
 * @param  buff 命令特定数据缓冲。
 * @return RES_OK、RES_ERROR 或 RES_PARERR。
 ******************************************************************************
 */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    uint32_t blk_cnt;

    if (pdrv != 0U)
    {
        return RES_PARERR;
    }

    if (disk_status_flags & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        /* 空操作 — f_sync 仅需保证数据已在介质上，
         * APP_SD_Card_Write 是阻塞调用，返回时数据已写入。 */
        return RES_OK;

    case GET_SECTOR_COUNT:
        blk_cnt = APP_SD_Card_GetBlockCount();
        if (blk_cnt == 0U)
        {
            return RES_ERROR;
        }
        *(DWORD *)buff = (DWORD)blk_cnt;
        return RES_OK;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512U;
        return RES_OK;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1U;    /* 每次擦除 1 扇区 */
        return RES_OK;

    default:
        return RES_PARERR;
    }
}

/* -------------------------------------------------------------------------- */
/*  FatFs 时间戳回调                                                          */
/* -------------------------------------------------------------------------- */

/* 前置声明来自 rtc.h */
#include "rtc.h"

/**
 ******************************************************************************
 * @brief  FatFs get_fattime：返回 FAT 格式的当前时间戳。
 * @return 包含位字段的 DWORD：年、月、日、时、分、秒/2。
 * @note   通过 APP_RTC_GetDateTime 读取 RTC。若 RTC 读取失败，
 *         回退到 2026-01-01 00:01:02。
 ******************************************************************************
 */
DWORD get_fattime(void)
{
    APP_RTC_DateTime_t dt;

    if (APP_RTC_GetDateTime(&dt) != HAL_OK)
    {
        return ((DWORD)(2026U - 1980U) << 25)
             | ((DWORD)1U << 21)
             | ((DWORD)1U << 16);
    }

    return ((DWORD)(dt.year - 1980U) << 25)
         | ((DWORD)dt.month << 21)
         | ((DWORD)dt.date << 16)
         | ((DWORD)dt.hours << 11)
         | ((DWORD)dt.minutes << 5)
         | ((DWORD)(dt.seconds / 2U));
}
