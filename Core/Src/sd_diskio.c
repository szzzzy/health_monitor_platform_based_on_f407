/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   FatFs -> app_sd_card 磁盘 I/O 桥接层
  *
  * 实现 FatFs 要求的 5 个 disk_* 函数 + get_fattime()。
  * 所有 SD 卡访问通过 app_sd_card 抽象层，不直接操作 HAL。
  *
  * 调用链：main.c → APP_DataLog_WriteRecord() → APP_SdFile_Write()
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

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }
    return disk_status_flags;
}

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

/*
 * 显式标记磁盘不可用。
 *
 * APP_SD_Card_Deinit() 只在 app_sd_card 层重置 card_initialized，
 * FatFs 层的 disk_status_flags 不会自动同步。此函数由错误恢复路径
 *（close_session_after_error / StopSession）调用，确保 FatFs 感知
 * 到卡已断开，后续 disk_read/disk_write 在 STA_NOINIT 检查时快速返回
 * RES_NOTRDY，不再尝试 HAL_SD I/O，避免在已 Deinit 的卡上二次阻塞。
 */
void sd_diskio_invalidate(void)
{
    disk_status_flags |= STA_NOINIT;
}

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
