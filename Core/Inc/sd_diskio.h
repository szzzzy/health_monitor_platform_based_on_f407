#ifndef __SD_DISKIO_H__
#define __SD_DISKIO_H__

/** @file sd_diskio.h @brief FatFs 与 app_sd_card 块设备层之间的状态桥接接口。 */

#include "diskio.h"

/** @brief 查询物理驱动器状态；当前仅支持 pdrv=0。 */
DSTATUS SD_CheckStatus_diskio(BYTE pdrv);

/* 由错误恢复路径调用，将 disk_status_flags 置为 STA_NOINIT，
 * 使 FatFs 感知到磁盘不可用，后续 I/O 操作短路返回。 */
void sd_diskio_invalidate(void);

#endif
