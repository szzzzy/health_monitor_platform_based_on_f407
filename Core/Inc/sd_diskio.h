#ifndef __SD_DISKIO_H__
#define __SD_DISKIO_H__

#include "diskio.h"

DSTATUS SD_CheckStatus_diskio(BYTE pdrv);

/* 由错误恢复路径调用，将 disk_status_flags 置为 STA_NOINIT，
 * 使 FatFs 感知到磁盘不可用，后续 I/O 操作短路返回。 */
void sd_diskio_invalidate(void);

#endif
