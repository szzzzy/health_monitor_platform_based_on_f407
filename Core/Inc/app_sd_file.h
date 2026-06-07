/**
  ******************************************************************************
  * @file    app_sd_file.h
  * @brief   SD 卡文件会话管理（挂载、打开、缓冲写、异常恢复）
  *
  * 与 app_data_log 的分工：
 *   app_sd_file — 所有 FAT 文件系统操作
 *   app_data_log — 二进制记录分片，调用 app_sd_file 完成写入
  ******************************************************************************
  */

#ifndef __APP_SD_FILE_H__
#define __APP_SD_FILE_H__

#include <stdint.h>
#include <stdbool.h>

#define APP_SD_FILE_BUF_SIZE  1024U   /* 1KB — 每 2 chunk 物理写入一次，防止突发 */

typedef enum {
    APP_SD_FILE_OK          = 0,
    APP_SD_FILE_NO_CARD     = 1,
    APP_SD_FILE_WRITE_ERROR = 2,
    APP_SD_FILE_DISK_FULL   = 3,
    APP_SD_FILE_CLOSED      = 4
} AppSdFileStatus_t;

void              APP_SdFile_Init(void);
AppSdFileStatus_t APP_SdFile_StartSession(void);
void              APP_SdFile_StopSession(void);
bool              APP_SdFile_IsReady(void);
AppSdFileStatus_t APP_SdFile_WriteBytes(const void *data, uint16_t len);
AppSdFileStatus_t APP_SdFile_Flush(void);
uint32_t          APP_SdFile_GetTotalWritten(void);
uint32_t          APP_SdFile_GetErrorCount(void);

#endif
