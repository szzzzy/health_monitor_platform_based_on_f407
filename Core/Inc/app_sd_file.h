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

#define APP_SD_FILE_BUF_SIZE  1024U   /* 保留的文件层缓冲容量配置，单位：字节 */

typedef enum {
    APP_SD_FILE_OK          = 0, /* 操作成功 */
    APP_SD_FILE_NO_CARD     = 1, /* 卡不存在或卷无法挂载 */
    APP_SD_FILE_WRITE_ERROR = 2, /* 打开、定位或写入失败 */
    APP_SD_FILE_DISK_FULL   = 3, /* 写入字节数小于请求长度 */
    APP_SD_FILE_CLOSED      = 4, /* 请求需要会话，但当前未打开 */
    APP_SD_FILE_SYNC_ERROR  = 5, /* f_sync 失败 */
    APP_SD_FILE_CLOSE_ERROR = 6, /* f_close 失败 */
    APP_SD_FILE_UNMOUNT_ERROR = 7 /* f_mount(NULL) 失败 */
} AppSdFileStatus_t;

/** @brief 只清零文件会话 RAM 状态，不挂载卷、不访问 SD 卡。 */
void              APP_SdFile_Init(void);
/** @brief 挂载卷并新建当日递增序号的 BIN 会话文件。 */
AppSdFileStatus_t APP_SdFile_StartSession(void);
/** @brief 同步、关闭并卸载当前会话；返回最先发生的收尾错误。 */
AppSdFileStatus_t APP_SdFile_StopSession(void);
/** @brief 查询卷已挂载且文件会话处于活跃状态。 */
bool              APP_SdFile_IsReady(void);
/** @brief 向当前会话写入指定字节数；短写按磁盘满处理。 */
AppSdFileStatus_t APP_SdFile_WriteBytes(const void *data, uint16_t len);
/** @brief 对当前文件执行 f_sync，但保持会话打开。 */
AppSdFileStatus_t APP_SdFile_Flush(void);
/** @brief 返回成功 f_write 次数和累计文件层错误次数。 */
uint32_t          APP_SdFile_GetWriteOpCount(void);
uint32_t          APP_SdFile_GetErrorCount(void);

#endif
