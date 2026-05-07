/**
  ******************************************************************************
  * @file    app_data_log.h
  * @brief   数据记录模块 API（CSV 格式化，委托 app_sd_file 写卡）
  *
  * CSV 格式：timestamp_utc,sample_id,type,value,unit,status
  * 同一次采样的所有变量共享同一时间戳和 sample_id。
  ******************************************************************************
  */

#ifndef __APP_DATA_LOG_H__
#define __APP_DATA_LOG_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_state.h"

#define APP_DATA_LOG_LINE_MAX  96U

typedef enum {
    APP_DATA_LOG_OK          = 0,
    APP_DATA_LOG_NO_CARD     = 1,
    APP_DATA_LOG_WRITE_ERROR = 2,
    APP_DATA_LOG_DISK_FULL   = 3,
    APP_DATA_LOG_CLOSED      = 4
} AppDataLogStatus_t;

void               APP_DataLog_Init(void);
AppDataLogStatus_t APP_DataLog_StartSession(void);
AppDataLogStatus_t APP_DataLog_WriteRecord(const AppState_t *app);
AppDataLogStatus_t APP_DataLog_Flush(void);
uint32_t           APP_DataLog_GetSampleId(void);

#endif
