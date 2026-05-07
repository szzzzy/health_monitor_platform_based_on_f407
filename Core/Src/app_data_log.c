/**
  ******************************************************************************
  * @file    app_data_log.c
  * @brief   数据记录 — CSV 格式化 + 委托 app_sd_file 写入 SD 卡
  *
  * CSV 格式：timestamp_utc,sample_id,type,value,unit,status
  * 同一次采样的所有变量共享同一时间戳和 sample_id。
  ******************************************************************************
  */

#include "app_data_log.h"
#include "app_sd_file.h"
#include "rtc.h"
#include <stdio.h>

#define RECORDS_PER_CYCLE   8U

static uint32_t sample_id = 0U;
static char     csv_header_written = 0;
static const char CSV_HEADER[] = "timestamp_utc,sample_id,type,value,unit,status\n";

/* ---- 辅助 ---- */

static bool rtc_ok(const APP_RTC_DateTime_t *dt)
{
    return (dt->year >= 2000U) && (dt->month >= 1U) && (dt->month <= 12U)
        && (dt->date >= 1U) && (dt->date <= 31U);
}

static void fmt_ts(const APP_RTC_DateTime_t *dt, char *out, size_t sz)
{
    if (rtc_ok(dt))
    {
        (void)snprintf(out, sz, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                       (unsigned int)dt->year,  (unsigned int)dt->month,
                       (unsigned int)dt->date,  (unsigned int)dt->hours,
                       (unsigned int)dt->minutes, (unsigned int)dt->seconds);
    }
    else
    {
        (void)snprintf(out, sz, "0000-00-00T00:00:00Z");
    }
}

/* 构建一行 CSV 文本并写入 SD 缓冲区 */
static void emit(const char *ts, uint32_t sid, const char *status,
                 const char *type, const char *val, const char *unit)
{
    char line[APP_DATA_LOG_LINE_MAX];
    (void)snprintf(line, sizeof(line), "%s,%lu,%s,%s,%s,%s\n",
                   ts, (unsigned long)sid, type, val, unit, status);
    (void)APP_SdFile_Write(line);
}

/* ---- 公共 API ---- */

void APP_DataLog_Init(void)
{
    APP_SdFile_Init();
    sample_id = 0U;
    csv_header_written = 0;
}

AppDataLogStatus_t APP_DataLog_StartSession(void)
{
    AppSdFileStatus_t ret = APP_SdFile_StartSession();
    if (ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)ret;

    if (!csv_header_written)
    {
        (void)APP_SdFile_Write(CSV_HEADER);
        csv_header_written = 1;
    }
    return APP_DATA_LOG_OK;
}

AppDataLogStatus_t APP_DataLog_WriteRecord(const AppState_t *app)
{
    char ts[24], st[8], vb[24];
    uint32_t sid;

    if (app == NULL) return APP_DATA_LOG_CLOSED;

    fmt_ts(&app->rtc_datetime, ts, sizeof(ts));

    if (app->rtc_time_valid != 0U)
        (void)snprintf(st, sizeof(st), "OK");
    else
        (void)snprintf(st, sizeof(st), "NO_RTC");

    sid = sample_id++;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->red_value);
    emit(ts, sid, st, "RED", vb, "count");

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->ir_value);
    emit(ts, sid, st, "IR", vb, "count");

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->baseline_ir);
    emit(ts, sid, st, "Baseline_IR", vb, "count");

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->finger_present);
    emit(ts, sid, st, "Finger", vb, "bool");

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->bpm_value);
    emit(ts, sid, st, "HR", vb, "bpm");

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->spo2_value);
    emit(ts, sid, st, "SpO2", vb, "%");

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->signal_quality);
    emit(ts, sid, st, "SignalQuality", vb, "0-100");

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->signal_ir_pi_x1000);
    emit(ts, sid, st, "PI_IR", vb, "x1000");

    return APP_DATA_LOG_OK;
}

AppDataLogStatus_t APP_DataLog_Flush(void)
{
    AppSdFileStatus_t ret = APP_SdFile_Flush();
    return (AppDataLogStatus_t)ret;
}

uint32_t APP_DataLog_GetSampleId(void)
{
    return sample_id;
}
