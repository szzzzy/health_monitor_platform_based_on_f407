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

/*
 * 构建一行 CSV 文本并写入 SD 缓冲区。
 *
 * 返回值改为 AppSdFileStatus_t：
 *   原来忽略 APP_SdFile_Write 的返回值（(void)），如果 SD 卡写入失败
 *   （例如卡已满、中途拔出），后续的 emit 调用会带着失效的 session
 *   继续格式化字符串并尝试写入，白白浪费 CPU 时间。
 *   现在将返回值向上传播，调用方可以在第一次失败时就中止整条记录，
 *   避免 8 次无效的格式化 + 写入操作。
 */
static AppSdFileStatus_t emit(const char *ts, uint32_t sid, const char *status,
                              const char *type, const char *val, const char *unit)
{
    char line[APP_DATA_LOG_LINE_MAX];
    (void)snprintf(line, sizeof(line), "%s,%lu,%s,%s,%s,%s\n",
                   ts, (unsigned long)sid, type, val, unit, status);
    return APP_SdFile_Write(line);
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
        ret = APP_SdFile_Write(CSV_HEADER);
        if (ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)ret;
        csv_header_written = 1;
    }
    return APP_DATA_LOG_OK;
}

/*
 * 写一条完整的 8 字段 CSV 记录。
 *
 * 每条记录包含：时间戳、采样号、手指标志、HR、SpO2、信号质量、PI 等。
 * 共调用 emit 8 次（每次写一行）。
 *
 * 写入失败时的提前返回：
 *   任何一次 emit 返回非 OK 状态，立即终止后续字段的写入并向上传播错误。
 *   这避免了在 SD 卡已失效的情况下继续执行剩余 7 次格式化+写入操作。
 *   caller（main.c 的 app_send_report_if_due）会通过 app_update_sd_log_status
 *   将错误状态同步到 AppState，OLED 显示侧据此展示 SD 卡异常。
 */
AppDataLogStatus_t APP_DataLog_WriteRecord(const AppState_t *app)
{
    char ts[24], st[8], vb[24];
    uint32_t sid;
    AppSdFileStatus_t write_ret;

    if (app == NULL) return APP_DATA_LOG_CLOSED;

    fmt_ts(&app->rtc_datetime, ts, sizeof(ts));

    if (app->rtc_time_valid != 0U)
        (void)snprintf(st, sizeof(st), "OK");
    else
        (void)snprintf(st, sizeof(st), "NO_RTC");

    sid = sample_id++;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->red_value);
    write_ret = emit(ts, sid, st, "RED", vb, "count");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->ir_value);
    write_ret = emit(ts, sid, st, "IR", vb, "count");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->baseline_ir);
    write_ret = emit(ts, sid, st, "Baseline_IR", vb, "count");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->finger_present);
    write_ret = emit(ts, sid, st, "Finger", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->bpm_value);
    write_ret = emit(ts, sid, st, "HR", vb, "bpm");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->spo2_value);
    write_ret = emit(ts, sid, st, "SpO2", vb, "%");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->signal_quality);
    write_ret = emit(ts, sid, st, "SignalQuality", vb, "0-100");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->signal_ir_pi_x1000);
    write_ret = emit(ts, sid, st, "PI_IR", vb, "x1000");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

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

/*
 * 查询当前 SD 日志会话是否可用，供 AppState 同步状态到显示层。
 *
 * 在 sd_card_ready 和 sd_log_active 标志中反映卡和文件系统的实时状态，
 * OLED 侧据此显示 "SD OK" / "SD ERR" 等状态提示。
 */
bool APP_DataLog_IsReady(void)
{
    return APP_SdFile_IsReady();
}

/* 返回自本次上电以来的累计落盘次数（每次 flush 计一次） */
uint32_t APP_DataLog_GetTotalWritten(void)
{
    return APP_SdFile_GetTotalWritten();
}

/* 返回自本次上电以来的累计写错误次数（写失败 + f_sync 失败 + lseek 失败） */
uint32_t APP_DataLog_GetErrorCount(void)
{
    return APP_SdFile_GetErrorCount();
}
