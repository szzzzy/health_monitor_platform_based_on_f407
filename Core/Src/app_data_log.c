/**
  ******************************************************************************
  * @file    app_data_log.c
  * @brief   MicroSD 数据记录模块实现
  *
  * CSV 格式：timestamp_utc,sample_id,type,value,unit,status
  * 同一次采样的所有变量共享同一时间戳和 sample_id。
  *
  * 写策略：
  *   - 2KB 内部缓冲区，累积后批量 f_write + f_sync
  *   - 缓冲区接近上限时自动落盘
  *
  * 异常恢复：
  *   - f_write 失败 → 关闭 / 卸载 → 标记失效 → 每 60s 重试
  ******************************************************************************
  */

#include "app_data_log.h"
#include "app_sd_card.h"
#include "rtc.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* 内部状态 */
static FATFS   fatfs;
static FIL     log_file;
static bool    volume_mounted  = false;
static bool    session_active  = false;
static uint16_t current_file_date;

static char     log_buf[APP_DATA_LOG_BUF_SIZE];
static uint16_t log_buf_pos = 0U;

static uint32_t sample_id    = 0U;
static uint32_t total_written = 0U;
static uint32_t total_errors  = 0U;
static uint32_t last_retry_tick = 0U;

#define RETRY_INTERVAL_MS     60000U
#define RECORDS_PER_WRITE     8U     /* 每周期 8 条记录 */
#define CSV_HEADER            "timestamp_utc,sample_id,type,value,unit,status\n"

/* ---- 辅助函数 ---- */

static bool datetime_is_valid(const APP_RTC_DateTime_t *dt)
{
    return (dt->year >= 2000U) && (dt->month >= 1U) && (dt->month <= 12U)
        && (dt->date >= 1U) && (dt->date <= 31U);
}

static void format_iso8601(const APP_RTC_DateTime_t *dt, char *out, size_t size)
{
    if (datetime_is_valid(dt))
    {
        (void)snprintf(out, size, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                       (unsigned int)dt->year,  (unsigned int)dt->month,
                       (unsigned int)dt->date,  (unsigned int)dt->hours,
                       (unsigned int)dt->minutes, (unsigned int)dt->seconds);
    }
    else
    {
        (void)snprintf(out, size, "0000-00-00T00:00:00Z");
    }
}

static void make_date_prefix(const APP_RTC_DateTime_t *dt, char *out, size_t size)
{
    if (datetime_is_valid(dt))
    {
        (void)snprintf(out, size, "%04u%02u%02u",
                       (unsigned int)dt->year,
                       (unsigned int)dt->month,
                       (unsigned int)dt->date);
    }
    else
    {
        (void)snprintf(out, size, "00000000");
    }
}

static uint16_t pack_date(const char *prefix)
{
    unsigned int y, m, d;
    if (sscanf(prefix, "%4u%2u%2u", &y, &m, &d) == 3)
    {
        return (uint16_t)(((y - 2000U) * 372U) + (m * 31U) + d);
    }
    return 0U;
}

static uint8_t find_next_sequence(const char *date_prefix)
{
    DIR           dir;
    FILINFO       fno;
    uint8_t       next_seq = 0U;
    char          pattern[16];

    (void)snprintf(pattern, sizeof(pattern), "%s_*.CSV", date_prefix);

    if (f_findfirst(&dir, &fno, "", pattern) != FR_OK)
    {
        return 0U;
    }

    do
    {
        const char *underscore = strchr(fno.fname, '_');
        const char *dot        = strchr(fno.fname, '.');

        if ((underscore != NULL) && (dot != NULL) && (dot > underscore))
        {
            unsigned int seq;
            if (sscanf(underscore + 1, "%2u", &seq) == 1)
            {
                if ((uint8_t)(seq + 1U) > next_seq)
                {
                    next_seq = (uint8_t)(seq + 1U);
                }
            }
        }
    } while (f_findnext(&dir, &fno) == FR_OK);

    f_closedir(&dir);
    return next_seq;
}

static AppDataLogStatus_t flush_buffer(void)
{
    FRESULT fr;
    UINT    bw;

    if (!session_active || (log_buf_pos == 0U))
    {
        return APP_DATA_LOG_OK;
    }

    fr = f_write(&log_file, log_buf, log_buf_pos, &bw);
    if ((fr != FR_OK) || (bw != log_buf_pos))
    {
        session_active = false;
        total_errors++;
        return APP_DATA_LOG_WRITE_ERROR;
    }

    fr = f_sync(&log_file);
    if (fr != FR_OK)
    {
        session_active = false;
        total_errors++;
        return APP_DATA_LOG_WRITE_ERROR;
    }

    log_buf_pos = 0U;
    return APP_DATA_LOG_OK;
}

/* 写一行记录到缓冲区 */
static void append_record(const char *ts, uint32_t sid, const char *status,
                          const char *type, const char *value, const char *unit)
{
    int len = snprintf(log_buf + log_buf_pos,
                       APP_DATA_LOG_LINE_MAX,
                       "%s,%lu,%s,%s,%s,%s\n",
                       ts, (unsigned long)sid, type, value, unit, status);
    log_buf_pos += (uint16_t)len;
}

/* ---- 公共 API ---- */

void APP_DataLog_Init(void)
{
    (void)memset(&fatfs, 0, sizeof(fatfs));
    (void)memset(&log_file, 0, sizeof(log_file));
    volume_mounted  = false;
    session_active  = false;
    log_buf_pos     = 0U;
    sample_id       = 0U;
    total_written   = 0U;
    total_errors    = 0U;
    last_retry_tick = 0U;
}

AppDataLogStatus_t APP_DataLog_StartSession(void)
{
    FRESULT fr;
    APP_RTC_DateTime_t dt;
    char   date_prefix[16];
    char   file_path[32];
    uint8_t seq;

    if (session_active)
    {
        APP_DataLog_StopSession();
    }

    APP_SD_Card_InitHardware();
    if (APP_SD_Card_Init() != APP_SD_CARD_OK)
    {
        return APP_DATA_LOG_NO_CARD;
    }

    fr = f_mount(&fatfs, "", 1);
    if (fr != FR_OK)
    {
        return APP_DATA_LOG_NO_CARD;
    }
    volume_mounted = true;

    (void)APP_RTC_GetDateTime(&dt);
    make_date_prefix(&dt, date_prefix, sizeof(date_prefix));
    current_file_date = pack_date(date_prefix);
    seq = find_next_sequence(date_prefix);

    (void)snprintf(file_path, sizeof(file_path), "%s_%02u.CSV",
                   date_prefix, (unsigned int)seq);

    fr = f_open(&log_file, file_path, FA_OPEN_APPEND | FA_WRITE);
    if (fr == FR_NO_FILE)
    {
        fr = f_open(&log_file, file_path, FA_CREATE_NEW | FA_WRITE);
        if (fr == FR_OK)
        {
            (void)f_puts(CSV_HEADER, &log_file);
            (void)f_sync(&log_file);
        }
    }

    if (fr != FR_OK)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
        return APP_DATA_LOG_NO_CARD;
    }

    (void)f_lseek(&log_file, f_size(&log_file));

    session_active = true;
    log_buf_pos    = 0U;
    last_retry_tick = HAL_GetTick();
    return APP_DATA_LOG_OK;
}

void APP_DataLog_StopSession(void)
{
    if (session_active)
    {
        (void)flush_buffer();
        (void)f_close(&log_file);
        session_active = false;
    }

    if (volume_mounted)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
    }

    log_buf_pos = 0U;
}

bool APP_DataLog_IsReady(void)
{
    return session_active;
}

AppDataLogStatus_t APP_DataLog_WriteRecord(const AppState_t *app)
{
    uint32_t now;
    char     ts[24];
    char     status_str[8];
    char     val_buf[24];
    uint32_t local_sid;

    if (app == NULL)
    {
        return APP_DATA_LOG_CLOSED;
    }

    if (!session_active)
    {
        now = HAL_GetTick();
        if ((now - last_retry_tick) < RETRY_INTERVAL_MS)
        {
            return APP_DATA_LOG_NO_CARD;
        }
        last_retry_tick = now;
        return APP_DataLog_StartSession();
    }

    /* 日期翻日检测 */
    {
        APP_RTC_DateTime_t dt;
        char date_prefix[16];

        (void)APP_RTC_GetDateTime(&dt);
        make_date_prefix(&dt, date_prefix, sizeof(date_prefix));
        if (pack_date(date_prefix) != current_file_date)
        {
            APP_DataLog_StopSession();
            return APP_DataLog_StartSession();
        }
    }

    format_iso8601(&app->rtc_datetime, ts, sizeof(ts));

    /* 状态字段 */
    if (app->rtc_time_valid != 0U)
    {
        (void)snprintf(status_str, sizeof(status_str), "OK");
    }
    else
    {
        (void)snprintf(status_str, sizeof(status_str), "NO_RTC");
    }

    /* 分配 sample_id — 同一周期内所有行共享 */
    local_sid = sample_id;
    sample_id++;

    /* 缓冲区空间检查 */
    if ((log_buf_pos + (APP_DATA_LOG_LINE_MAX * RECORDS_PER_WRITE)) >= APP_DATA_LOG_BUF_SIZE)
    {
        AppDataLogStatus_t ret = flush_buffer();
        if (ret != APP_DATA_LOG_OK)
        {
            (void)APP_DataLog_StopSession();
            return ret;
        }
    }

    /* 逐行写入 */
    (void)snprintf(val_buf, sizeof(val_buf), "%lu", (unsigned long)app->red_value);
    append_record(ts, local_sid, status_str, "RED", val_buf, "count");

    (void)snprintf(val_buf, sizeof(val_buf), "%lu", (unsigned long)app->ir_value);
    append_record(ts, local_sid, status_str, "IR", val_buf, "count");

    (void)snprintf(val_buf, sizeof(val_buf), "%lu", (unsigned long)app->baseline_ir);
    append_record(ts, local_sid, status_str, "Baseline_IR", val_buf, "count");

    (void)snprintf(val_buf, sizeof(val_buf), "%u", (unsigned int)app->finger_present);
    append_record(ts, local_sid, status_str, "Finger", val_buf, "bool");

    (void)snprintf(val_buf, sizeof(val_buf), "%u", (unsigned int)app->bpm_value);
    append_record(ts, local_sid, status_str, "HR", val_buf, "bpm");

    (void)snprintf(val_buf, sizeof(val_buf), "%u", (unsigned int)app->spo2_value);
    append_record(ts, local_sid, status_str, "SpO2", val_buf, "%");

    /* 信号质量作为诊断记录 */
    (void)snprintf(val_buf, sizeof(val_buf), "%u", (unsigned int)app->signal_quality);
    append_record(ts, local_sid, status_str, "SignalQuality", val_buf, "0-100");

    /* 灌注指数 */
    (void)snprintf(val_buf, sizeof(val_buf), "%u", (unsigned int)app->signal_ir_pi_x1000);
    append_record(ts, local_sid, status_str, "PI_IR", val_buf, "x1000");

    total_written++;

    return APP_DATA_LOG_OK;
}

AppDataLogStatus_t APP_DataLog_Flush(void)
{
    if (!session_active)
    {
        return APP_DATA_LOG_CLOSED;
    }

    AppDataLogStatus_t ret = flush_buffer();
    if (ret != APP_DATA_LOG_OK)
    {
        (void)APP_DataLog_StopSession();
    }
    return ret;
}

uint32_t APP_DataLog_GetSampleId(void)
{
    return sample_id;
}

uint32_t APP_DataLog_GetTotalWritten(void)
{
    return total_written;
}

uint32_t APP_DataLog_GetErrorCount(void)
{
    return total_errors;
}
