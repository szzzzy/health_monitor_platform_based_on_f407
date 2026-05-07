/**
  ******************************************************************************
  * @file    app_sd_file.c
  * @brief   SD 卡 FAT 文件会话管理
  *
  * 职责：
  *   - 挂载 / 卸载 FAT 卷
  *   - 按日创建 / 追加 CSV 文件（YYYYMMDD_NN.CSV）
  *   - 2KB 缓冲批量写入 + f_sync
  *   - 写失败自动关闭会话，每 60s 重试
  ******************************************************************************
  */

#include "app_sd_file.h"
#include "app_sd_card.h"
#include "rtc.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

#define RETRY_INTERVAL_MS  60000U

static FATFS   fatfs;
static FIL     log_file;
static bool    volume_mounted  = false;
static bool    session_active  = false;
static uint16_t current_file_date;

static char     wr_buf[APP_SD_FILE_BUF_SIZE];
static uint16_t wr_buf_pos = 0U;

static uint32_t total_flushes = 0U;
static uint32_t total_errors  = 0U;
static uint32_t last_retry_tick = 0U;

/* ---- 辅助 ---- */

static bool rtc_is_valid(const APP_RTC_DateTime_t *dt)
{
    return (dt->year >= 2000U) && (dt->month >= 1U) && (dt->month <= 12U)
        && (dt->date >= 1U) && (dt->date <= 31U);
}

static void make_date_prefix(const APP_RTC_DateTime_t *dt, char *out, size_t size)
{
    if (rtc_is_valid(dt))
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
    DIR     dir;
    FILINFO fno;
    uint8_t next_seq = 0U;
    char    pattern[16];

    (void)snprintf(pattern, sizeof(pattern), "%s_*.CSV", date_prefix);
    if (f_findfirst(&dir, &fno, "", pattern) != FR_OK) return 0U;

    do
    {
        const char *u = strchr(fno.fname, '_');
        const char *d = strchr(fno.fname, '.');
        if (u && d && (d > u))
        {
            unsigned int seq;
            if (sscanf(u + 1, "%2u", &seq) == 1)
            {
                if ((uint8_t)(seq + 1U) > next_seq) next_seq = (uint8_t)(seq + 1U);
            }
        }
    } while (f_findnext(&dir, &fno) == FR_OK);

    f_closedir(&dir);
    return next_seq;
}

static AppSdFileStatus_t flush_internal(void)
{
    FRESULT fr;
    UINT    bw;

    if (!session_active || (wr_buf_pos == 0U)) return APP_SD_FILE_OK;

    fr = f_write(&log_file, wr_buf, wr_buf_pos, &bw);
    if ((fr != FR_OK) || (bw != wr_buf_pos))
    {
        session_active = false;
        total_errors++;
        return APP_SD_FILE_WRITE_ERROR;
    }

    fr = f_sync(&log_file);
    if (fr != FR_OK)
    {
        session_active = false;
        total_errors++;
        return APP_SD_FILE_WRITE_ERROR;
    }

    wr_buf_pos = 0U;
    total_flushes++;
    return APP_SD_FILE_OK;
}

static bool check_date_rollover(void)
{
    APP_RTC_DateTime_t dt;
    char prefix[16];

    (void)APP_RTC_GetDateTime(&dt);
    make_date_prefix(&dt, prefix, sizeof(prefix));
    return (pack_date(prefix) != current_file_date);
}

/* ---- 公共 API ---- */

void APP_SdFile_Init(void)
{
    (void)memset(&fatfs, 0, sizeof(fatfs));
    (void)memset(&log_file, 0, sizeof(log_file));
    volume_mounted  = false;
    session_active  = false;
    wr_buf_pos      = 0U;
    total_flushes   = 0U;
    total_errors    = 0U;
    last_retry_tick = 0U;
}

AppSdFileStatus_t APP_SdFile_StartSession(void)
{
    FRESULT fr;
    APP_RTC_DateTime_t dt;
    char   date_prefix[16];
    char   file_path[32];
    uint8_t seq;

    if (session_active) APP_SdFile_StopSession();

    APP_SD_Card_InitHardware();
    if (APP_SD_Card_Init() != APP_SD_CARD_OK) return APP_SD_FILE_NO_CARD;

    fr = f_mount(&fatfs, "", 1);
    if (fr != FR_OK) return APP_SD_FILE_NO_CARD;
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
    }

    if (fr != FR_OK)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
        return APP_SD_FILE_NO_CARD;
    }

    (void)f_lseek(&log_file, f_size(&log_file));

    session_active  = true;
    wr_buf_pos      = 0U;
    last_retry_tick = HAL_GetTick();
    return APP_SD_FILE_OK;
}

void APP_SdFile_StopSession(void)
{
    if (session_active)
    {
        (void)flush_internal();
        (void)f_close(&log_file);
        session_active = false;
    }
    if (volume_mounted)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
    }
    wr_buf_pos = 0U;
}

bool APP_SdFile_IsReady(void)
{
    return session_active;
}

AppSdFileStatus_t APP_SdFile_Write(const char *str)
{
    size_t len;

    if (str == NULL) return APP_SD_FILE_CLOSED;

    /* 无会话则尝试启动 */
    if (!session_active)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_retry_tick) < RETRY_INTERVAL_MS) return APP_SD_FILE_NO_CARD;
        last_retry_tick = now;
        return APP_SdFile_StartSession();
    }

    /* 日期翻日 */
    if (check_date_rollover())
    {
        APP_SdFile_StopSession();
        return APP_SdFile_StartSession();
    }

    len = strlen(str);
    if (len == 0U) return APP_SD_FILE_OK;

    /* 缓冲区满则先落盘 */
    if ((wr_buf_pos + (uint16_t)len) >= APP_SD_FILE_BUF_SIZE)
    {
        AppSdFileStatus_t ret = flush_internal();
        if (ret != APP_SD_FILE_OK)
        {
            (void)APP_SdFile_StopSession();
            return ret;
        }
    }

    (void)memcpy(wr_buf + wr_buf_pos, str, len);
    wr_buf_pos += (uint16_t)len;

    return APP_SD_FILE_OK;
}

AppSdFileStatus_t APP_SdFile_Flush(void)
{
    if (!session_active) return APP_SD_FILE_CLOSED;
    AppSdFileStatus_t ret = flush_internal();
    if (ret != APP_SD_FILE_OK) (void)APP_SdFile_StopSession();
    return ret;
}

uint32_t APP_SdFile_GetTotalWritten(void) { return total_flushes; }
uint32_t APP_SdFile_GetErrorCount(void)  { return total_errors; }
