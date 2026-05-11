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

/*
 * 写入/同步失败后的级联清理：关闭文件 → 卸载卷 → 反初始化卡。
 *
 * 为什么不能只关文件：
 *   1. 写入失败意味着底层 SD 卡可能已经拔出或进入错误状态，
 *      只关文件会让下一次 f_write 带着坏 FATFS 结构继续操作，
 *      可能导致文件系统元数据损坏（目录表乱写、FAT 链表断裂）。
 *   2. 必须整栈重建：APP_SD_Card_Deinit 重置 SDIO 和卡状态，
 *      f_mount(NULL) 释放胖文件系统的内部缓存，
 *      确保下一次 Session 从零开始，状态一致。
 *   3. 记录重试时间戳，防止每次 APP_SdFile_Write 都立即尝试
 *      重新 Init 卡（60 秒间隔避免了 SD 上电延迟导致的快速重试风暴）。 */
static void close_session_after_error(void)
{
    if (session_active)
    {
        (void)f_close(&log_file);
        session_active = false;
    }

    if (volume_mounted)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
    }

    APP_SD_Card_Deinit();
    wr_buf_pos = 0U;
    last_retry_tick = HAL_GetTick();
}

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

    while (fno.fname[0] != '\0')
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
        if (f_findnext(&dir, &fno) != FR_OK)
        {
            break;
        }
    }

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
        total_errors++;
        close_session_after_error();
        return APP_SD_FILE_WRITE_ERROR;
    }

    fr = f_sync(&log_file);
    if (fr != FR_OK)
    {
        total_errors++;
        close_session_after_error();
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
        APP_SD_Card_Deinit();
        last_retry_tick = HAL_GetTick();
        return APP_SD_FILE_NO_CARD;
    }

    /* f_lseek 失败通常意味着底层存储介质在打开后变为不可读
     *（例如卡在文件长度查询期间被拔出），此时立即级联清理，
     * 避免后续 f_write 写到无效的文件句柄上。*/
    fr = f_lseek(&log_file, f_size(&log_file));
    if (fr != FR_OK)
    {
        (void)f_close(&log_file);
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
        APP_SD_Card_Deinit();
        total_errors++;
        last_retry_tick = HAL_GetTick();
        return APP_SD_FILE_WRITE_ERROR;
    }

    session_active  = true;
    wr_buf_pos      = 0U;
    last_retry_tick = HAL_GetTick();
    return APP_SD_FILE_OK;
}

/*
 * 停止文件会话：先刷缓冲、再关文件、卸载卷、反初始化卡。
 *
 * 注意：flush_internal() 在遇到写入错误时会调用 close_session_after_error()
 * 自行做完整级联清理，并将 session_active 设为 false。
 * 因此后续的 f_close / f_mount / APP_SD_Card_Deinit 都需要检查
 * session_active / volume_mounted 标志，防止重复关闭/卸载。 */
void APP_SdFile_StopSession(void)
{
    if (session_active)
    {
        (void)flush_internal();
        if (session_active)
        {
            (void)f_close(&log_file);
            session_active = false;
        }
    }
    if (volume_mounted)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
    }
    APP_SD_Card_Deinit();
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
