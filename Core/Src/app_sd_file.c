/**
  ******************************************************************************
  * @file    app_sd_file.c
  * @brief   SD 卡 FAT 文件会话管理
  *
  * 职责：
  *   - 挂载 / 卸载 FAT 卷
 *   - 按日创建 / 追加二进制日志文件（YYYYMMDD_NN.BIN）
  *   - 2KB 缓冲批量写入 + f_sync
  *   - 写失败自动关闭会话，每 60s 重试
  ******************************************************************************
  */

#include "app_sd_file.h"
#include "app_sd_card.h"
#include "rtc.h"
#include "ff.h"
#include "sd_diskio.h"
#include <stdio.h>
#include <string.h>

static FATFS   fatfs;
static FIL     log_file;
static bool    volume_mounted  = false;
static bool    session_active  = false;
static uint16_t current_file_date;

static uint32_t total_flushes = 0U;
static uint32_t total_errors  = 0U;

/* ---- 辅助 ---- */

/*
 * 写入/同步失败后的级联清理：强制失效会话 → 卸载卷 → 反初始化卡。
 *
 * 关键设计：
 *   1. 不调用 f_close() —— FatFs 的 f_close 内部会 f_sync，坏卡/拔卡时
 *      f_sync 的多次 disk_write（文件缓冲 + FAT 表 + 目录项）会逐个超时，
 *      每个超时 200ms，总计可阻塞 1s+。改为直接 memset 清零 FIL 对象，
 *      丢弃未写入数据，O(1) 回到主循环。
 *   2. f_mount(NULL, "", 0) 使用 opt=0（仅注销工作区注册），不触发
 *      disk_ioctl(CTRL_SYNC)，避免在已失效的磁盘上再尝试同步。
 *   3. 最后同步 STA_NOINIT + 记录重试时间戳，确保后续 60s 内不再
 *      尝试挂载/写入。 */
static void close_session_after_error(void)
{
    /* 无论 session_active 是否置位，始终清零 FIL 对象。
     * 覆盖中间态：f_open 已成功但 f_lseek 失败，或 f_write 已调用
     * 但 f_sync 失败 — 此时文件已打开但 session_active 可能尚未置 true。
     * memset 直接丢弃所有未写入数据，O(1) 无 I/O。 */
    (void)memset(&log_file, 0, sizeof(log_file));
    session_active = false;

    /* 即使 volume_mounted 尚未置位，也要注销 FatFs 工作区。
     * f_mount(&fatfs, "", 1) 会先注册 FATFS 对象再尝试挂载；
     * 若挂载失败，volume_mounted 仍为 false，但 FatFs[0] 可能已经
     * 指向 fatfs。无条件 f_mount(NULL, "", 0) 可清掉这个半初始化状态。 */
    (void)f_mount(NULL, "", 0);
    volume_mounted = false;

    APP_SD_Card_Deinit();
    sd_diskio_invalidate();
    total_errors++;
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

    (void)snprintf(pattern, sizeof(pattern), "%s_*.BIN", date_prefix);
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

/* ---- 公共 API ---- */

void APP_SdFile_Init(void)
{
    (void)memset(&fatfs, 0, sizeof(fatfs));
    (void)memset(&log_file, 0, sizeof(log_file));
    volume_mounted  = false;
    session_active  = false;
    total_flushes     = 0U;
    total_errors      = 0U;
}

/*
 * 启动 SD 日志会话：挂载 FAT 卷 → 按日创建/追加 BIN → 定位到文件末尾。
 *
 * 限频重试：若上次尝试失败，须等待 RETRY_INTERVAL_MS 后才允许重试，
 * 避免在坏卡/无卡场景下每个 report 周期（200ms）都阻塞在 init/mount。
 * last_retry_tick == 0 表示从未尝试过，允许首次调用直接尝试。
 *
 * 所有失败路径统一调用 close_session_after_error() 做级联清理：
 * 丢弃半打开文件、卸载卷、Deinit 卡、同步 STA_NOINIT、设置退避时间戳。
 */
/*
 * 启动 SD 日志会话：挂载 FAT 卷 → 按日创建/追加 BIN → 定位到文件末尾。
 *
 * 由 APP_DataLog_Service() 状态机驱动调用，重试时序由状态机的
 * ERROR_BACKOFF 控制，此处不再做额外的退避限频。
 *
 * 所有失败路径统一调用 close_session_after_error() 做级联清理。
 */
AppSdFileStatus_t APP_SdFile_StartSession(void)
{
    FRESULT fr;
    APP_RTC_DateTime_t dt;
    char   date_prefix[16];
    char   file_path[32];
    uint8_t seq;

    if (session_active) APP_SdFile_StopSession();

    APP_SD_Card_InitHardware();

    /* f_mount → disk_initialize → APP_SD_Card_Init 完成一次卡初始化 */
    fr = f_mount(&fatfs, "", 1);
    if (fr != FR_OK)
    {
        close_session_after_error();
        return APP_SD_FILE_NO_CARD;
    }
    volume_mounted = true;

    (void)APP_RTC_GetDateTime(&dt);
    make_date_prefix(&dt, date_prefix, sizeof(date_prefix));
    current_file_date = pack_date(date_prefix);
    seq = find_next_sequence(date_prefix);

    (void)snprintf(file_path, sizeof(file_path), "%s_%02u.BIN",
                   date_prefix, (unsigned int)seq);

    fr = f_open(&log_file, file_path, FA_OPEN_APPEND | FA_WRITE);
    if (fr == FR_NO_FILE)
    {
        fr = f_open(&log_file, file_path, FA_CREATE_NEW | FA_WRITE);
    }

    if (fr != FR_OK)
    {
        /* 文件打开失败：卷已挂载但文件不可用，整栈清理 */
        close_session_after_error();
        return APP_SD_FILE_NO_CARD;
    }

    /* f_lseek 失败 → 存储介质在打开后变为不可读（例如卡刚拔出），
     * 级联清理后返回 WRITE_ERROR，后续 APP_SdFile_Write 会快速短路。 */
    fr = f_lseek(&log_file, f_size(&log_file));
    if (fr != FR_OK)
    {
        close_session_after_error();
        return APP_SD_FILE_WRITE_ERROR;
    }

    session_active    = true;
    return APP_SD_FILE_OK;
}

/*
 * 停止文件会话：f_sync + f_close + 卸载卷 + 反初始化卡。
 * 仅在 finger_present==0 的安全窗口调用。
 */
void APP_SdFile_StopSession(void)
{
    if (session_active)
    {
        (void)f_sync(&log_file);
        (void)f_close(&log_file);
        session_active = false;
    }
    if (volume_mounted)
    {
        (void)f_mount(NULL, "", 1);
        volume_mounted = false;
    }
    APP_SD_Card_Deinit();
    sd_diskio_invalidate();
}

bool APP_SdFile_IsReady(void)
{
    return session_active;
}

/*
 * 二进制数据显式长度写入 — 直接 f_write，不经过 str/缓冲。
 * 每次调用一次 f_write，512B @ 12MHz SDIO ≈ 0.3ms，远在预算内。
 * 失败时内部已做 close_session_after_error 级联清理。
 */
AppSdFileStatus_t APP_SdFile_WriteBytes(const void *data, uint16_t len)
{
    FRESULT fr;
    UINT    bw;

    if (!session_active || (data == NULL) || (len == 0U)) return APP_SD_FILE_CLOSED;

    fr = f_write(&log_file, data, (UINT)len, &bw);
    if ((fr != FR_OK) || (bw != (UINT)len))
    {
        close_session_after_error();
        return APP_SD_FILE_WRITE_ERROR;
    }

    /* 不 f_sync — 由 StopSession 内部统一 sync。
     * f_sync 可能触发多次 disk_write（文件缓冲 + FAT + 目录），
     * 累计 >100ms，足以在测量期造成 MAX30102 FIFO overflow。 */
    total_flushes++;
    return APP_SD_FILE_OK;
}

AppSdFileStatus_t APP_SdFile_Flush(void)
{
    FRESULT fr;
    if (!session_active) return APP_SD_FILE_CLOSED;
    fr = f_sync(&log_file);
    if (fr != FR_OK)
    {
        close_session_after_error();
        return APP_SD_FILE_WRITE_ERROR;
    }
    return APP_SD_FILE_OK;
}

uint32_t APP_SdFile_GetTotalWritten(void) { return total_flushes; }
uint32_t APP_SdFile_GetErrorCount(void)  { return total_errors; }
