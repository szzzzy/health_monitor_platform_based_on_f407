/**
  ******************************************************************************
  * @file    app_sd_file.c
  * @brief   SD 卡 FAT 文件会话管理
  *
  * 职责：
  *   - 挂载 / 卸载 FAT 卷
 *   - 按日创建 / 追加二进制日志文件（YYYYMMDD_NN.BIN）
  *   - 接收日志层的 512 字节分片并执行 f_write/f_sync
  *   - 写失败立即使会话和磁盘驱动失效；60 s 重试由 app_data_log 管理
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

static uint32_t write_op_count = 0U;
static uint32_t total_errors  = 0U;

/* ---- 辅助 ---- */

/**
 * @brief  写入/同步失败后的级联清理：丢弃文件、卸载卷、反初始化卡。
 * @note   级联清理，避免进一步 I/O：
 *         - 使用 memset 清零 FIL 对象（跳过 f_close 以避免 f_sync I/O）
 *         - 使用 f_mount(NULL, "", 0) 注销而不访问磁盘
 *         - 反初始化 SD 卡并使磁盘驱动失效
 *         调用后状态机必须退避后再重试
 */
/*
 * 写入/同步失败后的级联清理：强制失效会话 → 卸载卷 → 反初始化卡。
 *
 * 关键设计：
 *   1. 不调用 f_close() —— FatFs 的 f_close 内部会 f_sync，坏卡/拔卡时
 *      f_sync 的多次 disk_write（文件缓冲 + FAT 表 + 目录项）会逐个超时，
 *      每个超时 200ms，总计可阻塞 1s+。改为直接 memset 清零 FIL 对象，
     *      丢弃未写入数据，O(1) 返回 SDtask 状态机。
 *   2. f_mount(NULL, "", 0) 使用 opt=0（仅注销工作区注册），不触发
 *      disk_ioctl(CTRL_SYNC)，避免在已失效的磁盘上再尝试同步。
 *   3. 最后同步 STA_NOINIT + 记录重试时间戳，确保后续 60s 内不再
 *      尝试挂载/写入。 */
static void close_session_after_error(void)
{
    /* 无论会话激活标志是否置位，始终清零 FIL 对象。
     * 覆盖中间态：f_open 已成功但 f_lseek 失败，或 f_write 已调用
     * 但 f_sync 失败 — 此时文件已打开但会话激活标志可能尚未置 true。
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

/**
 * @brief  验证 RTC 日期/时间结构体的字段。
 * @param  dt 要验证的 RTC 日期/时间结构体指针。
 * @return 若年份 >= 2000、月份在 1-12 之间、日期在 1-31 之间则返回 true。
 */
static bool rtc_is_valid(const APP_RTC_DateTime_t *dt)
{
    return (dt->year >= 2000U) && (dt->month >= 1U) && (dt->month <= 12U)
        && (dt->date >= 1U) && (dt->date <= 31U);
}

/**
 * @brief  将 RTC 时间戳的日期部分格式化为 "YYYYMMDD"。
 * @param  dt   RTC 日期/时间结构体指针。
 * @param  out  格式化字符串的输出缓冲。
 * @param  size 输出缓冲的大小。
 * @note   若 RTC 日期无效（rtc_is_valid 返回 false），
 *         则向输出缓冲写入 "00000000"。
 */
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

/**
 * @brief  将日期字符串打包为 16 位可排序数值键。
 * @param  prefix "YYYYMMDD" 格式的日期字符串。
 * @return 使用公式 ((Y-2000)*372 + M*31 + D) 打包的日期值，
 *         若字符串无法解析则返回 0。
 * @note   用于在不解析完整日期结构的情况下比较文件日期。
 */
static uint16_t pack_date(const char *prefix)
{
    unsigned int y, m, d;
    if (sscanf(prefix, "%4u%2u%2u", &y, &m, &d) == 3)
    {
        return (uint16_t)(((y - 2000U) * 372U) + (m * 31U) + d);
    }
    return 0U;
}

/**
 * @brief  查找给定日期前缀的下一个可用序号。
 * @param  date_prefix "YYYYMMDD" 格式的日期字符串。
 * @return 新 BIN 文件的下一个序号 (0-255)。
 * @note   扫描根目录中匹配 "YYYYMMDD_*.BIN" 的文件，
 *         返回 max(seq) + 1。若 f_findfirst 出错则返回 0。
 */
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

/**
 * @brief  初始化 SD 文件模块状态（仅软件，无 I/O）。
 * @note   清零 FATFS 和 FIL 结构体并重置所有计数器。
 *         必须在任何其他 APP_SdFile 函数之前调用。
 *         不访问 SD 卡或文件系统。
 */
void APP_SdFile_Init(void)
{
    (void)memset(&fatfs, 0, sizeof(fatfs));
    (void)memset(&log_file, 0, sizeof(log_file));
    volume_mounted  = false;
    session_active  = false;
    write_op_count     = 0U;
    total_errors      = 0U;
}

/**
 * @brief  挂载 FAT 卷并创建或追加当前日期的 BIN 会话文件。
 * @return APP_SD_FILE_OK 成功，APP_SD_FILE_NO_CARD 挂载或打开失败，
 *         APP_SD_FILE_WRITE_ERROR 定位文件末尾失败。
 * @note   由 APP_DataLog_Service() 在 SDtask 上下文中调用，因此允许执行物理 I/O；
 *         重试退避由 app_data_log 状态机负责。本函数任一步失败都会统一清理
 *         文件句柄和挂载状态，避免后续重试继承半初始化资源。
 */
AppSdFileStatus_t APP_SdFile_StartSession(void)
{
    FRESULT fr;
    APP_RTC_DateTime_t dt;
    char   date_prefix[16];
    char   file_path[32];
    uint8_t seq;

    if (session_active)
    {
        AppSdFileStatus_t stop_status = APP_SdFile_StopSession();
        if (stop_status != APP_SD_FILE_OK)
        {
            return stop_status;
        }
    }

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

/**
 * @brief  停止当前 SD 日志会话并释放所有资源。
 * @note   执行 f_sync + f_close + 卸载 FAT 卷 + 反初始化 SD 卡。
 *         仅在 finger_present==0（安全窗口）时调用。
 *         使用 f_mount(NULL, "", 1) 的 opt=1 以避免额外磁盘 I/O。
 */
/*
 * 停止文件会话：f_sync + f_close + 卸载卷 + 反初始化卡。
 * 仅在 finger_present==0 的安全窗口调用。
 */
AppSdFileStatus_t APP_SdFile_StopSession(void)
{
    AppSdFileStatus_t status = APP_SD_FILE_OK;
    FRESULT fr;

    if (session_active)
    {
        fr = f_sync(&log_file);
        if (fr != FR_OK)
        {
            status = APP_SD_FILE_SYNC_ERROR;
        }

        fr = f_close(&log_file);
        if ((fr != FR_OK) && (status == APP_SD_FILE_OK))
        {
            status = APP_SD_FILE_CLOSE_ERROR;
        }
        session_active = false;
    }
    if (volume_mounted)
    {
        fr = f_mount(NULL, "", 0);
        if ((fr != FR_OK) && (status == APP_SD_FILE_OK))
        {
            status = APP_SD_FILE_UNMOUNT_ERROR;
        }
        volume_mounted = false;
    }
    APP_SD_Card_Deinit();
    sd_diskio_invalidate();

    if (status != APP_SD_FILE_OK)
    {
        total_errors++;
    }

    return status;
}

/**
 * @brief  检查 SD 日志会话当前是否激活并就绪。
 * @return 若日志文件已打开并接受写入则返回 true，否则返回 false。
 */
bool APP_SdFile_IsReady(void)
{
    return session_active;
}

/**
 * @brief  向打开的日志文件写入二进制数据。
 * @param  data 要写入的数据缓冲指针。
 * @param  len  要写入的字节数。
 * @return APP_SD_FILE_OK 成功，APP_SD_FILE_CLOSED 无活动会话，
 *         APP_SD_FILE_WRITE_ERROR 失败（触发级联清理）。
 * @note   使用直接的 f_write（不经过字符串辅助缓冲）。
 *         f_sync 有意推迟到 APP_SdFile_StopSession 中执行，
 *         以避免阻塞测量循环超 100ms。
 */
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

    /* 不 f_sync — 由停止会话流程内部统一同步。
     * f_sync 可能触发多次 disk_write（文件缓冲 + FAT + 目录），
     * 累计 >100ms，足以在测量期造成 MAX30102 FIFO 溢出。 */
    write_op_count++;
    return APP_SD_FILE_OK;
}

/**
 * @brief  刷新（f_sync）当前打开的日志文件。
 * @return APP_SD_FILE_OK 成功，APP_SD_FILE_CLOSED 无活动会话，
 *         APP_SD_FILE_WRITE_ERROR f_sync 失败（触发会话清理）。
 */
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

/**
 * @brief  获取成功 f_write 调用的总次数。
 * @return 自初始化以来的总刷新/写入计数。
 */
/* 返回成功 f_write 调用次数（非刷新次数、非写入字节/样本数）。 */
uint32_t APP_SdFile_GetWriteOpCount(void) { return write_op_count; }
/**
 * @brief  获取遇到的 SD 写入错误总次数。
 * @return 自初始化以来的总错误计数。
 */
uint32_t APP_SdFile_GetErrorCount(void)  { return total_errors; }
