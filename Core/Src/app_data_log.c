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

#define RECORDS_PER_CYCLE   26U

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

/*
 * 启动 SD 日志会话并写入 CSV 表头。
 *
 * 每次新会话都写入表头，因为可能遇到以下场景：
 *   - 日期翻日 → 新文件需要表头
 *   - 错误恢复 → 上一会话异常终止，文件末尾可能不完整
 * 重复表头行不影响 CSV 解析器（标准允许注释/头行出现在任意位置），
 * 且每个表头仅 56 字节，代价可忽略。
 */
AppDataLogStatus_t APP_DataLog_StartSession(void)
{
    AppSdFileStatus_t ret = APP_SdFile_StartSession();
    if (ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)ret;

    ret = APP_SdFile_Write(CSV_HEADER);
    if (ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)ret;
    csv_header_written = 1;
    return APP_DATA_LOG_OK;
}

/*
 * 写一条完整的 CSV 记录（当前 26 字段：基础 8 项 + RR/IBI/HRV + motion/Poincare + 频域 HRV 扩展）。
 *
 * 每条记录包含：RED, IR, Baseline_IR, Finger, HR, SpO2, SignalQuality, PI_IR,
 *   RR_Valid, RR, IBI_Valid, IBI, HRV_Valid, Mean_IBI, SDNN, RMSSD,
 *   MotionArtifact, MotionScore, SD1, SD2, SD1_SD2, RhythmIrregular,
 *   HRV_Freq_Valid, LF_Power, HF_Power, LF_HF
 * 共调用 emit 26 次（每次写一行）。
 *
 * 写入失败时的提前返回：
 *   任何一次 emit 返回非 OK 状态，立即终止后续字段的写入并向上传播错误。
 *   caller（main.c 的 app_send_report_if_due）会通过 app_update_sd_log_status
 *   将错误状态同步到 AppState，OLED 显示侧据此展示 SD 卡异常。
 */
/*
 * 写一条完整的 CSV 记录（当前 26 字段）。
 *
 * 懒启动策略：
 *   若 SD 日志会话尚未就绪，先尝试 APP_DataLog_StartSession() 挂载 SD、
 *   创建/打开 CSV 并写入表头。失败时 StartSession 内部会记录退避时间戳
 *  （RETRY_INTERVAL_MS），后续写入在退避期内快速返回，不阻塞主循环。
 *   成功时 APP_SdFile_Write 会正常缓冲数据。
 */
AppDataLogStatus_t APP_DataLog_WriteRecord(const AppState_t *app)
{
    char ts[24], st[8], vb[24];
    uint32_t sid;
    AppSdFileStatus_t write_ret;

    if (app == NULL) return APP_DATA_LOG_CLOSED;

    /* 懒启动：延迟到首次写入时才挂载 SD，避免坏卡/无卡阻塞启动流程。
     * APP_DataLog_StartSession() 内部已含 60s 退避 + CSV 表头写入。 */
    if (!APP_DataLog_IsReady())
    {
        AppDataLogStatus_t ret = APP_DataLog_StartSession();
        if (ret != APP_DATA_LOG_OK) return ret;
    }

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

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->rr_valid);
    write_ret = emit(ts, sid, st, "RR_Valid", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->rr_bpm);
    write_ret = emit(ts, sid, st, "RR", vb, "bpm");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->ibi_valid);
    write_ret = emit(ts, sid, st, "IBI_Valid", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->latest_ibi_ms);
    write_ret = emit(ts, sid, st, "IBI", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_valid);
    write_ret = emit(ts, sid, st, "HRV_Valid", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_mean_ibi_ms);
    write_ret = emit(ts, sid, st, "Mean_IBI", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_sdnn_ms);
    write_ret = emit(ts, sid, st, "SDNN", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_rmssd_ms);
    write_ret = emit(ts, sid, st, "RMSSD", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->motion_artifact);
    write_ret = emit(ts, sid, st, "MotionArtifact", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->motion_score);
    write_ret = emit(ts, sid, st, "MotionScore", vb, "0-100");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_sd1_ms);
    write_ret = emit(ts, sid, st, "SD1", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_sd2_ms);
    write_ret = emit(ts, sid, st, "SD2", vb, "ms");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_sd1_sd2_x100);
    write_ret = emit(ts, sid, st, "SD1_SD2", vb, "x100");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->rhythm_irregular);
    write_ret = emit(ts, sid, st, "RhythmIrregular", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_freq_valid);
    write_ret = emit(ts, sid, st, "HRV_Freq_Valid", vb, "bool");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->hrv_lf_power_x100);
    write_ret = emit(ts, sid, st, "LF_Power", vb, "ms2x100");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%lu", (unsigned long)app->hrv_hf_power_x100);
    write_ret = emit(ts, sid, st, "HF_Power", vb, "ms2x100");
    if (write_ret != APP_SD_FILE_OK) return (AppDataLogStatus_t)write_ret;

    (void)snprintf(vb, sizeof(vb), "%u", (unsigned int)app->hrv_lf_hf_x100);
    write_ret = emit(ts, sid, st, "LF_HF", vb, "x100");
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
