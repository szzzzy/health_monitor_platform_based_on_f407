/**
  ******************************************************************************
  * @file    app_data_log.h
  * @brief   SD 数据日志 — 二进制环形缓冲 + 后台分片写入
  *
  * 架构：
  *   实时路径 (100Hz):   PushSample()  → 固定 16 字节样本入队，O(1)
  *   后台路径 (SDtask):    ServiceBudget(budget_ms) → 分片 + f_write
  *   停止路径 (手指离开): OnMeasurementStop() → 延迟排空 → f_sync/f_close
  *
  * 关键约束：
  *   - PushSample 不做任何 snprintf / f_write / f_sync
  *   - ServiceBudget 每轮至多一次 f_write (512B 分片)，至多 budget_ms 时间
  *   - 环形缓冲满时覆盖最旧记录并累计 dropped，不阻塞采样
  ******************************************************************************
  */

#ifndef __APP_DATA_LOG_H__
#define __APP_DATA_LOG_H__

#include <stdint.h>

#define DATA_LOG_MODE_ROLLING_LAST_WINDOW 1U     /* RAM 满时保留最近窗口、覆盖最旧记录 */
#define DATA_LOG_RETENTION_SAMPLES       2048U   /* RAM 保留样本数：约 20.48 s @ 100 Hz */
#define DATA_LOG_RETENTION_MS            20480UL /* 对外声明的保留窗口，单位：ms */
#define DATA_LOG_RING_SAMPLES             DATA_LOG_RETENTION_SAMPLES
#define DATA_LOG_CHUNK_SAMPLES    32U   /* 512 字节 = 1 SD 扇区 */

/* 紧凑二进制样本记录 (16 字节)，无动态内存 */
typedef struct __attribute__((packed)) {
  uint32_t tick;     /* PPG 样本时间戳，单位：ms */
  uint32_t red;      /* MAX30102 RED 原始 18 位样本 */
  uint32_t ir;       /* MAX30102 IR 原始 18 位样本 */
  int16_t  ecg;      /* 记录时刻最近的 ECG 滤波值 */
  uint8_t  flags;    /* 位0:手指就位, 位1:接触稳定, 位2:FIFO 溢出 */
  uint8_t  seq;      /* 8 位递增记录序号，用于离线识别缺口 */
} DataLogRawSample_t;

/* 二进制文件头 (32 字节)，每个会话文件开头写入一次 */
typedef struct __attribute__((packed)) {
  uint8_t  magic[4];      /* "BMLG" */
  uint16_t version;       /* 格式版本 2：reserved[] 中含滚动窗口元数据 */
  uint16_t sample_rate_hz;/* PPG 记录采样率，单位：Hz */
  uint32_t start_tick;    /* 会话启动时 HAL Tick，单位：ms */
  uint8_t  reserved[20];  /* [0]=模式，[1..4]=保留毫秒数，[5..8]=启动前丢弃数 */
} DataLogFileHeader_t;

/* SD 日志运行时状态，供 OLED / UART 查询 */
typedef struct {
  uint16_t buffered;       /* 环形缓冲中待写入样本数 */
  uint16_t dropped;        /* 累计丢弃样本数（环形缓冲溢出） */
  uint16_t written;        /* 本次会话已持久化样本数 */
  uint32_t total_written;  /* 累计持久化样本总数（跨会话） */
  uint8_t  paused;         /* 1 = SD 写入已暂停（慢/满/错误） */
  uint8_t  sd_error;       /* 最后一次 SD 错误码 (AppSdFileStatus_t) */
  uint8_t  state;          /* 内部状态机阶段 (SdLogState_t) */
  uint32_t last_write_ms;  /* 最近一次 SD 写入耗时 (ms) */
  uint32_t last_backlog;   /* 最近一次观测到的堆积量 */
  uint32_t retention_ms;   /* RAM 滚动保留窗口，单位：ms */
  uint32_t unsynced;       /* 已交给 f_write、尚未经停止同步确认的样本数 */
  uint32_t sync_error_count; /* f_sync/f_close 失败累计次数 */
  uint8_t  rolling_mode;   /* DATA_LOG_MODE_* 当前策略 */
} DataLogStatus_t;

/* ---- 公共 API ---- */

void     APP_DataLog_Init(void);

/*
 * 实时路径：将原始样本写入环形缓冲。仅做固定字段赋值，零格式化。
 * 调用方 (app_measurement_read_sensor_sample) 每成功读一个 FIFO 样本调用一次。
 */
void     APP_DataLog_PushSample(uint32_t tick, uint32_t red, uint32_t ir,
                                int16_t ecg, uint8_t flags);

/*
 * SDtask 路径：按预算写 SD。每轮至多写一个 512B 分片。
 *   budget_ms: 本轮允许的最大耗时 (ms)，超出后立即返回
 * 返回实际写入的字节数（0 或 512）。
 */
uint16_t APP_DataLog_ServiceBudget(uint32_t budget_ms);

/* 手指离开 / 测量停止时调用，排空剩余缓冲并 f_sync。 */
void     APP_DataLog_OnMeasurementStop(void);

/* 查询运行时状态。 */
void     APP_DataLog_GetStatus(DataLogStatus_t *status);

/* 遗留兼容：供 app_state 快速查询是否活跃。 */
uint8_t  APP_DataLog_IsActive(void);

/*
 * 测量活跃门控：active=1 时 ServiceBudget/ServiceDeferredStop 直接返回 0，
 * 禁止一切物理 SD I/O（包括 f_write/f_sync/f_close）。由 SD 任务每轮更新。
 */
void     APP_DataLog_SetMeasurementActive(uint8_t active);

/*
 * 分片执行延迟排空：每轮至多排空 1 个分片或执行 1 次 f_sync/f_close。
 * 返回 1 = 排空已完成，0 = 仍在进行中或无待处理任务。
 */
uint8_t  APP_DataLog_ServiceDeferredStop(void);

/* 查询是否有延迟排空待执行。 */
uint8_t  APP_DataLog_IsFlushPending(void);

/* 返回 SD 状态机阶段的可读标签，供 OLED 显示层使用。 */
const char *APP_DataLog_StateLabel(uint8_t state);

#endif
