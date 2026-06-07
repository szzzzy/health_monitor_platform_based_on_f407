/**
  ******************************************************************************
  * @file    app_data_log.h
  * @brief   SD 数据日志 — 二进制环形缓冲 + 后台分片写入
  *
  * 架构：
  *   实时路径 (100Hz):   PushSample()  → 16 字节 memcpy 入队，O(1)
  *   后台路径 (主循环):   ServiceBudget(budget_ms, max_bytes) → 格式化 + f_write
  *   停止路径 (手指离开): OnMeasurementStop() → flush + f_sync
  *
  * 关键约束：
  *   - PushSample 不做任何 snprintf / f_write / f_sync
  *   - ServiceBudget 每轮至多一次 f_write，至多 budget_ms 时间
  *   - 环形缓冲满时覆盖最旧记录（静默丢日志，不破坏采样）
  ******************************************************************************
  */

#ifndef __APP_DATA_LOG_H__
#define __APP_DATA_LOG_H__

#include <stdint.h>

#define DATA_LOG_RING_SAMPLES   2048U   /* ~20 秒 @ 100Hz, 32KB */
#define DATA_LOG_CHUNK_SAMPLES    32U   /* 512 字节 = 1 SD 扇区 */

/* 紧凑二进制样本记录 (16 字节)，无动态内存 */
typedef struct __attribute__((packed)) {
  uint32_t tick;
  uint32_t red;
  uint32_t ir;
  int16_t  ecg;
  uint8_t  flags;    /* bit0:finger_present, bit1:contact_settle, bit2:fifo_ovf */
  uint8_t  seq;
} DataLogRawSample_t;

/* 二进制文件头 (32 字节)，每个 session 文件开头写入一次 */
typedef struct __attribute__((packed)) {
  uint8_t  magic[4];      /* "BMLG" */
  uint16_t version;       /* 1 */
  uint16_t sample_rate_hz;
  uint32_t start_tick;
  uint8_t  reserved[20];
} DataLogFileHeader_t;

/* SD 日志运行时状态，供 OLED / UART 查询 */
typedef struct {
  uint16_t buffered;       /* 环形缓冲中待写入样本数 */
  uint16_t dropped;        /* 累计丢弃样本数（环形缓冲溢出） */
  uint16_t written;        /* 本次 session 已持久化样本数 */
  uint8_t  paused;         /* 1 = SD 写入已暂停（慢/满/错误） */
  uint8_t  sd_error;       /* 最后一次 SD 错误码 (AppSdFileStatus_t) */
  uint8_t  state;          /* 内部状态机阶段 (SdLogState_t) */
  uint32_t last_write_ms;  /* 最近一次 SD 写入耗时 (ms) */
  uint32_t last_backlog;   /* 最近一次观测到的堆积量 */
} DataLogStatus_t;

/* ---- API ---- */

void     APP_DataLog_Init(void);

/*
 * 实时路径：将原始样本推入环形缓冲。仅做 16 字节 memcpy，零格式化。
 * 调用方 (app_measurement_read_sensor_sample) 每成功读一个 FIFO 样本调用一次。
 */
void     APP_DataLog_PushSample(uint32_t tick, uint32_t red, uint32_t ir,
                                int16_t ecg, uint8_t flags);

/*
 * 后台路径：按预算写 SD。每轮主循环调用一次。
 *   budget_ms: 本轮允许的最大耗时 (ms)，超出后立即返回
 *   max_bytes: 本轮允许写入的最大字节数
 * 返回实际写入的字节数。
 */
uint16_t APP_DataLog_ServiceBudget(uint32_t budget_ms, uint16_t max_bytes);

/* 手指离开 / 测量停止时调用，flush 剩余缓冲并 f_sync。 */
void     APP_DataLog_OnMeasurementStop(void);

/* 查询运行时状态。 */
void     APP_DataLog_GetStatus(DataLogStatus_t *status);

/* 遗留兼容：供 app_state 快速查询是否活跃。 */
uint8_t  APP_DataLog_IsActive(void);

/*
 * 测量活跃门控：active=1 时 ServiceBudget 直接返回 0，
 * 禁止一切物理 SD I/O（包括 f_write）。由 main.c 每轮更新。
 */
void     APP_DataLog_SetMeasurementActive(uint8_t active);

/*
 * 分片执行延迟 flush：每轮至多 1 chunk drain 或 1 次 f_sync/f_close。
 * 返回 1 = flush 已完成，0 = 仍在进行中或无 pending。
 */
uint8_t  APP_DataLog_ServiceDeferredStop(void);

/* 查询是否有延迟 flush 待执行。 */
uint8_t  APP_DataLog_IsFlushPending(void);

#endif
