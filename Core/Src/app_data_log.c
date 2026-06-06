/**
  ******************************************************************************
  * @file    app_data_log.c
  * @brief   SD 二进制日志引擎 — 环形缓冲 + 分片写入
  *
  * 实时路径：PushSample() 仅 16 字节 memcpy
  * 后台路径：ServiceBudget() 格式化 chunk → APP_SdFile_WriteBytes()
  * 停止路径：OnMeasurementStop() → 分片排空 → f_sync
  *
  * 文件格式：
  *   [DataLogFileHeader_t 32B] [DataLogRawSample_t × N] ...
  * 文件命名：LOG_XXXXX.BIN（递增序号）
  *
  * 反压门控：ring_count > 25% → 不启动/不恢复 SD 会话
  *          ring_count > 50% → 暂停写入 + BACKOFF
  *          保证 MAX30102 FIFO 优先于 SD 日志
  ******************************************************************************
  */

#include "app_data_log.h"
#include "app_sd_file.h"
#include "main.h"
#include <string.h>

/* 环形缓冲：2048 × 16B = 32KB */
static DataLogRawSample_t ring[DATA_LOG_RING_SAMPLES];
static uint16_t ring_head  = 0U;
static uint16_t ring_tail  = 0U;
static uint16_t ring_count = 0U;

/* 状态机 */
typedef enum {
  SD_STATE_IDLE = 0,
  SD_STATE_TRY_START,
  SD_STATE_ACTIVE,
  SD_STATE_BACKOFF
} SdState_t;

static SdState_t  sd_state     = SD_STATE_IDLE;
static uint32_t   backoff_until = 0U;
static uint32_t   total_dropped = 0U;
static uint32_t   total_written = 0U;
static uint8_t    write_seq     = 0U;
static uint8_t    file_seq      = 0U;
static uint8_t    sd_paused     = 0U;
static uint8_t    sd_last_error = 0U;
static uint32_t   last_write_ms = 0U;
static uint32_t   last_backlog  = 0U;
static uint16_t   session_written = 0U;

#define BACKOFF_MS       60000U
#define WRITE_CHUNK_BYTES ((uint32_t)DATA_LOG_CHUNK_SAMPLES * sizeof(DataLogRawSample_t))

/* 反压阈值：ring 超过此比例则拒绝启动/恢复 SD 写入 */
#define BACKLOG_START_GUARD  (DATA_LOG_RING_SAMPLES / 4U)   /* 25% = 512 样本 */
#define BACKLOG_PAUSE_GUARD  (DATA_LOG_RING_SAMPLES / 2U)   /* 50% = 1024 样本 */
#define STOP_DRAIN_MAX_CHUNKS 4U  /* 手指离开时最多排空 4 chunk (2KB)，保证重捕获体验 */

/* 格式化缓冲区：文件头(32B) + chunk(512B) = 544B，无需更大 */
static uint8_t fmt_buf[sizeof(DataLogFileHeader_t) + WRITE_CHUNK_BYTES];

/* ---- 内部辅助 ---- */

static uint16_t ring_pop(DataLogRawSample_t *dst, uint16_t count)
{
  uint16_t copied = 0U;
  while ((copied < count) && (ring_count > 0U))
  {
    (void)memcpy(&dst[copied], &ring[ring_tail], sizeof(DataLogRawSample_t));
    ring_tail = (ring_tail + 1U) % DATA_LOG_RING_SAMPLES;
    ring_count--;
    copied++;
  }
  return copied;
}

/* 检查反压：ring 积压过多 → 不入队，让 FIFO 优先 */
static uint8_t backlog_ok_for_sd(void)
{
  if (ring_count >= BACKLOG_PAUSE_GUARD)
  {
    sd_paused = 1U;
    return 0U;
  }
  if (ring_count >= BACKLOG_START_GUARD)
  {
    return 0U;
  }
  return 1U;
}

/* 启动/重新启动 SD 二进制日志会话。受 backlog_ok_for_sd 门控。 */
static uint8_t sd_try_start(void)
{
  AppSdFileStatus_t ret;
  DataLogFileHeader_t header;

  if (!backlog_ok_for_sd()) return 0U;

  ret = APP_SdFile_StartSession();
  if (ret != APP_SD_FILE_OK)
  {
    sd_last_error = (uint8_t)ret;
    sd_state = SD_STATE_BACKOFF;
    backoff_until = HAL_GetTick() + BACKOFF_MS;
    return 0U;
  }

  header.magic[0] = 'B'; header.magic[1] = 'M';
  header.magic[2] = 'L'; header.magic[3] = 'G';
  header.version  = 1U;
  header.sample_rate_hz = 100U;
  header.start_tick = HAL_GetTick();
  (void)memset(header.reserved, 0, sizeof(header.reserved));

  ret = APP_SdFile_WriteBytes(&header, (uint16_t)sizeof(header));
  if (ret != APP_SD_FILE_OK)
  {
    sd_last_error = (uint8_t)ret;
    sd_paused = 1U;
    sd_state = SD_STATE_BACKOFF;
    backoff_until = HAL_GetTick() + BACKOFF_MS;
    return 0U;
  }

  sd_state = SD_STATE_ACTIVE;
  sd_paused = 0U;
  session_written = 0U;
  return 1U;
}

/*
 * 从环形缓冲 pop 一个 chunk，直接 f_write 写入 SD。
 * 每次 512B @ 12MHz SDIO ≈ 0.3ms，远在预算内。
 * 返回写入的样本数。
 */
static uint16_t sd_write_one_chunk(uint32_t budget_ms)
{
  uint32_t t0;
  AppSdFileStatus_t ret;
  uint16_t popped;

  if (ring_count < DATA_LOG_CHUNK_SAMPLES) return 0U;

  t0 = HAL_GetTick();
  popped = ring_pop((DataLogRawSample_t *)fmt_buf, DATA_LOG_CHUNK_SAMPLES);
  if (popped == 0U) return 0U;

  ret = APP_SdFile_WriteBytes(fmt_buf, WRITE_CHUNK_BYTES);
  last_write_ms = HAL_GetTick() - t0;

  /*
   * 写入超预算或失败 → 暂停 SD，进入 backoff。
   * WriteBytes 内部失败时已做 close_session_after_error 级联清理。
   */
  if ((ret != APP_SD_FILE_OK) || (last_write_ms > budget_ms))
  {
    if (ret != APP_SD_FILE_OK) { sd_last_error = (uint8_t)ret; }
    sd_paused = 1U;
    if (last_write_ms > budget_ms || ret != APP_SD_FILE_OK)
    {
      total_dropped += popped;
      popped = 0U;
    }
    sd_state = SD_STATE_BACKOFF;
    backoff_until = HAL_GetTick() + BACKOFF_MS;
    return 0U;
  }

  sd_paused = 0U;
  session_written += popped;
  total_written += popped;
  return popped;
}

/* ---- 公共 API ---- */

void APP_DataLog_Init(void)
{
  APP_SdFile_Init();
  (void)memset(ring, 0, sizeof(ring));
  ring_head   = 0U;
  ring_tail   = 0U;
  ring_count  = 0U;
  total_dropped = 0U;
  total_written = 0U;
  write_seq     = 0U;
  file_seq      = 0U;
  sd_state     = SD_STATE_IDLE;
  backoff_until = 0U;
  sd_paused     = 0U;
  sd_last_error = 0U;
  last_write_ms = 0U;
  last_backlog  = 0U;
  session_written = 0U;
}

void APP_DataLog_PushSample(uint32_t tick, uint32_t red, uint32_t ir,
                             int16_t ecg, uint8_t flags)
{
  DataLogRawSample_t *dst;

  /* 环形缓冲满 → 覆盖最旧记录 */
  if (ring_count >= DATA_LOG_RING_SAMPLES)
  {
    ring_tail = (ring_tail + 1U) % DATA_LOG_RING_SAMPLES;
    ring_count--;
    total_dropped++;
  }

  dst = &ring[ring_head];
  dst->tick  = tick;
  dst->red   = red;
  dst->ir    = ir;
  dst->ecg   = ecg;
  dst->flags = flags;
  dst->seq   = write_seq++;

  ring_head = (ring_head + 1U) % DATA_LOG_RING_SAMPLES;
  ring_count++;
}

uint16_t APP_DataLog_ServiceBudget(uint32_t budget_ms, uint16_t max_bytes)
{
  uint16_t chunk_bytes;
  uint16_t written;

  (void)max_bytes;
  last_backlog = ring_count;

  switch (sd_state)
  {
  case SD_STATE_IDLE:
    if ((ring_count >= DATA_LOG_CHUNK_SAMPLES) && backlog_ok_for_sd())
    {
      sd_state = SD_STATE_TRY_START;
      /* fall through */
    }
    else { return 0U; }
    /* fall through */

  case SD_STATE_TRY_START:
    if (!sd_try_start()) return 0U;
    /* fall through */

  case SD_STATE_ACTIVE:
    /* 反压门控：backlog 过高时暂停写入（不进 BACKOFF，等 backlog 下降后自动恢复） */
    if (!backlog_ok_for_sd())
    {
      return 0U;
    }
    chunk_bytes = (uint16_t)WRITE_CHUNK_BYTES;
    written = sd_write_one_chunk(budget_ms);
    if (written == 0U)
    {
      if (sd_state == SD_STATE_BACKOFF) return 0U;
      if (!APP_SdFile_IsReady()) { sd_state = SD_STATE_IDLE; }
      return 0U;
    }
    return chunk_bytes;

  case SD_STATE_BACKOFF:
    if (HAL_GetTick() >= backoff_until)
    {
      sd_state = SD_STATE_IDLE;
      sd_paused = 0U;
    }
    return 0U;

  default:
    sd_state = SD_STATE_IDLE;
    return 0U;
  }
}

/*
 * 手指离开时：分片排空至多 STOP_DRAIN_MAX_CHUNKS × 512B = 2KB，
 * 剩余样本直接丢弃（2KB ≈ 0.6ms @ 12MHz SDIO，不影响重捕获）。
 * 最后 APP_SdFile_StopSession 做 flush+f_sync+关闭文件。
 */
void APP_DataLog_OnMeasurementStop(void)
{
  uint8_t drain_chunks = 0U;

  while ((ring_count >= DATA_LOG_CHUNK_SAMPLES) && (drain_chunks < STOP_DRAIN_MAX_CHUNKS))
  {
    if (sd_state == SD_STATE_IDLE || sd_state == SD_STATE_BACKOFF)
    {
      if (!sd_try_start()) break;
    }
    if (sd_write_one_chunk(10U) == 0U) break;
    drain_chunks++;
  }

  /* 排空最后不足一个 chunk 的样本（至多 31 样本 = 496B） */
  if ((ring_count > 0U) && (sd_state == SD_STATE_ACTIVE) && (drain_chunks < STOP_DRAIN_MAX_CHUNKS))
  {
    uint16_t remaining = ring_pop((DataLogRawSample_t *)fmt_buf, ring_count);
    if (remaining > 0U)
    {
      (void)APP_SdFile_WriteBytes(fmt_buf, (uint16_t)(remaining * sizeof(DataLogRawSample_t)));
      total_written += remaining;
      session_written += remaining;
    }
  }

  /* 丢弃剩余未排空的样本（drain_chunks 已达上限或 SD 不可用） */
  if (ring_count > 0U)
  {
    total_dropped += ring_count;
    ring_tail = ring_head;
    ring_count = 0U;
  }

  /* 停止会话 → f_sync → 关闭文件 */
  APP_SdFile_StopSession();
  sd_state = SD_STATE_IDLE;
  sd_paused = 0U;
  file_seq = (file_seq + 1U) % 100U;
  session_written = 0U;
}

void APP_DataLog_GetStatus(DataLogStatus_t *status)
{
  if (status == NULL) return;
  status->buffered      = ring_count;
  status->dropped       = (uint16_t)(total_dropped > 0xFFFFU ? 0xFFFFU : total_dropped);
  status->written       = session_written;
  status->paused        = sd_paused;
  status->sd_error      = sd_last_error;
  status->state         = (uint8_t)sd_state;
  status->last_write_ms = last_write_ms;
  status->last_backlog  = last_backlog;
}

uint8_t APP_DataLog_IsActive(void)
{
  return (sd_state == SD_STATE_ACTIVE) ? 1U : 0U;
}
