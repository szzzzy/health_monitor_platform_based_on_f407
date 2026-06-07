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

/* Ring buffer: 2048 x 16B = 32KB. */
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

/* 测量活跃门控 */
static uint8_t    measurement_active = 0U;

/* 延迟 flush 状态机 */
static uint8_t    flush_pending = 0U;
typedef enum {
  FLUSH_STATE_IDLE = 0,
  FLUSH_STATE_DRAINING,
  FLUSH_STATE_SYNC,
  FLUSH_STATE_DONE
} FlushDeferredState_t;
static FlushDeferredState_t flush_deferred_state = FLUSH_STATE_IDLE;
#define FLUSH_DRAIN_MAX_CHUNKS 8U

#define BACKOFF_MS       60000U
#define WRITE_CHUNK_BYTES ((uint32_t)DATA_LOG_CHUNK_SAMPLES * sizeof(DataLogRawSample_t))

/* 反压阈值：ring 超过此比例则拒绝启动/恢复 SD 写入 */
#define BACKLOG_START_GUARD  (DATA_LOG_RING_SAMPLES / 4U)   /* 25% = 512 样本 */
#define BACKLOG_PAUSE_GUARD  (DATA_LOG_RING_SAMPLES / 2U)   /* 50% = 1024 样本 */
/* 格式化缓冲区：文件头(32B) + chunk(512B) = 544B，无需更大 */
static uint8_t fmt_buf[sizeof(DataLogFileHeader_t) + WRITE_CHUNK_BYTES];

/* ---- 内部辅助 ---- */

static uint32_t ring_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void ring_exit_critical(uint32_t primask)
{
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }
}

static uint16_t ring_get_count(void)
{
  uint16_t count;
  uint32_t primask = ring_enter_critical();
  count = ring_count;
  ring_exit_critical(primask);
  return count;
}

static uint16_t ring_clear_pending(void)
{
  uint16_t dropped;
  uint32_t primask = ring_enter_critical();
  dropped = ring_count;
  ring_tail = ring_head;
  ring_count = 0U;
  ring_exit_critical(primask);
  return dropped;
}
static uint16_t ring_pop(DataLogRawSample_t *dst, uint16_t count)
{
  uint16_t copied = 0U;
  uint32_t primask = ring_enter_critical();

  while ((copied < count) && (ring_count > 0U))
  {
    (void)memcpy(&dst[copied], &ring[ring_tail], sizeof(DataLogRawSample_t));
    ring_tail = (ring_tail + 1U) % DATA_LOG_RING_SAMPLES;
    ring_count--;
    copied++;
  }

  ring_exit_critical(primask);
  return copied;
}

/* 检查反压：ring 积压过多 → 不入队，让 FIFO 优先 */
static uint8_t backlog_ok_for_sd(void)
{
  uint16_t count = ring_get_count();

  if (count >= BACKLOG_PAUSE_GUARD)
  {
    sd_paused = 1U;
    return 0U;
  }
  if (count >= BACKLOG_START_GUARD)
  {
    return 0U;
  }
  return 1U;
}

/* 启动/重新启动 SD 二进制日志会话。受 backlog_ok_for_sd 门控。 */
static uint8_t sd_try_start(uint8_t enforce_backlog_guard)
{
  AppSdFileStatus_t ret;
  DataLogFileHeader_t header;

  if (sd_state == SD_STATE_BACKOFF)
  {
    if (HAL_GetTick() < backoff_until)
    {
      return 0U;
    }
    sd_state = SD_STATE_IDLE;
    sd_paused = 0U;
  }

  if ((enforce_backlog_guard != 0U) && (!backlog_ok_for_sd())) return 0U;

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

  if (ring_get_count() < DATA_LOG_CHUNK_SAMPLES) return 0U;

  t0 = HAL_GetTick();
  popped = ring_pop((DataLogRawSample_t *)fmt_buf, DATA_LOG_CHUNK_SAMPLES);
  if (popped == 0U) return 0U;

  ret = APP_SdFile_WriteBytes(fmt_buf, WRITE_CHUNK_BYTES);
  last_write_ms = HAL_GetTick() - t0;

  /*
   * 写入超预算或失败 → 暂停 SD，进入 backoff。
   * WriteBytes 内部失败时已做 close_session_after_error 级联清理。
   */
  if (ret != APP_SD_FILE_OK)
  {
    sd_last_error = (uint8_t)ret;
    sd_paused = 1U;
    total_dropped += popped;
    sd_state = SD_STATE_BACKOFF;
    backoff_until = HAL_GetTick() + BACKOFF_MS;
    return 0U;
  }

  session_written += popped;
  total_written += popped;

  if (last_write_ms > budget_ms)
  {
    sd_paused = 1U;
    sd_state = SD_STATE_BACKOFF;
    backoff_until = HAL_GetTick() + BACKOFF_MS;
    return 0U;
  }

  sd_paused = 0U;
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
  measurement_active = 0U;
  flush_pending = 0U;
  flush_deferred_state = FLUSH_STATE_IDLE;
}

void APP_DataLog_PushSample(uint32_t tick, uint32_t red, uint32_t ir,
                             int16_t ecg, uint8_t flags)
{
  DataLogRawSample_t *dst;
  uint32_t primask = ring_enter_critical();

  /* Ring full: overwrite the oldest sample. */
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
  ring_exit_critical(primask);
}
uint16_t APP_DataLog_ServiceBudget(uint32_t budget_ms, uint16_t max_bytes)
{
  uint16_t chunk_bytes;
  uint16_t written;

  (void)max_bytes;
  last_backlog = ring_get_count();

  /* 测量活跃时禁止一切物理 SD I/O，包括 f_write */
  if (measurement_active != 0U) { return 0U; }

  switch (sd_state)
  {
  case SD_STATE_IDLE:
    if ((ring_get_count() >= DATA_LOG_CHUNK_SAMPLES) && backlog_ok_for_sd())
    {
      sd_state = SD_STATE_TRY_START;
      /* fall through */
    }
    else { return 0U; }
    /* fall through */

  case SD_STATE_TRY_START:
    if (!sd_try_start(1U)) return 0U;
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
 * 手指离开时：仅设置延迟 flush 标志 (O(1))。
 * 实际 drain + f_sync + f_close 由 APP_DataLog_ServiceDeferredStop() 在安全窗口分片执行。
 */
void APP_DataLog_OnMeasurementStop(void)
{
  flush_pending = 1U;
  flush_deferred_state = FLUSH_STATE_IDLE;
}

void APP_DataLog_GetStatus(DataLogStatus_t *status)
{
  uint16_t buffered;

  if (status == NULL) return;

  buffered = ring_get_count();
  status->buffered      = buffered;
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

void APP_DataLog_SetMeasurementActive(uint8_t active)
{
  measurement_active = active;
}

/*
 * 分片延迟 flush 状态机。
 * 每轮只做一个动作（1 chunk f_write 或 1 次 f_sync+f_close），
 * 避免在安全窗口一次性阻塞过久。
 *
 * 状态转移: IDLE → DRAINING → SYNC → DONE → IDLE
 * 仅在 finger_present==0 的安全窗口由 main.c 调用。
 */
uint8_t APP_DataLog_ServiceDeferredStop(void)
{
  if (flush_pending == 0U) { return 0U; }

  switch (flush_deferred_state)
  {
  case FLUSH_STATE_IDLE:
    /* 检查是否有 session 可用；如无，丢弃 ring 后直接完成 */
    if (sd_state != SD_STATE_ACTIVE)
    {
      /* 尝试启动 session 以便写剩余数据 */
      if (!sd_try_start(0U))
      {
        /* 启动失败时保留 ring，等待 BACKOFF 到期后继续尝试。 */
        return 0U;
      }
    }
    flush_deferred_state = FLUSH_STATE_DRAINING;
    /* fall through */

  case FLUSH_STATE_DRAINING:
    if (sd_state != SD_STATE_ACTIVE)
    {
      flush_deferred_state = FLUSH_STATE_IDLE;
      return 0U;
    }

    if (ring_get_count() >= DATA_LOG_CHUNK_SAMPLES)
    {
      (void)sd_write_one_chunk(10U);
      return 0U; /* 还有数据，下轮继续 drain */
    }
    /* ring 不足一个 chunk → 排空剩余零散样本 */
    if (ring_get_count() > 0U)
    {
      uint16_t remaining = ring_pop((DataLogRawSample_t *)fmt_buf, ring_get_count());
      if (remaining > 0U)
      {
        AppSdFileStatus_t ret = APP_SdFile_WriteBytes(fmt_buf,
            (uint16_t)(remaining * sizeof(DataLogRawSample_t)));
        if (ret == APP_SD_FILE_OK)
        {
          total_written += remaining;
          session_written += remaining;
        }
        else
        {
          sd_last_error = (uint8_t)ret;
          total_dropped += remaining;
          sd_state = SD_STATE_BACKOFF;
          sd_paused = 1U;
          backoff_until = HAL_GetTick() + BACKOFF_MS;
          flush_deferred_state = FLUSH_STATE_IDLE;
          return 0U;
        }
      }
    }
    flush_deferred_state = FLUSH_STATE_SYNC;
    /* fall through */

  case FLUSH_STATE_SYNC:
    APP_SdFile_StopSession();
    {
      uint16_t leftover = ring_clear_pending();
      total_dropped += leftover;
    }
    sd_state = SD_STATE_IDLE;
    sd_paused = 0U;
    flush_deferred_state = FLUSH_STATE_DONE;
    /* fall through */

  case FLUSH_STATE_DONE:
    flush_pending = 0U;
    flush_deferred_state = FLUSH_STATE_IDLE;
    file_seq = (file_seq + 1U) % 100U;
    session_written = 0U;
    return 1U;

  default:
    flush_deferred_state = FLUSH_STATE_IDLE;
    return 0U;
  }
}

uint8_t APP_DataLog_IsFlushPending(void)
{
  return flush_pending;
}
