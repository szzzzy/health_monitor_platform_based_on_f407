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
#include "app_rtos.h"
#include "main.h"
#include <string.h>

/* 环形缓冲区：2048 x 16B = 32KB */
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

/**
 ******************************************************************************
 * @brief  禁用中断并返回先前的 PRIMASK 状态
 * @param  无
 * @return 先前的 PRIMASK 值，用于 ring_exit_critical
 * @note  必须与 ring_exit_critical 配对使用。不支持嵌套调用。
 ******************************************************************************
 */
static uint32_t ring_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

/**
 ******************************************************************************
 * @brief  恢复中断至 ring_enter_critical 之前的状态
 * @param  primask 由 ring_enter_critical 保存的 PRIMASK 值
 * @return 无
 * @note  仅在临界区之前中断已启用时才重新启用中断。
 ******************************************************************************
 */
static void ring_exit_critical(uint32_t primask)
{
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }
}

/**
 ******************************************************************************
 * @brief  获取环形缓冲区中当前的样本数
 * @param  无
 * @return 缓冲样本数（线程安全）
 * @note  使用临界区保证原子读取
 ******************************************************************************
 */
static uint16_t ring_get_count(void)
{
  uint16_t count;
  uint32_t primask = ring_enter_critical();
  count = ring_count;
  ring_exit_critical(primask);
  return count;
}

/**
 ******************************************************************************
 * @brief  丢弃环形缓冲区中当前的所有样本
 * @param  无
 * @return 被丢弃的样本数
 * @note  在 flush 完成期间用于原子地丢弃剩余数据
 ******************************************************************************
 */
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
/**
 ******************************************************************************
 * @brief  从环形缓冲区弹出最多 count 个样本
 * @param  dst 用于复制样本的目标数组
 * @param  count 最多弹出的样本数
 * @return 实际复制的样本数
 * @note  线程安全；使用临界区。如果缓冲区数据不足，返回值可能小于 count。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  根据反压阈值检查环形缓冲区积压
 * @param  无
 * @return 如果积压可接受返回 1，如果暂停或过大返回 0
 * @note  当积压超过 50% 容量时设置 sd_paused。
 *        当环形缓冲区超过 25% 时阻止 SD 写入。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  启动或重新启动 SD 二进制日志会话
 * @param  enforce_backlog_guard 非零则在启动前强制检查积压
 * @return 成功返回 1，失败返回 0（backoff 待处理、积压已满或 SD 错误）
 * @note  启动会话后写入文件头。重复失败时进入 BACKOFF 状态。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  从环形缓冲区弹出一个 chunk 并写入 SD 卡
 * @param  budget_ms 允许的最大写入持续时间（毫秒）
 * @return 写入的样本数，失败时返回 0
 * @note  写入失败或超预算时进入 BACKOFF 状态。
 *        chunk 大小为 DATA_LOG_CHUNK_SAMPLES（32 个样本，512 字节）。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  初始化数据日志模块及其环形缓冲区
 * @param  无
 * @return 无
 * @note  清除所有状态变量并初始化底层 SD 文件层。
 *        必须在任何其他数据日志 API 之前调用一次。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  从实时上下文向环形缓冲区推送一个原始样本
 * @param  tick HAL 滴答计数器值
 * @param  red  红色 LED 原始读数
 * @param  ir   红外 LED 原始读数
 * @param  ecg  ECG 原始读数
 * @param  flags 样本状态标志
 * @return 无
 * @note  O(1) 且中断安全。环形缓冲区满时丢弃最旧样本。
 ******************************************************************************
 */
void APP_DataLog_PushSample(uint32_t tick, uint32_t red, uint32_t ir,
                             int16_t ecg, uint8_t flags)
{
  DataLogRawSample_t *dst;
  uint32_t primask = ring_enter_critical();

  /* 环形缓冲区满：覆盖最旧样本 */
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
/**
 ******************************************************************************
 * @brief  在时间预算内服务 SD 写入路径
 * @param  budget_ms 此服务调用的最大允许时间
 * @param  max_bytes 最大写入字节数（保留，未使用）
 * @return 写入的字节数，如果未写入则返回 0
 * @note  实现 SD 状态机（IDLE, TRY_START, ACTIVE, BACKOFF）。
 *        当 measurement_active 设置时不会发生物理 SD I/O。
 ******************************************************************************
 */
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

/**
 ******************************************************************************
 * @brief  通知测量会话已停止
 * @param  无
 * @return 无
 * @note  O(1)；仅设置待处理标志。实际的 flush 工作由
 *        APP_DataLog_ServiceDeferredStop 在安全窗口中完成。
 ******************************************************************************
 */
/*
 * 手指离开时：仅设置延迟 flush 标志 (O(1))。
 * 实际 drain + f_sync + f_close 由 APP_DataLog_ServiceDeferredStop() 在安全窗口分片执行。
 */
void APP_DataLog_OnMeasurementStop(void)
{
  flush_pending = 1U;
  flush_deferred_state = FLUSH_STATE_IDLE;
}

/**
 ******************************************************************************
 * @brief  将当前数据日志状态读入状态结构体
 * @param  status 指向要填充的 DataLogStatus_t 的指针
 * @return 无
 * @note  如果 status 为 NULL 则不执行任何操作。返回计数器和状态的快照。
 ******************************************************************************
 */
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
/**
 ******************************************************************************
 * @brief  检查 SD 写入会话当前是否活跃
 * @param  无
 * @return 活跃时返回 1，否则返回 0
 ******************************************************************************
 */
uint8_t APP_DataLog_IsActive(void)
{
  return (sd_state == SD_STATE_ACTIVE) ? 1U : 0U;
}

/**
 ******************************************************************************
 * @brief  设置控制 SD I/O 的测量活跃标志
 * @param  active 非零表示测量活跃，零允许 SD 写入
 * @return 无
 * @note  当活跃时，APP_DataLog_ServiceBudget 不执行物理 SD 写入。
 ******************************************************************************
 */
void APP_DataLog_SetMeasurementActive(uint8_t active)
{
  measurement_active = active;
}

/**
 ******************************************************************************
 * @brief  服务延迟 flush 状态机（drain, sync, close）
 * @param  无
 * @return flush 完全完成时返回 1，仍在进行中时返回 0
 * @note  实现多周期状态机：IDLE -> DRAINING -> SYNC -> DONE。
 *        每次调用仅执行一次 I/O 操作以避免长时间阻塞。
 ******************************************************************************
 */
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
    /* StopSession 内部: f_sync → f_close → f_mount(unmount) → Deinit
     * 每一步都可能阻塞 (卡慢/坏卡时可达 200ms+)。
     * 标记阶段码以便崩溃时定位在此。 */
    {
      AppState_t *s = app_rtos_get_state();
      if (s != NULL) { s->sd_task_phase = PHASE_SD_FLUSH; }
      APP_SdFile_StopSession();
      if (s != NULL) { s->sd_task_phase = PHASE_SD_IDLE; }
    }
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

/**
 ******************************************************************************
 * @brief  检查延迟 flush 是否待处理
 * @param  无
 * @return 如果 flush 待处理则返回 1，否则返回 0
 ******************************************************************************
 */
uint8_t APP_DataLog_IsFlushPending(void)
{
  return flush_pending;
}
