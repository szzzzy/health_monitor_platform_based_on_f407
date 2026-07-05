#include "app_protocol.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_data_log.h"
#include "eeprom_cmd.h"
#include "usart.h"

/* 串口上报与接收缓冲长度；1536 覆盖追加的 SQI/PTT 尾部诊断字段。 */
#define JSON_PAYLOAD_SIZE     1536U
#define UART_RX_LINE_SIZE     64U
#define STM32_UART_TIMEOUT_MS 100U

static char json_payload[JSON_PAYLOAD_SIZE];
static char uart_rx_line[UART_RX_LINE_SIZE];
static uint16_t uart_rx_line_len;

static uint16_t build_sensor_packet(const AppState_t *app, char *buffer, size_t buffer_size);
static bool send_uart_line(AppState_t *app, const char *payload, uint16_t payload_len);
static const char *app_skip_spaces(const char *text);
static uint8_t app_text_starts_with_keyword(const char *text, const char *keyword);
static const char *app_get_time_command_payload(const char *line);
static uint8_t app_parse_fixed_uint(const char *text, uint8_t digits, uint16_t *value);
static uint8_t app_parse_datetime_text(const char *text, APP_RTC_DateTime_t *date_time);
static void app_send_rtc_set_response(AppState_t *app, uint8_t success, const char *reason);
static bool app_process_uart_line(AppState_t *app, const char *line);

/**
 * @brief  初始化 UART 命令行协议模块。
 * @note   将内部 UART RX 行缓冲长度清零。
 *         必须在系统启动且任何 UART 通信开始前调用一次。
 */
/* 初始化串口接收行缓冲。 */
void app_protocol_init(void)
{
  uart_rx_line_len = 0U;
}

/**
 * @brief  将当前 RTC 日期/时间快照读入应用状态。
 * @param  app 应用状态指针（更新 rtc_datetime 字段）。
 * @note   根据 RTC 状态更新 rtc_read_ok 和 rtc_time_valid 标志。
 *         读取失败时清零 rtc_datetime 结构体。
 *         显示和串口上报都会使用该快照，保证单次刷新周期内时间戳一致。
 */
/* 读取 RTC 当前快照，供显示与上报统一使用。 */
void app_protocol_update_rtc_snapshot(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if (APP_RTC_GetDateTime(&app->rtc_datetime) == HAL_OK)
  {
    app->rtc_read_ok = 1U;
  }
  else
  {
    (void)memset(&app->rtc_datetime, 0, sizeof(app->rtc_datetime));
    app->rtc_read_ok = 0U;
  }

  app->rtc_time_valid = APP_RTC_IsTimeValid();
}

/**
 * @brief  轮询 USART2 DMA 环形缓冲，接收命令行。
 * @param  app 应用状态指针（收到有效命令时更新）。
 * @note   处理 DMA 空闲行中断后累计的接收字节。
 *         行以 '\n' 分隔，'\r' 字符会被跳过。
 *         识别命令：TIME（查询）和 SETTIME（设置 RTC）。
 *         收到有效 SETTIME 命令时更新 RTC 并发送响应。
 *         若单行超过 UART_RX_LINE_SIZE，则重置缓冲。
 */
/* 轮询 USART2 DMA 缓冲区，按”单行命令”协议接收 TIME / SETTIME。 */
void app_protocol_poll_uart_commands(AppState_t *app)
{
  uint16_t current_pos;
  uint16_t buf_size = UART_DMA_RX_BUF_SIZE;
  uint8_t rx_byte;

  if (app == NULL)
  {
    return;
  }

  if (usart_get_dma_idle_flag() == 0U)
  {
    return;
  }

  usart_clear_dma_idle_flag();

  /*
   * DMA_CNDTR 递减计数 → 当前写入位置 = buf_size - CNDTR。
   * 从 last_pos 读到 current_pos，处理所有已接收字节。
   */
  current_pos = (uint16_t)(buf_size - (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));

  while (usart_get_dma_last_pos() != current_pos)
  {
    rx_byte = usart_get_dma_rx_buf()[usart_get_dma_last_pos()];
    usart_set_dma_last_pos((uint16_t)((usart_get_dma_last_pos() + 1U) % buf_size));

    if (rx_byte == '\r')
    {
      continue;
    }

    if (rx_byte == '\n')
    {
      if (uart_rx_line_len > 0U)
      {
        uart_rx_line[uart_rx_line_len] = '\0';
        app->uart_rx_message_valid = app_process_uart_line(app, uart_rx_line);
        uart_rx_line_len = 0U;
      }

      continue;
    }

    if (uart_rx_line_len < (UART_RX_LINE_SIZE - 1U))
    {
      uart_rx_line[uart_rx_line_len++] = (char)rx_byte;
    }
    else
    {
      app->uart_rx_message_valid = false;
      uart_rx_line_len = 0U;
    }
  }
}

/**
 * @brief  组装当前传感器测量上报，并通过 UART 发送。
 * @param  app 应用状态指针（读取全部测量字段）。
 * @note   组包前先更新 RTC 快照，再通过 USART2 发送格式化文本行。
 *         报文由 build_sensor_packet() 构建，并由 send_uart_line() 发送。
 */
/* 组装当前测量报文并通过串口发送。 */
void app_protocol_send_sensor_report(AppState_t *app)
{
  uint16_t payload_len;

  if (app == NULL)
  {
    return;
  }

  app_protocol_update_rtc_snapshot(app);
  payload_len = build_sensor_packet(app, json_payload, sizeof(json_payload));
  (void)send_uart_line(app, json_payload, payload_len);
}

/**
 * @brief  构建逗号分隔的传感器测量上报报文。
 * @param  app         应用状态结构体指针。
 * @param  buffer      格式化报文字符串输出缓冲。
 * @param  buffer_size 输出缓冲大小。
 * @return 格式化报文字符串长度（字节），出错时返回 0。
 * @note   上报格式向后兼容：旧版上位机可解析前缀字段，
 *         新版上位机使用追加的诊断字段（SQ、PI、R/BAL、
 *         传感器/SD/RTC 诊断）判断可信度。
 *         字段：M,rtc_valid,yyyymmdd,hhmmss,red,ir,...,crash_flag,...,stack_hwm
 */
/*
 * 字段顺序兼容旧上位机，不可改变。
 * 旧上位机可忽略尾部字段继续解析前缀；新上位机应优先使用尾部 SQ/PI/R/BAL
 * 和传感器/SD/RTC 诊断字段来解释指标可信度。
 *
 * M,rtc_valid,yyyymmdd,hhmmss,red,ir,baseline_ir,finger,bpm_valid,bpm,spo2_valid,spo2,
 *   rr_valid,rr,ibi_valid,ibi,hrv_valid,mean_ibi,sdnn,rmssd,
 *   motion_artifact,motion_score,sd1,sd2,sd1_sd2_x100,rhythm_irregular,
 *   hrv_freq_valid,lf_power_x100,hf_power_x100,lf_hf_x100,
 *   signal_quality,raw_signal_present,signal_ir_pi_x1000,signal_red_pi_x1000,
 *   signal_ir_ac_rms,signal_red_ac_rms,spo2_ratio_valid,spo2_ratio_x1000,spo2_balance_status,
 *   baseline_range_ir,adaptive_finger_on_delta,adaptive_finger_off_delta,
 *   ir_signal_delta,ir_signal_span,red_signal_span,finger_on_confirm_count,finger_off_confirm_count,
 *   sensor_last_read_status,sensor_error_streak,
 *   sensor_fifo_write_ptr,sensor_fifo_read_ptr,sensor_fifo_overflow_count,sensor_fifo_available_samples,
 *   sensor_read_ok_count,sensor_read_busy_count,sensor_read_error_count,sensor_recover_count,
 *   sensor_last_sample_tick,sensor_sample_change_count,sensor_sample_same_count,sensor_last_i2c_error,
 *   rtc_read_ok,uart_rx_message_valid,uart_tx_message_valid,
 *   sd_log_active,sd_state,sd_error,sd_total_written,
 *   display_refresh_count,display_last_refresh_tick,debug_mode,current_page,
 *   ecg_valid,ecg_hr,ecg_rr_ms,ecg_lead_off,ecg_r_peak_ms,ecg_filtered,ptt_valid,ptt_ms,
 *   ecg_sample_count,ecg_adc_sat_count,ecg_dma_overflow_count,ecg_lead_off_count,
 *   ecg_no_r_peak_timeout_count,crash_flag,crash_source,crash_task,crash_phase,
 *   crash_tick,reboot_count,reset_flags,max_task_phase,ui_task_phase,sd_task_phase,
 *   wdt_task_phase,max_task_stack_hwm,ui_task_stack_hwm,sd_task_stack_hwm,
 *   wdt_task_stack_hwm,max_task_heartbeat,ui_task_heartbeat,
 *   ecg_signal_quality,ecg_invalid_reason,ecg_raw_span,ecg_filtered_span,
 *   ecg_noise_level,ecg_qrs_threshold,ecg_peak_snr_x100,ecg_dma_available_high_watermark,
 *   ppg_sqi_score,ppg_sqi_flags,ppg_sqi_low_perfusion_count,ppg_sqi_motion_count,
 *   ppg_sqi_balance_count,ppg_sqi_transition_count,ppg_sqi_ibi_reject_count,
 *   ppg_sqi_amp_reject_count,ppg_sqi_suppressed_count,
 *   ptt_reject_ecg_count,ptt_reject_ppg_count,ptt_reject_range_count,ptt_reject_jump_count
 */

/* ---- 小型 CSV 构建器：避免超长 snprintf，逐字段追加 ---- */
typedef struct {
  char    *buf;
  size_t   size;
  size_t   len;
  uint8_t  overflow;
} AppCsvBuilder_t;

static void app_csv_init(AppCsvBuilder_t *b, char *buf, size_t size)
{
  if (b == NULL) { return; }
  b->buf = buf;
  b->size = size;
  b->len = 0U;
  b->overflow = 0U;
  if ((buf != NULL) && (size > 0U)) { buf[0] = '\0'; }
}

static void app_csv_appendf(AppCsvBuilder_t *b, const char *fmt, ...)
{
  va_list args;
  int written;
  size_t remaining;

  if ((b == NULL) || (b->overflow != 0U)) { return; }
  if ((b->buf == NULL) || (b->size == 0U)) { b->overflow = 1U; return; }

  remaining = b->size - b->len;
  if (remaining <= 1U) { b->overflow = 1U; return; }

  va_start(args, fmt);
  written = vsnprintf(b->buf + b->len, remaining, fmt, args);
  va_end(args);

  if (written < 0) { b->overflow = 1U; return; }

  b->len += (size_t)written;
  if (b->len >= b->size) {
    b->len = b->size - 1U;
    b->buf[b->len] = '\0';
    b->overflow = 1U;
  }
}
static uint16_t build_sensor_packet(const AppState_t *app, char *buffer, size_t buffer_size)
{
  AppCsvBuilder_t b;
  uint16_t year = 0U;
  uint8_t month = 0U, date = 0U;
  uint8_t hours = 0U, minutes = 0U, seconds = 0U;

  if ((app == NULL) || (buffer == NULL) || (buffer_size == 0U)) { return 0U; }

  if (app->rtc_read_ok != 0U) {
    year = app->rtc_datetime.year;   month = app->rtc_datetime.month;
    date = app->rtc_datetime.date;   hours = app->rtc_datetime.hours;
    minutes = app->rtc_datetime.minutes; seconds = app->rtc_datetime.seconds;
  }

  app_csv_init(&b, buffer, buffer_size);

  /* 字段顺序必须与注释中声明的一致，旧上位机依赖此顺序解析。 */
  app_csv_appendf(&b, "M,");

  /* -- 时间戳 -- */
  app_csv_appendf(&b, "%u,",        (unsigned int)app->rtc_time_valid);
  app_csv_appendf(&b, "%04u%02u%02u,", (unsigned int)year, (unsigned int)month, (unsigned int)date);
  app_csv_appendf(&b, "%02u%02u%02u,", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);

  /* -- PPG 原始值 -- */
  app_csv_appendf(&b, "%lu,", (unsigned long)app->red_value);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ir_value);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->baseline_ir);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->finger_present);

  /* -- BPM/SpO2/RR -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->bpm_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->bpm_value);
  app_csv_appendf(&b, "%u,", (unsigned int)app->spo2_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->spo2_value);
  app_csv_appendf(&b, "%u,", (unsigned int)app->rr_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->rr_bpm);

  /* -- IBI/HRV -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->ibi_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->latest_ibi_ms);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_mean_ibi_ms);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_sdnn_ms);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_rmssd_ms);

  /* -- 运动/SD/节律 -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->motion_artifact);
  app_csv_appendf(&b, "%u,", (unsigned int)app->motion_score);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_sd1_ms);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_sd2_ms);
  app_csv_appendf(&b, "%u,", (unsigned int)app->hrv_sd1_sd2_x100);
  app_csv_appendf(&b, "%u,", (unsigned int)app->rhythm_irregular);

  /* -- HRV 频域 -- */
  app_csv_appendf(&b, "%u,",  (unsigned int)app->hrv_freq_valid);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->hrv_lf_power_x100);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->hrv_hf_power_x100);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->hrv_lf_hf_x100);

  /* -- SQ / PI -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->signal_quality);
  app_csv_appendf(&b, "%u,", (unsigned int)app->raw_signal_present);
  app_csv_appendf(&b, "%u,", (unsigned int)app->signal_ir_pi_x1000);
  app_csv_appendf(&b, "%u,", (unsigned int)app->signal_red_pi_x1000);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->signal_ir_ac_rms);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->signal_red_ac_rms);

  /* -- SpO2 比率 -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->spo2_ratio_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->spo2_ratio_x1000);
  app_csv_appendf(&b, "%u,", (unsigned int)app->spo2_balance_status);

  /* -- 基线 / 自适应 -- */
  app_csv_appendf(&b, "%lu,", (unsigned long)app->baseline_range_ir);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->adaptive_finger_on_delta);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->adaptive_finger_off_delta);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ir_signal_delta);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ir_signal_span);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->red_signal_span);

  /* -- 手指确认 -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->finger_on_confirm_count);
  app_csv_appendf(&b, "%u,", (unsigned int)app->finger_off_confirm_count);

  /* -- 传感器统计 -- */
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_last_read_status);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_error_streak);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_fifo_write_ptr);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_fifo_read_ptr);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_fifo_overflow_count);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sensor_fifo_available_samples);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_read_ok_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_read_busy_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_read_error_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_recover_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_last_sample_tick);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_sample_change_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_sample_same_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sensor_last_i2c_error);

  /* -- RTC / UART -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->rtc_read_ok);
  app_csv_appendf(&b, "%u,", (unsigned int)app->uart_rx_message_valid);
  app_csv_appendf(&b, "%u,", (unsigned int)app->uart_tx_message_valid);

  /* -- SD 日志 -- */
  app_csv_appendf(&b, "%u,",  (unsigned int)(APP_DataLog_IsActive() ? 1U : 0U));
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sd_state);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->sd_error);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->sd_total_written);

  /* -- 显示 -- */
  app_csv_appendf(&b, "%lu,", (unsigned long)app->display_refresh_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->display_last_refresh_tick);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->page_mode);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->current_page);

  /* -- ECG/PTT (旧上位机忽略尾部即可) -- */
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_valid);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_hr);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_rr_ms);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_lead_off);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_r_peak_ms);
  app_csv_appendf(&b, "%d,",  (int)app->ecg_filtered);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ptt_valid);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ptt_ms);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_sample_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_adc_sat_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_dma_overflow_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_lead_off_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_no_r_peak_timeout_count);

  /* -- 崩溃诊断 (旧上位机忽略尾部即可) -- */
  app_csv_appendf(&b, "%u,",  (unsigned int)app->crash_flag);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->crash_source);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->crash_task);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->crash_phase);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->crash_tick);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->reboot_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->reset_flags);

  /* -- 任务阶段码 / 栈水印 / 心跳 -- */
  app_csv_appendf(&b, "%u,", (unsigned int)app->max_task_phase);
  app_csv_appendf(&b, "%u,", (unsigned int)app->ui_task_phase);
  app_csv_appendf(&b, "%u,", (unsigned int)app->sd_task_phase);
  app_csv_appendf(&b, "%u,", (unsigned int)app->wdt_task_phase);
  app_csv_appendf(&b, "%u,", (unsigned int)app->max_task_stack_hwm);
  app_csv_appendf(&b, "%u,", (unsigned int)app->ui_task_stack_hwm);
  app_csv_appendf(&b, "%u,", (unsigned int)app->sd_task_stack_hwm);
  app_csv_appendf(&b, "%u,", (unsigned int)app->wdt_task_stack_hwm);

  /* 任务心跳 (100-101) */
  app_csv_appendf(&b, "%lu,", (unsigned long)app->max_task_heartbeat);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ui_task_heartbeat);

  /* ECG 质量与诊断 (102-109) */
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_signal_quality);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_invalid_reason);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_raw_span);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_filtered_span);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_noise_level);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ecg_qrs_threshold);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ecg_peak_snr_x100);

  /* 追加式尾部诊断字段：保持前序字段顺序，旧上位机仍可按前缀解析。
   * Append-only diagnostics: keep existing field order stable for old parsers. */
  app_csv_appendf(&b, "%u,", (unsigned int)app->ecg_dma_available_high_watermark);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ppg_sqi_score);
  app_csv_appendf(&b, "%u,",  (unsigned int)app->ppg_sqi_flags);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_low_perfusion_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_motion_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_balance_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_transition_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_ibi_reject_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_amp_reject_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ppg_sqi_suppressed_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ptt_reject_ecg_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ptt_reject_ppg_count);
  app_csv_appendf(&b, "%lu,", (unsigned long)app->ptt_reject_range_count);
  app_csv_appendf(&b, "%lu",  (unsigned long)app->ptt_reject_jump_count);

  return (uint16_t)b.len;
}

/**
 * @brief  通过 USART2 发送带 CRLF 结尾的文本行。
 * @param  app          应用状态指针（用于更新标志位）。
 * @param  payload      待发送字符串（尚未附加 CRLF）。
 * @param  payload_len  payload 字符串长度。
 * @return payload 和 CRLF 均发送成功时返回 true，
 *         否则返回 false（同时更新 app->uart_tx_message_valid）。
 * @note   每段发送使用阻塞式 HAL_UART_Transmit，超时 100 ms。
 *         静态 CRLF 字节序列会自动附加到末尾。
 */
/* 通过 USART2 发送一行文本，并在末尾补充 CRLF。 */
static bool send_uart_line(AppState_t *app, const char *payload, uint16_t payload_len)
{
  HAL_StatusTypeDef status;
  static const uint8_t line_end[] = "\r\n";

  if ((app == NULL) || (payload == NULL) || (payload_len == 0U))
  {
    if (app != NULL)
    {
      app->uart_tx_message_valid = false;
    }

    return false;
  }

  status = HAL_UART_Transmit(&huart2,
                             (uint8_t *)payload,
                             (uint16_t)payload_len,
                             STM32_UART_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    app->uart_tx_message_valid = false;
    return false;
  }

  status = HAL_UART_Transmit(&huart2,
                             (uint8_t *)line_end,
                             (uint16_t)(sizeof(line_end) - 1U),
                             STM32_UART_TIMEOUT_MS);
  app->uart_tx_message_valid = (status == HAL_OK);
  return app->uart_tx_message_valid;
}

/**
 * @brief  跳过字符串开头的空白字符（空格和制表符）。
 * @param  text 输入字符串指针。
 * @return 指向第一个非空白字符的指针；若 text 为 NULL 则返回 NULL。
 */
/* 跳过命令字符串前导空格。 */
static const char *app_skip_spaces(const char *text)
{
  if (text == NULL)
  {
    return NULL;
  }

  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }

  return text;
}

/* 不区分大小写地判断字符串是否以某个关键字开头。 */
static uint8_t app_text_starts_with_keyword(const char *text, const char *keyword)
{
  if ((text == NULL) || (keyword == NULL))
  {
    return 0U;
  }

  while (*keyword != '\0')
  {
    if (*text == '\0')
    {
      return 0U;
    }

    if (toupper((unsigned char)*text) != toupper((unsigned char)*keyword))
    {
      return 0U;
    }

    text++;
    keyword++;
  }

  return 1U;
}

/* 解析 TIME / SETTIME 命令后面的时间负载。 */
static const char *app_get_time_command_payload(const char *line)
{
  const char *cursor;
  uint8_t keyword_length;

  cursor = app_skip_spaces(line);
  if (cursor == NULL)
  {
    return NULL;
  }

  if (app_text_starts_with_keyword(cursor, "TIME") != 0U)
  {
    keyword_length = 4U;
  }
  else if (app_text_starts_with_keyword(cursor, "SETTIME") != 0U)
  {
    keyword_length = 7U;
  }
  else
  {
    return NULL;
  }

  if ((cursor[keyword_length] != '\0') &&
      (cursor[keyword_length] != ' ') &&
      (cursor[keyword_length] != '\t') &&
      (cursor[keyword_length] != '='))
  {
    return NULL;
  }

  cursor += keyword_length;
  cursor = app_skip_spaces(cursor);
  if ((cursor != NULL) && (*cursor == '='))
  {
    cursor++;
  }

  return app_skip_spaces(cursor);
}

/* 固定位数十进制解析器，用于解析 yyyy/mm/dd/hh/mm/ss。 */
static uint8_t app_parse_fixed_uint(const char *text, uint8_t digits, uint16_t *value)
{
  uint8_t i;
  uint16_t parsed_value = 0U;

  if ((text == NULL) || (value == NULL))
  {
    return 0U;
  }

  for (i = 0U; i < digits; i++)
  {
    if (isdigit((unsigned char)text[i]) == 0)
    {
      return 0U;
    }

    parsed_value = (uint16_t)((parsed_value * 10U) + (uint16_t)(text[i] - '0'));
  }

  *value = parsed_value;
  return 1U;
}

/* 解析形如 2026-04-14 12:34:56 的时间文本。 */
static uint8_t app_parse_datetime_text(const char *text, APP_RTC_DateTime_t *date_time)
{
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hours;
  uint16_t minutes;
  uint16_t seconds;
  const char *tail;

  if ((text == NULL) || (date_time == NULL))
  {
    return 0U;
  }

  if (strlen(text) < 19U)
  {
    return 0U;
  }

  if ((text[4] != '-') || (text[7] != '-') ||
      ((text[10] != ' ') && (text[10] != 'T')) ||
      (text[13] != ':') || (text[16] != ':'))
  {
    return 0U;
  }

  if ((app_parse_fixed_uint(text, 4U, &year) == 0U) ||
      (app_parse_fixed_uint(&text[5], 2U, &month) == 0U) ||
      (app_parse_fixed_uint(&text[8], 2U, &day) == 0U) ||
      (app_parse_fixed_uint(&text[11], 2U, &hours) == 0U) ||
      (app_parse_fixed_uint(&text[14], 2U, &minutes) == 0U) ||
      (app_parse_fixed_uint(&text[17], 2U, &seconds) == 0U))
  {
    return 0U;
  }

  tail = app_skip_spaces(&text[19]);
  if ((tail == NULL) || (*tail != '\0'))
  {
    return 0U;
  }

  date_time->year = year;
  date_time->month = (uint8_t)month;
  date_time->date = (uint8_t)day;
  date_time->weekday = 0U;
  date_time->hours = (uint8_t)hours;
  date_time->minutes = (uint8_t)minutes;
  date_time->seconds = (uint8_t)seconds;
  return 1U;
}

/*
 * RTC 设置回包格式： * T,set_ok,rtc_valid,yyyymmdd,hhmmss,reason
 */
static void app_send_rtc_set_response(AppState_t *app, uint8_t success, const char *reason)
{
  char response[160];
  uint16_t year = 0U;
  uint16_t month = 0U;
  uint16_t date = 0U;
  uint16_t hours = 0U;
  uint16_t minutes = 0U;
  uint16_t seconds = 0U;
  uint8_t rtc_valid = 0U;

  if (app != NULL)
  {
    rtc_valid = app->rtc_time_valid;

    if (app->rtc_read_ok != 0U)
    {
      year = app->rtc_datetime.year;
      month = app->rtc_datetime.month;
      date = app->rtc_datetime.date;
      hours = app->rtc_datetime.hours;
      minutes = app->rtc_datetime.minutes;
      seconds = app->rtc_datetime.seconds;
    }
  }

  (void)snprintf(response,
                 sizeof(response),
                 "T,%u,%u,%04u%02u%02u,%02u%02u%02u,%s",
                 (unsigned int)success,
                 (unsigned int)rtc_valid,
                 (unsigned int)year,
                 (unsigned int)month,
                 (unsigned int)date,
                 (unsigned int)hours,
                 (unsigned int)minutes,
                 (unsigned int)seconds,
                 (reason != NULL) ? reason : "ok");

  (void)send_uart_line(app, response, (uint16_t)strlen(response));
}

/* 处理单行串口命令，支持 eeprom 和 time 两类命令。 */
static bool app_process_uart_line(AppState_t *app, const char *line)
{
  APP_RTC_DateTime_t new_time = {0};
  const char *payload;

  if ((app == NULL) || (line == NULL))
  {
    return false;
  }

  /* EEPROM commands: info/dump/reset/setsn/sethw/setmfg/setled */
  if (eeprom_cmd_process(app, line))
  {
    return true;
  }

  payload = app_get_time_command_payload(line);
  if (payload == NULL)
  {
    return false;
  }

  if (app_parse_datetime_text(payload, &new_time) == 0U)
  {
    app_send_rtc_set_response(app, 0U, "bad_format");
    return false;
  }

  if (APP_RTC_SetDateTime(&new_time) != HAL_OK)
  {
    app_send_rtc_set_response(app, 0U, "invalid_datetime");
    return false;
  }

  app_protocol_update_rtc_snapshot(app);
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
  app_send_rtc_set_response(app, 1U, NULL);
  return true;
}
