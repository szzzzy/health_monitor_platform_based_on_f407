#ifndef __APP_STATE_H__
#define __APP_STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "main.h"
#include "rtc.h"

/* 应用层固定按 10 ms 节拍运行，对应约 100 Hz 采样/调度频率。 */
#define APP_SAMPLE_PERIOD_MS 10U

/* R/BAL 分级：基于 RED/IR 的 AC/DC 比值落在低/正常/高区间。 */
#define APP_OXY_BALANCE_UNKNOWN 0U
#define APP_OXY_BALANCE_OK      1U
#define APP_OXY_BALANCE_LOW     2U
#define APP_OXY_BALANCE_HIGH    3U

/* OLED 当前支持的页面类型。VITALS 页汇总展示 HR/RR/IBI/HRV/PI/SQ 等多种指标。 */
typedef enum
{
  DISPLAY_PAGE_BPM = 0U,
  DISPLAY_PAGE_SPO2,
  DISPLAY_PAGE_VITALS,
  DISPLAY_PAGE_DEBUG,
  DISPLAY_PAGE_COUNT
} DisplayPage_t;

/*
 * 轮询式按键状态：
 * - pressed_latch 用于避免长按时重复触发
 * - cooldown_ticks 用于实现简单的软件消抖
 */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t pressed_latch;
  uint8_t cooldown_ticks;
} PageButton_t;

/*
 * 应用运行时的共享状态。
 * 这里集中保存主循环会反复读写的测量值、页面状态、RTC 快照与按键状态，
 * 这样各模块之间只传一个上下文指针，后续维护更容易。
 */
typedef struct
{
  uint32_t red_value;
  uint32_t ir_value;
  uint32_t baseline_ir;
  uint32_t baseline_range_ir;
  uint32_t adaptive_finger_on_delta;
  uint32_t adaptive_finger_off_delta;
  uint32_t ir_signal_delta;
  uint32_t ir_signal_span;
  uint32_t red_signal_span;
  uint32_t signal_ir_ac_rms;
  uint32_t signal_red_ac_rms;
  uint32_t sensor_read_ok_count;
  uint32_t sensor_read_busy_count;
  uint32_t sensor_read_error_count;
  uint32_t sensor_recover_count;
  uint32_t sensor_last_sample_tick;
  uint32_t sensor_sample_change_count;
  uint32_t sensor_sample_same_count;
  uint32_t sensor_last_i2c_error;
  uint32_t display_refresh_count;
  uint32_t display_last_refresh_tick;
  uint8_t sensor_fifo_write_ptr;
  uint8_t sensor_fifo_read_ptr;
  uint8_t sensor_fifo_overflow_count;
  uint8_t sensor_fifo_available_samples;
  uint8_t raw_signal_present;
  uint8_t signal_quality;
  /* PI：存储 (AC/DC) × 1000，即标准灌注指数(%) × 10。例如 PI=12.3% → 值 123。
   * OXY 页显示为 PI:12.3（除以 10 取整+余数）。uint16 承载 >25.5% 的 PI 值。 */
  uint16_t signal_ir_pi_x1000;
  uint16_t signal_red_pi_x1000;
  uint8_t sensor_last_read_status;
  uint8_t sensor_error_streak;
  uint8_t finger_present;
  uint8_t bpm_valid;
  uint8_t bpm_value;
  uint8_t raw_bpm_history[3];
  uint8_t raw_bpm_history_count;
  uint8_t raw_bpm_history_index;
  uint8_t bpm_candidate_value;
  uint8_t bpm_candidate_count;
  uint8_t bpm_invalid_hold_count;
  uint8_t spo2_valid;
  uint8_t spo2_value;
  /* IBI/HRV：由流式脉冲检测 (app_stream_pulse_update) 驱动。valid=1 表示 32 拍窗口内已有 >=4 个有效间隔。 */
  uint8_t ibi_valid;
  uint16_t latest_ibi_ms;
  uint8_t hrv_valid;
  uint16_t hrv_mean_ibi_ms;
  uint16_t hrv_sdnn_ms;
  uint16_t hrv_rmssd_ms;
  /* RR：比 IBI/HRV 更慢更严格——需要 SQ>=35、>=10 拍、>=8 秒窗口且脉搏幅度有明显调制。 */
  uint8_t rr_valid;
  uint8_t rr_bpm;
  /* R/BAL：由 RED/IR 的 AC RMS 和 DC 原始值计算比值 R，3:1 平滑。弱帧只标 invalid 不清旧值。 */
  uint8_t spo2_ratio_valid;
  uint16_t spo2_ratio_x1000;
  uint8_t spo2_balance_status;
  uint8_t refresh_div;
  uint8_t finger_on_confirm_count;
  uint8_t finger_off_confirm_count;
  uint8_t report_due;
  uint8_t display_refresh_requested;
  uint8_t display_brightness_index;
  /* 最近一次完整串口接收/发送的报文是否有效。 */
  bool uart_rx_message_valid;
  bool uart_tx_message_valid;
  DisplayPage_t current_page;
  uint8_t rtc_time_valid;
  uint8_t rtc_read_ok;
  APP_RTC_DateTime_t rtc_datetime;
  PageButton_t brightness_button;
  PageButton_t page_prev_button;
  PageButton_t page_next_button;
  /* SD 卡数据记录状态 */
  uint8_t  sd_card_ready;
  uint8_t  sd_log_active;
  uint8_t  sd_log_error;
  uint32_t sd_total_written;
} AppState_t;

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_H__ */
