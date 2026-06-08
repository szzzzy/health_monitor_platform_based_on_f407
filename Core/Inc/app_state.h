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
  DISPLAY_PAGE_ECG,
  DISPLAY_PAGE_DEBUG,
  DISPLAY_PAGE_COUNT,
  DISPLAY_PAGE_NORMAL_COUNT = DISPLAY_PAGE_ECG + 1U
} DisplayPage_t;

/* Debug 子页面类型。 */
typedef enum
{
  DBG_SUB_D1_MAX = 0U,
  DBG_SUB_D2_FIFO,
  DBG_SUB_D3_PPG_RAW,
  DBG_SUB_D4_PPG_Q,
  DBG_SUB_D5_ALGO,
  DBG_SUB_D6_SYS,
  DBG_SUB_D7_SD,
  DBG_SUB_COUNT
} DebugSubPage_t;

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
  uint32_t sensor_read_attempt_count;
  uint32_t sensor_read_ok_count;
  uint32_t sensor_read_busy_count;
  uint32_t sensor_read_error_count;
  uint32_t sensor_recover_count;
  uint32_t sensor_last_sample_tick;
  uint32_t sensor_sample_change_count;
  uint32_t sensor_sample_same_count;
  uint32_t sensor_last_i2c_error;
  uint32_t sensor_stale_count;
  uint32_t sensor_recovery_fail_count;
  uint32_t display_refresh_count;
  uint32_t display_last_refresh_tick;
  uint8_t sensor_fifo_write_ptr;
  uint8_t sensor_fifo_read_ptr;
  uint16_t sensor_fifo_overflow_count;
  uint8_t sensor_fifo_available_samples;
  uint16_t fifo_high_watermark;      /* 批量 drain 的峰值样本数，用于 backlog 门控 */
  uint16_t max_sample_gap_ms;        /* 连续样本间最大时间间隔 (ms) */
  uint16_t display_skipped_count;    /* 累计跳过的 OLED 刷新次数 */
  uint8_t  flush_pending;            /* 1 = DeferredStop 待执行 */
  uint32_t fifo_overflow_total;      /* FIFO overflow 累计次数 */
  uint32_t oled_reinit_count;        /* OLED 重新初始化次数 */
  uint32_t i2c_recover_count;        /* I2C 总线恢复次数（启动+运行时） */
  uint16_t ui_forced_count;          /* OLED 强制刷新次数（跳过超时后） */
  uint32_t last_ui_skip_tick;        /* 最近一次 OLED 跳过的时间戳 */
  uint32_t max_task_heartbeat;       /* MAXtask 心跳（每轮递增） */
  uint32_t ui_task_heartbeat;        /* UiTask 心跳（每轮递增） */
  uint8_t raw_signal_present;
  uint8_t signal_quality;
  /*
   * PPG-only motion 检测：不使用加速度计，纯从 PPG 信号特征推断。
   * motion_artifact=1 → 冻结 HR/SpO2/RR 输出（valid=0，旧值保留）。
   * Motion 期间不清 HRV/IBI ring buffer——只标 invalid。
   * 手指离开 / runtime reset / sensor recovery 才会清 HRV/RR 历史。
   */
  uint8_t motion_artifact;
  /* 原始 motion score 0–100，基于 AC RMS 尖峰 + RED/IR 平衡 + SQ 骤降三条信号合成。 */
  uint8_t motion_score;
  /* PI：存储 (AC/DC) × 1000，即标准灌注指数(%) × 10。例如 PI=12.3% → 值 123。
   * OXY 页显示为 PI:12.3（除以 10 取整+余数）。uint16 承载 >25.5% 的 PI 值。
   * signal_ir_pi_x1000 由 beat-based EMA 计算（ir_pi_ac_ema / ir_dc），
   * 不再从窗口 RMS 推导，避免稳定佩戴后 PI 衰减到 0.1。 */
  uint16_t signal_ir_pi_x1000;
  uint16_t signal_red_pi_x1000;
  uint32_t ir_pi_ac_ema;       /* accepted beat peak-to-trough amplitude 的 EMA */
  uint8_t  ir_pi_ac_ema_valid; /* 置 1 后至少有一个已接受的 beat */
  uint32_t last_beat_sample;   /* 上一次接受 beat 的全局样本编号，用于 stale 判定 */
  uint8_t sensor_last_read_status;
  uint8_t sensor_error_streak;
  uint8_t sensor_health;           /* 0=OK, 1=STALE, 2=RECOVERING, 3=I2C_ERR, 4=INIT_FAIL, 5=FIFO_CLEAR_FAIL */
  uint32_t sensor_last_ok_tick;    /* 最后一次成功读到新样本的 tick */
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
  /*
   * IBI/HRV 由流式脉冲检测 (app_stream_pulse_update) 驱动。
   * valid=1 表示 ring buffer 中已有 >=3 个有效间隔。
   * 频域 HRV 仅提供 32 拍短窗口 LF/HF 估计，不实现 VLF，也不是标准 5 分钟频谱分析。
   */
  uint8_t ibi_valid;
  uint16_t latest_ibi_ms;
  uint16_t accepted_ibi_count;  /* 本次测量接受的 IBI 总数（调试用） */
  uint8_t hrv_valid;
  uint16_t hrv_mean_ibi_ms;
  uint16_t hrv_sdnn_ms;
  uint16_t hrv_rmssd_ms;
  /*
   * SD1/SD2 是基于同一 32 拍 IBI ring buffer 的 Poincare 短窗口描述符，
   * 不是诊断分类。
   * SD1 = RMSSD / sqrt(2)  （短时变异性，Poincare 椭圆短轴）。
   * SD2 = sqrt(2*SDNN^2 - SD1^2)  （长时变异性，Poincare 椭圆长轴）。
   * SD1/SD2 x100 = 短/长期变异性比值。
   */
  uint16_t hrv_sd1_ms;
  uint16_t hrv_sd2_ms;
  uint16_t hrv_sd1_sd2_x100;
  /*
   * Short-window frequency HRV estimate from the same 32-beat IBI buffer.
   * hrv_lf_power_x100 / hrv_hf_power_x100 are band powers in ms^2 x100.
   * hrv_lf_hf_x100 is LF/HF x100. Non-diagnostic trend display only.
   */
  uint8_t hrv_freq_valid;
  uint32_t hrv_lf_power_x100;
  uint32_t hrv_hf_power_x100;
  uint16_t hrv_lf_hf_x100;
  /*
   * 短窗口节律提示，不是诊断分类。
   * 同时满足 RMSSD >= 120 ms 且 SD1/SD2 x100 >= 70 时置位。
   * 高 SD1/SD2 比值 + 高 RMSSD 可能暗示节律不齐，
   * 但判读需要临床上下文。
   */
  uint8_t rhythm_irregular;
  /* RR：比 IBI/HRV 更慢——需要 SQ>=25、>=8 拍、>=6 秒窗口且脉搏幅度有明显调制。 */
  uint8_t rr_valid;
  uint8_t rr_bpm;
  /* R/BAL：由 RED/IR 的 AC RMS 和 DC 原始值计算比值 R，3:1 平滑。弱帧只标 invalid 不清旧值。 */
  uint8_t spo2_ratio_valid;
  uint16_t spo2_ratio_x1000;
  uint8_t spo2_balance_status;
  uint8_t refresh_div;
  uint8_t finger_on_confirm_count;
  uint8_t finger_off_confirm_count;
  uint16_t contact_settle_samples; /* 手指刚放上后的接触稳定倒计数（100Hz 节拍） */
  uint8_t report_due;
  uint8_t display_refresh_requested;
  uint8_t debug_mode;
  DebugSubPage_t debug_sub_page;
  DisplayPage_t saved_normal_page;
  /* 最近一次完整串口接收/发送的报文是否有效。 */
  bool uart_rx_message_valid;
  bool uart_tx_message_valid;
  DisplayPage_t current_page;
  uint8_t rtc_time_valid;
  uint8_t rtc_read_ok;
  APP_RTC_DateTime_t rtc_datetime;
  PageButton_t debug_toggle_button;
  PageButton_t page_prev_button;
  PageButton_t page_next_button;
  /* SD 卡数据记录状态 */
  uint16_t sd_buffered;       /* 环形缓冲中待写入样本数 */
  uint16_t sd_dropped;        /* 累计丢弃样本数 */
  uint16_t sd_written;        /* 本次 session 已持久化样本数 */
  uint8_t  sd_paused;         /* 1 = SD 写入暂停（慢/满/错误） */
  uint8_t  sd_error;          /* 最后一次 SD 错误码 */
  uint8_t  sd_state;          /* 内部状态: 0=IDLE 1=TRY 2=ACTIVE 3=BACKOFF */
  uint32_t sd_last_write_ms;  /* 最近一次 SD 写入耗时 (ms) */
  uint32_t sd_total_written;  /* 累计持久化样本总数 */
  /* ECG 状态：原始 ADC 值、滤波后 AC 值、导联脱落标志 */
  uint16_t ecg_raw;
  int16_t  ecg_filtered;
  uint8_t  ecg_lead_off;
  uint8_t  ecg_valid;
  uint8_t  ecg_hr;
  uint16_t ecg_rr_ms;
  uint32_t ecg_r_peak_ms;
  /* ECG 可观测状态计数器 */
  uint32_t ecg_sample_count;
  uint32_t ecg_adc_fail_count;
  uint32_t ecg_dma_overflow_count;
  uint32_t ecg_adc_sat_count;
  uint32_t ecg_lead_off_count;
  uint32_t ecg_no_r_peak_timeout_count;
  /* PTT 状态 */
  uint8_t  ptt_valid;
  uint16_t ptt_ms;
  /* ---- 诊断阶段码：每个任务在关键路径前写入，崩溃时从 BKP 寄存器恢复 ---- */
  uint8_t  max_task_phase;       /* MAXtask 当前阶段 (0=idle/等待通知) */
  uint8_t  ui_task_phase;        /* UiTask 当前阶段 */
  uint8_t  sd_task_phase;        /* SDtask 当前阶段 */
  uint8_t  wdt_task_phase;       /* watchdogtask 当前阶段 */
  uint8_t  crash_flag;           /* 1 = 上次启动检测到崩溃 (BKP 中有有效记录) */
  uint8_t  crash_source;         /* 崩溃源：1=HardFault 2=Mem 3=Bus 4=Usage 5=StackOvf 6=Assert */
  uint8_t  crash_task;           /* 崩溃时所在任务 (1=MAX 2=WDT 3=UI 4=SD 5=default) */
  uint8_t  crash_phase;          /* 崩溃时任务的阶段码 */
  uint32_t crash_tick;           /* 崩溃时的系统 tick */
  uint32_t reset_flags;          /* RCC_CSR 复位标志 (启动时捕获) */
  uint32_t reboot_count;         /* 累计重启次数 (从 BKP 恢复) */
  /* 各任务栈高水位 (0=未采样)，由 UiTask 周期性采集 */
  uint16_t max_task_stack_hwm;   /* MAXtask 栈最小剩余字 */
  uint16_t ui_task_stack_hwm;
  uint16_t sd_task_stack_hwm;
  uint16_t wdt_task_stack_hwm;
} AppState_t;

/* ---- 诊断阶段码定义 ---- */
/* MAXtask phases */
#define PHASE_MAX_IDLE           0U
#define PHASE_MAX_NOTIFY_WAIT    1U
#define PHASE_MAX_ECG            2U
#define PHASE_MAX_FIFO_CHECK     3U
#define PHASE_MAX_I2C_ACQ        4U
#define PHASE_MAX_FIFO_DRAIN     5U
#define PHASE_MAX_SAMPLE_PROC    6U
#define PHASE_MAX_RECOVERY       7U
#define PHASE_MAX_WATCHDOG       8U
#define PHASE_MAX_HEARTBEAT      9U

/* UiTask phases */
#define PHASE_UI_IDLE            0U
#define PHASE_UI_POLL_UART       1U
#define PHASE_UI_BUTTONS         2U
#define PHASE_UI_DISPLAY_CHECK   3U
#define PHASE_UI_DISPLAY_REFRESH 4U
#define PHASE_UI_REPORT_SEND     5U
#define PHASE_UI_DELAY           6U

/* SDtask phases */
#define PHASE_SD_IDLE            0U
#define PHASE_SD_SET_ACTIVE      1U
#define PHASE_SD_SAFE_CHECK      2U
#define PHASE_SD_FLUSH           3U
#define PHASE_SD_SERVICE_BUDGET  4U
#define PHASE_SD_STATUS          5U
#define PHASE_SD_DELAY           6U

/* Watchdogtask phases */
#define PHASE_WDT_IDLE           0U
#define PHASE_WDT_REFRESH        1U
#define PHASE_WDT_DELAY          2U

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_H__ */
