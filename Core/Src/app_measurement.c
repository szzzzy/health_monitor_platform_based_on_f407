/**
  ******************************************************************************
  * @file    app_measurement.c
  * @brief   测量流水线 — 传感器读取、手指检测、BPM/SpO2 算法调度
  *
  * 职责：
  *   1. MAX30102 FIFO 读取与原始数据解析
  *   2. 背景基线采集（上电无手指阶段）
  *   3. 手指就位/离开检测（滞回状态机 + 自适应阈值）
  *   4. 测量处理流水线调度（SpO2 窗口 → 信号质量 → motion → BPM）
  *   5. 传感器恢复（连续 I2C 错误超阈值后重建初始化）
  *   6. 200ms 周期上报/刷新标志管理
  *
  * 本模块不直接操作 OLED / SD / UART — 只更新 AppState 字段，
  * 由 main.c 的调度循环统一驱动显示和上报。
  ******************************************************************************
  */

#include "app_measurement.h"

#include <string.h>

#include "app_bpm_filter.h"
#include "app_data_log.h"
#include "app_display.h"
#include "app_hrv.h"
#include "app_motion.h"
#include "app_oxy_status.h"
#include "app_ppg_pulse.h"
#include "app_ppg_signal.h"
#include "app_ptt.h"
#include "app_rr.h"
#include "app_spo2_filter.h"
#include "i2c.h"
#include "max30102.h"

/* 传感器恢复：连续错误超过此次数 → 重建 I2C + MAX30102 初始化 */
#define APP_SENSOR_RECOVERY_ERROR_COUNT   8U
/* BPM 评估节奏：每 3 个样本运行一次，降低计算开销 */
#define APP_BPM_EVALUATE_INTERVAL_SAMPLES 3U
/* 自相关 BPM 与时域 BPM 混合的最大允许偏差 (bpm) */
#define APP_BPM_ACORR_BLEND_MAX_DIFF      12U
/* 信号质量下限：低于此值抑制 SpO2 输出 */
#define APP_SIGNAL_QUALITY_MIN_FOR_SPO2   30U
/* 信号质量下限：低于此值抑制 BPM 输出 */
#define APP_SIGNAL_QUALITY_MIN_FOR_BPM    25U
/* 高级指标超时：无新脉搏超过此窗口则清零 HRV/RR */
#define APP_ADVANCED_PULSE_STALE_SAMPLES  (MAX30102_ALGO_SAMPLE_RATE_HZ * 8U)
/* 手指接触稳定倒计数：避免接触瞬态污染检测器 */
#define APP_CONTACT_SETTLE_SAMPLES        200U
/* beat-based PI 超时：无新 beat 超过此窗口则标无效 */
#define APP_PI_STALE_SAMPLES              (MAX30102_ALGO_SAMPLE_RATE_HZ * 5U)
/* 传感器无新样本看门狗超时 (ms) */
#define APP_SENSOR_STALE_WARN_MS          1000U  /* OLED 显示 SENSOR WAIT 的阈值 */
#define APP_SENSOR_STALE_TIMEOUT_MS       3000U
#define APP_SENSOR_STALE_CONFIRM_INTERVAL_MS 1000U
#define APP_SENSOR_STALE_CONFIRM_COUNT       3U
/* 两次恢复尝试之间的最小间隔 (ms) */
#define APP_SENSOR_RECOVERY_RETRY_MS      5000U
#define APP_SENSOR_RECOVERY_INIT_ATTEMPTS 2U

/* FIFO 读取缓冲：SpO2 模式每样本 6 字节 (RED[3] + IR[3]) */
static uint8_t fifo_buf[6];
/* 背景基线统计（无手指阶段累积 IR min/max/avg） */
static MAX30102_Baseline_t baseline_data;
/* SpO2 滑动窗口状态（RED/IR 原始值 + 滤波值环形缓冲） */
static MAX30102_SpO2_t spo2_state;
/* 样本变化调试追踪：检测连续样本是否更新 */
static struct
{
  uint32_t red_value;
  uint32_t ir_value;
  uint8_t initialized;
} sample_debug_state;
/* BPM 评估抽头计数器：每 N 个样本才跑一次完整 BPM 计算 */
static uint8_t bpm_update_decimator;
/* A delayed main loop must get a chance to drain FIFO before bus recovery. */
static uint32_t sensor_last_recovery_tick = 0UL;
static uint32_t sensor_last_stale_probe_tick = 0UL;
static uint8_t  sensor_stale_probe_count = 0U;
static uint8_t  sensor_recovery_fail_count = 0U;

static void app_reset_measurement_outputs(AppState_t *app);
static void app_invalidate_advanced_outputs(AppState_t *app);
static void app_reset_advanced_metrics(AppState_t *app);
static HAL_StatusTypeDef app_measurement_reinit_sensor(void);

/* 初始化测量子系统的 AppState 字段（手指阈值、包络等）。 */
void app_measurement_init_state(AppState_t *app)
{
  app_ppg_signal_init_state(app);
}

/* 上电或手指状态切换时重置全部测量运行时状态。 */
void app_measurement_reset_runtime(void)
{
  max30102_baseline_reset(&baseline_data);
  max30102_spo2_reset(&spo2_state);
  (void)memset(&sample_debug_state, 0, sizeof(sample_debug_state));
  bpm_update_decimator = 0U;
  app_spo2_filter_reset(NULL);
  app_motion_reset(NULL);
  app_ppg_signal_reset_envelope();
  app_reset_advanced_metrics(NULL);
  app_display_reset_waveforms();
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  sensor_recovery_fail_count = 0U;
}

/*
 * 基线采集阶段：读取一个 FIFO 样本并纳入背景统计。
 *
 * 返回值：1 = 成功采集一个样本，0 = 未采到（I2C 忙/错误/FIFO 空）。
 * 读忙 (HAL_BUSY) 不算错误，仅记录计数供调试。
 */

uint8_t app_measurement_collect_baseline_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;

  if (app == NULL)
  {
    return 0U;
  }

  read_status = max30102_read_fifo(fifo_buf, 6U);
  if (read_status != HAL_OK)
  {
    if (read_status == HAL_BUSY)
    {
      app->sensor_read_busy_count++;
      if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
        app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
        app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        if (app->sensor_error_streak < 0xFFU)
        {
          app->sensor_error_streak++;
        }
        return 0U;
      }
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
      return 0U;
    }

    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
    app->sensor_read_error_count++;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    if (app->sensor_error_streak < 0xFFU)
    {
      app->sensor_error_streak++;
    }
    app_measurement_recover_sensor(app);
    return 0U;
  }

  max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
  max30102_baseline_add_ir(&baseline_data, app->ir_value);
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
  app->sensor_read_ok_count++;
  app->sensor_error_streak = 0U;
  app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
  app->sensor_last_sample_tick = HAL_GetTick();
  app->sensor_last_ok_tick = app->sensor_last_sample_tick;
  app->sensor_health = (uint8_t)SENSOR_HEALTH_OK;
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  return 1U;
}

/* 基线是否已采够目标样本数。 */
uint8_t app_measurement_baseline_ready(void)
{
  return max30102_baseline_is_ready(&baseline_data, APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 基线采集进度百分比（0–100），供 OLED 状态页显示。 */
uint16_t app_measurement_get_baseline_progress_percent(void)
{
  if (baseline_data.sample_count >= APP_MEASUREMENT_BASELINE_SAMPLES)
  {
    return 100U;
  }

  return (uint16_t)((baseline_data.sample_count * 100U) / APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 背景基线平均 IR 值（基线阶段采完后使用）。 */
uint32_t app_measurement_get_baseline_average(void)
{
  return max30102_baseline_get_average_ir(&baseline_data);
}

/* 背景基线 IR 波动范围（用于评估背景稳定性）。 */
uint32_t app_measurement_get_baseline_range(void)
{
  return max30102_baseline_get_range_ir(&baseline_data);
}

/* 运行时动态跟踪基线（跟随环境光/温度漂移）。 */
uint32_t app_measurement_get_tracked_baseline(void)
{
  return max30102_baseline_get_tracked_ir(&baseline_data);
}

/* 用开机背景采样结果为运行时基线跟踪器设定初值。 */
void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir)
{
  max30102_baseline_seed_tracking(&baseline_data, baseline_ir, noise_ir);
}

/* 判断背景基线是否足够稳定（波动范围在门限内）。 */
uint8_t app_measurement_baseline_is_stable(void)
{
  return max30102_baseline_is_stable(&baseline_data, APP_MEASUREMENT_BASELINE_STABLE_RANGE);
}

/*
 * 主循环传感器读取入口 — 从 MAX30102 FIFO 读取一个 SpO2 样本。
 *
 * 流程：
 *   1. DMA 读取 FIFO（6 字节/RED+IR）
 *   2. 解析 RED/IR 原始值 → 写入 app->red_value / app->ir_value
 *   3. 更新 FIFO 调试字段（写指针、读指针、溢出计数）
 *   4. 更新样本变化追踪（调试用）
 *   5. 更新运行时基线跟踪值
 *
 * 返回值：APP_MEASUREMENT_READ_OK / WAIT / ERROR。
 *   WAIT = FIFO 中样本不足或 I2C 忙，上层应等下个节拍重试。
 *   ERROR = I2C 错误，上层可调用 app_measurement_recover_sensor()。
 */
AppMeasurementReadStatus_t app_measurement_read_sensor_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;
  const MAX30102_FifoDebug_t *fifo_debug;

  if (app == NULL)
  {
    return APP_MEASUREMENT_READ_ERROR;
  }

  if (app->sensor_read_attempt_count < 0xFFFFFFFFUL)
  {
    app->sensor_read_attempt_count++;
  }

  read_status = max30102_read_fifo(fifo_buf, 6U);
  fifo_debug = max30102_get_fifo_debug();
  if (fifo_debug != NULL)
  {
    app->sensor_fifo_overflow_count = fifo_debug->overflow_count;
    app->sensor_fifo_write_ptr = fifo_debug->write_ptr;
    app->sensor_fifo_read_ptr = fifo_debug->read_ptr;
    app->sensor_fifo_available_samples = fifo_debug->available_samples;
  }

  if (read_status == HAL_OK)
  {
    max30102_parse_spo2_sample(fifo_buf, &app->red_value, &app->ir_value);
    if (sample_debug_state.initialized != 0U)
    {
      if ((sample_debug_state.red_value != app->red_value) ||
          (sample_debug_state.ir_value != app->ir_value))
      {
        app->sensor_sample_change_count++;
      }
      else
      {
        app->sensor_sample_same_count++;
      }
    }
    else
    {
      app->sensor_sample_change_count++;
      sample_debug_state.initialized = 1U;
    }

    sample_debug_state.red_value = app->red_value;
    sample_debug_state.ir_value = app->ir_value;
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_ppg_signal_update_activity(app);
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
    app->sensor_read_ok_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    app->sensor_last_sample_tick = HAL_GetTick();
    app->sensor_last_ok_tick = app->sensor_last_sample_tick;
    sensor_last_stale_probe_tick = 0UL;
    sensor_stale_probe_count = 0U;
    app->sensor_health = (uint8_t)SENSOR_HEALTH_OK;

    /* 实时路径 PushSample：仅 16 字节 memcpy，零格式化/零 SD I/O */
    {
      uint8_t log_flags = 0U;
      if (app->finger_present != 0U)      log_flags |= 0x01U;
      if (app->contact_settle_samples > 0U) log_flags |= 0x02U;
      if (app->sensor_fifo_overflow_count > 0U) log_flags |= 0x04U;
      APP_DataLog_PushSample(app->sensor_last_sample_tick,
                             app->red_value, app->ir_value,
                             (int16_t)app->ecg_filtered,
                             log_flags);
    }

    return APP_MEASUREMENT_READ_OK;
  }

  if (read_status == HAL_BUSY)
  {
    app->sensor_read_busy_count++;
    /* BUSY + I2C 故障 → 返回 ERROR，走阈值恢复路径 */
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
      app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
      if (app->sensor_error_streak < 0xFFU)
      {
        app->sensor_error_streak++;
      }
      return APP_MEASUREMENT_READ_ERROR;
    }
    /* BUSY + I2C READY → FIFO 暂时空或溢出清空后的等待，保留 WAIT */
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
    return APP_MEASUREMENT_READ_WAIT;
  }

  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
  app->sensor_read_error_count++;
  app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
  app->sensor_health = (uint8_t)SENSOR_HEALTH_I2C_ERR;
  if (app->sensor_error_streak < 0xFFU)
  {
    app->sensor_error_streak++;
  }

  return APP_MEASUREMENT_READ_ERROR;
}

/* 根据背景噪声自适应更新手指检测的 on/off 阈值。 */
void app_measurement_update_adaptive_thresholds(AppState_t *app)
{
  app_ppg_signal_update_adaptive_thresholds(app, &baseline_data);
}

/*
 * 手指就位/离开检测 — 滞回状态机 + 自适应阈值。
 *
 * 设计：
 *   - 手指离开→就位：需连续 N 拍 raw_signal_present=1，防噪声误触发。
 *     就位后设置 contact_settle_samples 倒计数，期间抑制算法输出。
 *   - 手指就位→离开：需连续 M 拍 raw_signal_present=0，防短暂信号
 *     丢失导致状态抖动。离开后重新跟踪背景基线。
 *   - 任何状态切换都会重置测量输出、包络和高级指标。
 */
void app_measurement_update_finger_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->raw_signal_present = app_ppg_signal_is_raw_present(app);

  if (app->finger_present == 0U)
  {
    app->finger_off_confirm_count = 0U;

    if (app->raw_signal_present == 0U)
    {
      app->finger_on_confirm_count = 0U;
      app_ppg_signal_track_background_ir(app, &baseline_data);
      return;
    }

    if (app->finger_on_confirm_count < 0xFFU)
    {
      app->finger_on_confirm_count++;
    }

    if (app->finger_on_confirm_count >= APP_PPG_SIGNAL_FINGER_ON_CONFIRM_COUNT)
    {
      app->finger_present = 1U;
      app->finger_on_confirm_count = 0U;
      app_reset_measurement_outputs(app);
      app_ppg_signal_reset_envelope();
      app->contact_settle_samples = APP_CONTACT_SETTLE_SAMPLES;
      app->ir_pi_ac_ema = 0U;
      app->ir_pi_ac_ema_valid = 0U;
      app->last_beat_sample = 0U;
      app->raw_signal_present = 1U;
      app->report_due = 1U;
      app->display_refresh_requested = 1U;
    }

    return;
  }

  app->finger_on_confirm_count = 0U;

  if (app->raw_signal_present != 0U)
  {
    app->finger_off_confirm_count = 0U;
    return;
  }

  /* Track background IR during finger-off confirmation window so the baseline
   * converges toward the true no-finger level before finger is confirmed off. */
  app_ppg_signal_track_background_ir(app, &baseline_data);

  if (app->finger_off_confirm_count < 0xFFU)
  {
    app->finger_off_confirm_count++;
  }

  if (app->finger_off_confirm_count >= APP_PPG_SIGNAL_FINGER_OFF_CONFIRM_COUNT)
  {
    app->finger_present = 0U;
    app->finger_off_confirm_count = 0U;
    app->finger_on_confirm_count = 0U;

    /* 手指离开 → flush SD 缓冲 + f_sync */
    APP_DataLog_OnMeasurementStop();

    /* Background baseline has been tracked during the off-confirm window via
     * app_ppg_signal_track_background_ir (called when raw_signal_present==0).
     * Do not re-seed from a single IR sample here — it may still be elevated
     * from the finger removal transient and would raise the finger-on threshold. */
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    max30102_baseline_seed_tracking(&baseline_data,
                                    app->baseline_ir,
                                    APP_PPG_SIGNAL_REACQUIRE_NOISE_IR);
    app_ppg_signal_init_state(app);
    app_reset_measurement_outputs(app);
    app_ppg_signal_reset_envelope();
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }
}

/*
 * 测量处理流水线 — 每拍运行一次（100 Hz）。
 *
 * 处理顺序（按依赖关系排列）：
 *   1. 接触稳定倒计数（warm-up 到期时重置全部高级算法状态）
 *   2. SpO2 窗口更新 → 获取滤波后波形样本 → 送入 OLED 波形缓冲
 *   3. 信号质量 + PI 指标计算 (max30102_get_signal_metrics)
 *   4. Motion artifact 检测 (app_motion_update_artifact)
 *   5. Motion 期间冻结 BPM/SpO2/RR/HRV 输出
 *   6. Beat-based PI 更新（从已接受 beat 的 peak-to-trough 幅度计算）
 *   7. PPGA 脉搏脉冲检测 (app_ppg_pulse_update) → IBI → HRV
 *   8. SpO2 原始值计算 → 滤波器平滑 → 写 AppState
 *   9. BPM 抽头评估 → 时域峰值检测 + 自相关交叉验证 → 滤波器平滑
 *
 * 手指未就位时直接返回，不做处理。
 */
void app_measurement_process(AppState_t *app)
{
  MAX30102_SignalMetrics_t signal_metrics;
  MAX30102_PulseInfo_t pulse_info;
  uint8_t raw_bpm_valid = 0U;
  uint8_t raw_bpm_value = 0U;
  uint8_t acorr_bpm_valid = 0U;
  uint8_t acorr_bpm = 0U;
  uint8_t raw_spo2_valid = 0U;
  uint8_t raw_spo2_value = 0U;
  uint8_t signal_quality = 0U;
  int32_t red_waveform_sample = 0;
  int32_t ir_waveform_sample = 0;

  if ((app == NULL) || (app->finger_present == 0U))
  {
    return;
  }

  /* 接触稳定倒计数：手指刚放上时抑制 motion/stale/beat detector */
  if (app->contact_settle_samples > 0U)
  {
    app->contact_settle_samples--;
    if (app->contact_settle_samples == 0U)
    {
      /* Warm-up 结束：重置全部高级算法状态，避免接触瞬态污染 */
      app_ppg_pulse_reset();
      app_hrv_reset(app);
      app_rr_reset(app);
      app_bpm_filter_reset(app);
      app_spo2_filter_reset(app);
      app->ir_pi_ac_ema = 0U;
      app->ir_pi_ac_ema_valid = 0U;
      app->last_beat_sample = 0U;
      bpm_update_decimator = 0U;
    }
  }

  max30102_spo2_add_sample(&spo2_state, app->red_value, app->ir_value);
  if (max30102_spo2_get_latest_filtered(&spo2_state,
                                         &red_waveform_sample,
                                         &ir_waveform_sample) != 0U)
  {
    app_display_add_ir_sample(ir_waveform_sample);
    app_display_add_red_sample(red_waveform_sample);
  }

  if (max30102_get_signal_metrics(&spo2_state, &signal_metrics) != 0U)
  {
    if (max30102_calculate_signal_quality(&spo2_state, &signal_metrics, &signal_quality) != 0U)
    {
      app_oxy_status_update_from_metrics(app, &signal_metrics, signal_quality);
    }
    else
    {
      app_oxy_status_update_from_metrics(app, &signal_metrics, 0U);
    }

    app_motion_update_artifact(app, &signal_metrics, signal_quality);
  }
  else
  {
    app_oxy_status_clear_instant(app);
    app_motion_update_artifact(app, NULL, 0U);
    app_invalidate_advanced_outputs(app);
  }

  if (app->motion_artifact != 0U)
  {
    if (app->contact_settle_samples > 0U)
    {
      app->motion_artifact = 0U;
    }
    else
    {
      app->bpm_valid = 0U;
      app_spo2_filter_update_output(app, 0U, 0U);
      app_invalidate_advanced_outputs(app);
      return;
    }
  }

  if (app->contact_settle_samples == 0U)
  {
    if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
    {
      app->rr_valid = 0U;
    }
    else if (app->signal_quality < APP_RR_SIGNAL_QUALITY_MIN)
    {
      app->rr_valid = 0U;
    }

    if (app_hrv_is_peak_stale(spo2_state.total_samples, APP_ADVANCED_PULSE_STALE_SAMPLES) != 0U)
    {
      app_invalidate_advanced_outputs(app);
    }

    /* Beat-based PI：从已接受 beat 的 peak-to-trough 幅度 EMA 计算。
     * 无新 beat 时保持最近值，超过阈值标无效。 */
    if (app->ir_pi_ac_ema_valid != 0U)
    {
      uint32_t ir_dc = (uint32_t)(spo2_state.ir_sum / spo2_state.sample_count);
      if (ir_dc != 0U)
      {
        uint64_t pi = ((uint64_t)app->ir_pi_ac_ema * 1000ULL + (ir_dc / 2ULL)) / ir_dc;
        app->signal_ir_pi_x1000 = (pi > 0xFFFFULL) ? 0xFFFFU : (uint16_t)pi;
      }
    }

    if ((app->last_beat_sample != 0U) &&
        ((spo2_state.total_samples - app->last_beat_sample) > APP_PI_STALE_SAMPLES))
    {
      app->ir_pi_ac_ema_valid = 0U;
    }

    if (app_ppg_pulse_update(app, ir_waveform_sample, spo2_state.total_samples, &pulse_info) != 0U)
    {
      app_ppg_pulse_process_metrics(app, &pulse_info, spo2_state.total_samples);
      app_ptt_update_from_ppg_peak(app, pulse_info.latest_peak_sample, spo2_state.total_samples);
    }
  }

  raw_spo2_valid = max30102_calculate_spo2(&spo2_state, &raw_spo2_value);
  if ((raw_spo2_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2))
  {
    raw_spo2_valid = 0U;
  }
  if (app->contact_settle_samples == 0U)
  {
    app_spo2_filter_update_output(app, raw_spo2_valid, raw_spo2_value);
  }

  if (app->contact_settle_samples == 0U)
  {
    bpm_update_decimator++;
    if (bpm_update_decimator < APP_BPM_EVALUATE_INTERVAL_SAMPLES)
    {
      return;
    }

    bpm_update_decimator = 0U;
    raw_bpm_valid = max30102_calculate_bpm_with_pulse(&spo2_state, &raw_bpm_value, &pulse_info);

    if (app->signal_quality >= APP_SIGNAL_QUALITY_MIN_FOR_BPM)
    {
      acorr_bpm_valid = max30102_autocorr_bpm(&spo2_state, &acorr_bpm);
    }

    if ((raw_bpm_valid != 0U) && (acorr_bpm_valid != 0U))
    {
      uint8_t diff = (raw_bpm_value > acorr_bpm) ? (raw_bpm_value - acorr_bpm)
                                                 : (acorr_bpm - raw_bpm_value);
      if (diff <= APP_BPM_ACORR_BLEND_MAX_DIFF)
      {
        raw_bpm_value = (uint8_t)(((uint16_t)raw_bpm_value + (uint16_t)acorr_bpm * 3U + 2U) / 4U);
      }
    }
    else if (acorr_bpm_valid != 0U)
    {
      raw_bpm_valid = 1U;
      raw_bpm_value = acorr_bpm;
    }

    if ((raw_bpm_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM))
    {
      raw_bpm_valid = 0U;
      pulse_info.beat_valid = 0U;
    }

    (void)pulse_info.beat_valid;
    if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
    {
      app->rr_valid = 0U;
    }

    if (app_bpm_filter_update(app, raw_bpm_valid, raw_bpm_value) != 0U)
    {
      bpm_update_decimator = 0U;
    }
  }
}

/* 20 拍 (=200ms) 计时器：置位 report_due 和 display_refresh_requested。
 * 由主循环在每次采样后调用，驱动协议上报和 OLED 刷新。 */
void app_measurement_update_periodic_flags(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->refresh_div++;
  if (app->refresh_div < 20U)
  {
    return;
  }

  app->refresh_div = 0U;
  app->report_due = 1U;
  app->display_refresh_requested = 1U;
}

/* 上次恢复尝试的时间戳，用于限频。上电初始化为 0，避免首次被限频拦截。 */
/*
 * 统一限频恢复入口 — I2C 总线恢复 + MAX30102 重新初始化 + 算法输出重置。
 *
 * 限频：两次恢复尝试间隔 >= APP_SENSOR_RECOVERY_RETRY_MS。
 * 成功 → 清空错误计数、重置算法输出、重新播种基线、触发 OLED 刷新。
 * 失败 → 保持 error_streak 在阈值，由上层在下个周期重新触发。
 *
 * 同时被 ERROR 路径 (app_measurement_recover_sensor) 和
 * stale 看门狗 (app_measurement_service_sensor_watchdog) 调用。
 */
static void app_measurement_do_recovery(AppState_t *app)
{
  uint32_t now;

  if (app == NULL) return;

  now = HAL_GetTick();

  /* 限频 */
  if ((sensor_last_recovery_tick != 0UL) &&
      ((now - sensor_last_recovery_tick) < APP_SENSOR_RECOVERY_RETRY_MS))
  {
    return;
  }

  sensor_last_recovery_tick = now;

  app->sensor_health = (uint8_t)SENSOR_HEALTH_RECOVERING;
  app->display_refresh_requested = 1U;

  /* I2C bus recovery + MAX30102 re-init. */
  if (app_measurement_reinit_sensor() != HAL_OK)
  {
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    app->sensor_health = (uint8_t)SENSOR_HEALTH_INIT_FAIL;
    if (sensor_recovery_fail_count < 0xFFU)
    {
      sensor_recovery_fail_count++;
    }
    app->sensor_recovery_fail_count = sensor_recovery_fail_count;
    return;
  }

  /* 恢复成功 — 清理错误状态，保留 finger_present 和算法状态 */
  now = HAL_GetTick();
  sensor_recovery_fail_count = 0U;
  app->sensor_recovery_fail_count = 0U;
  app->sensor_error_streak = 0U;
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
  app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
  app->sensor_health = (uint8_t)SENSOR_HEALTH_OK;
  app->sensor_recover_count++;
  /* Give MAX30102 time to publish its first sample after reset. */
  app->sensor_last_sample_tick = now;
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  sample_debug_state.initialized = 0U;
  /* Enter short reacquire: keep finger_present so we don't show PLACE FINGER,
   * but set contact_settle to suppress algorithm output during re-stabilization. */
  app->contact_settle_samples = APP_CONTACT_SETTLE_SAMPLES;
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
}

/* Runtime recovery helper: clear I2C, then retry MAX30102 init. */
static HAL_StatusTypeDef app_measurement_reinit_sensor(void)
{
  HAL_StatusTypeDef init_status = HAL_ERROR;
  uint8_t attempt;

  for (attempt = 0U; attempt < APP_SENSOR_RECOVERY_INIT_ATTEMPTS; attempt++)
  {
    (void)MX_I2C1_RecoverBus();

    init_status = max30102_init();
    if (init_status == HAL_OK)
    {
      return HAL_OK;
    }

    HAL_Delay(20U);
  }

  return init_status;
}

/* Sensor ERROR-path recovery entry. */
void app_measurement_recover_sensor(AppState_t *app)
{
  if ((app == NULL) || (app->sensor_error_streak < APP_SENSOR_RECOVERY_ERROR_COUNT))
  {
    return;
  }

  app_measurement_do_recovery(app);
}

/*
 * MAX30102 无新样本看门狗。
 *
 * - >1s 无样本：置 SENSOR_HEALTH_STALE，OLED 可显示 WAIT 状态
 * - >3s 无样本：确认持久停顿后触发限频恢复
 * - recovery 失败：置 SENSOR_HEALTH_INIT_FAIL
 */
void app_measurement_service_sensor_watchdog(AppState_t *app)
{
  uint32_t now;
  uint8_t  sample_stale = 0U;

  if (app == NULL) return;

  /* 恢复期间不重复触发，等待恢复流程自行更新 health */
  if (app->sensor_health == (uint8_t)SENSOR_HEALTH_RECOVERING) return;

  now = HAL_GetTick();

  /* >1s 无新样本 → 快速标记 STALE，OLED 可见 */
  if ((app->sensor_last_sample_tick != 0UL) &&
      ((now - app->sensor_last_sample_tick) > APP_SENSOR_STALE_WARN_MS))
  {
    app->sensor_health = (uint8_t)SENSOR_HEALTH_STALE;
  }

  if ((app->sensor_last_sample_tick != 0UL) &&
      ((now - app->sensor_last_sample_tick) > APP_SENSOR_STALE_TIMEOUT_MS))
  {
    sample_stale = 1U;
  }

  if ((app->sensor_read_ok_count == 0UL) &&
      (now > (APP_SENSOR_STALE_TIMEOUT_MS + 500U)))
  {
    sample_stale = 1U;
  }

  if (sample_stale == 0U)
  {
    sensor_last_stale_probe_tick = 0UL;
    sensor_stale_probe_count = 0U;
    return;
  }

  /* OLED/SD work can delay the main loop. Confirm a persistent stall before
   * clearing MAX30102 FIFO and rebuilding the shared I2C bus. */
  if ((sensor_last_stale_probe_tick != 0UL) &&
      ((now - sensor_last_stale_probe_tick) < APP_SENSOR_STALE_CONFIRM_INTERVAL_MS))
  {
    return;
  }

  sensor_last_stale_probe_tick = now;
  if (sensor_stale_probe_count < 0xFFU)
  {
    sensor_stale_probe_count++;
  }

  if (sensor_stale_probe_count < APP_SENSOR_STALE_CONFIRM_COUNT)
  {
    return;
  }

  app->sensor_stale_count++;
  app_measurement_do_recovery(app);
}

/* 手指状态切换时重置全部测量输出：波形、SpO2 窗口、滤波器、motion、包络等。 */
static void app_reset_measurement_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_display_reset_waveforms();
  max30102_spo2_reset(&spo2_state);
  app_bpm_filter_reset(app);
  app_spo2_filter_reset(app);
  app_motion_reset(app);
  app->raw_signal_present = 0U;
  app->ir_signal_delta = 0U;
  app->ir_signal_span = 0U;
  app->red_signal_span = 0U;
  app->ir_pi_ac_ema = 0U;
  app->ir_pi_ac_ema_valid = 0U;
  app->last_beat_sample = 0U;
  app->contact_settle_samples = 0U;
  app_oxy_status_reset(app);
  app_reset_advanced_metrics(app);

  /* PTT 依赖 PPG 脉搏波峰与 ECG R 峰的时间差，手指状态切换后旧历史无效 */
  app_ptt_reset(app);
}

/* 信号中断/motion 时仅标无效，不清 buffer，中断结束可立即恢复。 */
static void app_invalidate_advanced_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_hrv_invalidate_outputs(app);
  app->rr_valid = 0U;
}

/* 完全重置高级指标：清空 HRV/RR ring buffer + PPGA 脉搏检测器。 */
static void app_reset_advanced_metrics(AppState_t *app)
{
  app_hrv_reset(app);
  app_rr_reset(app);
  app_ppg_pulse_reset();
}
