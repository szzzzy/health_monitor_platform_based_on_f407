/**
  ******************************************************************************
  * @file    app_measurement.c
  * @brief   测量流水线 — 传感器读取、手指检测、BPM/SpO2 算法调度
  *
  * 职责：
  *   1. MAX30102 FIFO 读取与原始数据解析
  *   2. 背景基线采集（上电无手指阶段）
  *   3. 手指就位/离开检测（滞回状态机 + 自适应阈值）
  *   4. 测量处理流水线调度（SpO2 窗口 → 信号质量 → 运动检测 → BPM）
  *   5. 传感器恢复（连续 I2C 错误超阈值后重建初始化）
  *   6. 200ms 周期上报/刷新标志管理
  *
  * 本模块不执行 OLED、物理 SD I/O 或 UART 发送；它更新 AppState，并把原始
  * 日志记录压入 RAM 环形缓冲。显示/上报由 Uitask 驱动，写卡由 SDtask 驱动。
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
#include "app_ppg_side_elgendi.h"
#include "app_ppg_sqi.h"
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
#define APP_SIGNAL_QUALITY_MIN_FOR_BPM    22U
/* 高级指标超时：无新脉搏超过此窗口则清零 HRV/RR */
#define APP_ADVANCED_PULSE_STALE_SAMPLES  (MAX30102_ALGO_SAMPLE_RATE_HZ * 8U)
/* 手指接触稳定倒计数：避免接触瞬态污染检测器 */
#define APP_CONTACT_SETTLE_SAMPLES        200U
/* 基于搏动的 PI 超时：无新搏动超过此窗口则标无效 */
#define APP_PI_STALE_SAMPLES              (MAX30102_ALGO_SAMPLE_RATE_HZ * 5U)
#define APP_PPG_BEAT_BPM_STALE_SAMPLES    (MAX30102_ALGO_SAMPLE_RATE_HZ * 3U)
#define APP_PPG_BPM_MIN_RESULT            35U
#define APP_PPG_BPM_MAX_RESULT            220U
#define APP_SPO2_BEAT_RATIO_HISTORY_SIZE  8U
#define APP_SPO2_BEAT_RATIO_MIN_COUNT     3U
#define APP_SPO2_BEAT_STALE_SAMPLES       (MAX30102_ALGO_SAMPLE_RATE_HZ * 8U)
#define APP_SPO2_BEAT_MIN_AC              30U
#define APP_SPO2_BEAT_RATIO_MIN_X1000     300U
#define APP_SPO2_BEAT_RATIO_MAX_X1000     3000U
#define APP_HR_FUSION_DIFF_BPM            18U
#define APP_HR_FUSION_CONFIRM_COUNT       4U
#define APP_HR_FUSION_MIN_SQ              35U
#define APP_SENSOR_STALE_TIMEOUT_MS       3000U
#define APP_SENSOR_STALE_CONFIRM_INTERVAL_MS 1000U
#define APP_SENSOR_STALE_CONFIRM_COUNT       3U
#define APP_OUTPUT_BPM_STALE_MS           3000U
#define APP_OUTPUT_SPO2_STALE_MS          8000U
#define APP_OUTPUT_PTT_STALE_MS           3000U
#define APP_STATUS_REPORT_PERIOD_MS        200U
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
static struct
{
  uint8_t valid;
  uint8_t bpm;
  uint32_t sample;
} ppg_beat_bpm_state;
static struct
{
  uint16_t ratio_x1000[APP_SPO2_BEAT_RATIO_HISTORY_SIZE];
  uint8_t count;
  uint8_t write_index;
  uint32_t sample;
} spo2_beat_state;
static uint8_t hr_fusion_mismatch_count;
/* 两次恢复之间留出时间，使 MAXtask 有机会先排空暂时积压的 FIFO。 */
static uint32_t sensor_last_recovery_tick = 0UL;
static uint32_t sensor_last_stale_probe_tick = 0UL;
static uint8_t  sensor_stale_probe_count = 0U;
static uint8_t  sensor_recovery_fail_count = 0U;
static uint8_t  measurement_time_initialized = 0U;
static uint8_t  measurement_last_sensor_health = 0U;
static uint32_t measurement_last_report_tick = 0UL;

static void app_reset_measurement_outputs(AppState_t *app);
static void app_invalidate_advanced_outputs(AppState_t *app);
static void app_reset_advanced_metrics(AppState_t *app);
static HAL_StatusTypeDef app_measurement_reinit_sensor(void);
static void app_measurement_reset_beat_states(void);
static uint8_t app_measurement_bpm_from_ibi(uint16_t ibi_ms, uint8_t *bpm_value);
static uint8_t app_measurement_ppg_beat_bpm_recent(uint32_t current_sample);
static uint8_t app_measurement_spo2_beat_recent(uint32_t current_sample);
static uint8_t app_measurement_spo2_update_from_beat(AppState_t *app,
                                                     const MAX30102_SpO2_t *state,
                                                     const MAX30102_PulseInfo_t *pulse,
                                                     uint8_t *spo2_value);
static void app_measurement_spo2_add_ratio(uint16_t ratio_x1000,
                                           uint32_t beat_sample);
static uint16_t app_measurement_spo2_select_ratio(void);
static uint8_t app_measurement_spo2_from_ratio(uint16_t ratio_x1000,
                                               uint8_t *spo2_value);
static void app_measurement_update_hr_fusion(AppState_t *app);
/* 将当前门控状态映射到上报用 reason code；不改变滤波器状态。 */
static uint8_t app_measurement_bpm_gate_reason(const AppState_t *app);
static uint8_t app_measurement_spo2_gate_reason(const AppState_t *app);
/* 输出年龄/陈旧标志维护，供 OLED/USART 判断旧值可信度。 */
static uint16_t app_measurement_elapsed_ms16(uint32_t now, uint32_t tick);
static void app_measurement_update_output_age(AppState_t *app);
static void app_measurement_reset_sampling_continuity(AppState_t *app,
                                                       uint8_t invalid_reason);

/**
 ******************************************************************************
 * @brief  初始化 AppState 中的测量子系统字段。
 * @param  app 指向共享应用状态（不能为 NULL）。
 * @return 无。
 * @note   委托给 app_ppg_signal_init_state 设置手指阈值
 *         和 PPG 信号包络默认值。
 ******************************************************************************
 */
void app_measurement_init_state(AppState_t *app)
{
  app_ppg_signal_init_state(app);
  app_ppg_sqi_reset(app);
  app_ppg_side_elgendi_reset(app);
}

/**
 ******************************************************************************
 * @brief  重置所有测量运行时状态。
 * @param  无。
 * @return 无。
 * @note   在上电时和每次手指状态切换时调用。重置
 *         基线统计、SpO2 窗口、运动检测器、PPG 包络、
 *         算法滤波器、波形显示和传感器看门狗定时器。
 ******************************************************************************
 */
void app_measurement_reset_runtime(void)
{
  max30102_baseline_reset(&baseline_data);
  max30102_spo2_reset(&spo2_state);
  (void)memset(&sample_debug_state, 0, sizeof(sample_debug_state));
  bpm_update_decimator = 0U;
  app_measurement_reset_beat_states();
  app_spo2_filter_reset(NULL);
  app_motion_reset(NULL);
  app_ppg_sqi_reset(NULL);
  app_ppg_side_elgendi_reset(NULL);
  app_ppg_signal_reset_envelope();
  app_reset_advanced_metrics(NULL);
  app_display_reset_waveforms();
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  sensor_recovery_fail_count = 0U;
  measurement_time_initialized = 0U;
  measurement_last_report_tick = 0UL;
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

/**
 ******************************************************************************
 * @brief  检查背景基线采集是否完成。
 * @param  无。
 * @return 如果基线样本数达到 APP_MEASUREMENT_BASELINE_SAMPLES 则返回 1，
 *         否则返回 0。
 * @note   在上电无手指阶段使用，用于控制进入正常
 *         测量操作的门控。
 ******************************************************************************
 */
uint8_t app_measurement_baseline_ready(void)
{
  return max30102_baseline_is_ready(&baseline_data, APP_MEASUREMENT_BASELINE_SAMPLES);
}

/**
 ******************************************************************************
 * @brief  获取基线采集进度百分比。
 * @param  无。
 * @return 进度 0--100，完成后固定为 100。
 * @note   在上电基线采集阶段显示在 OLED 状态页面上。
 *
 ******************************************************************************
 */
uint16_t app_measurement_get_baseline_progress_percent(void)
{
  if (baseline_data.sample_count >= APP_MEASUREMENT_BASELINE_SAMPLES)
  {
    return 100U;
  }

  return (uint16_t)((baseline_data.sample_count * 100U) / APP_MEASUREMENT_BASELINE_SAMPLES);
}

/**
 ******************************************************************************
 * @brief  获取背景基线采集的平均 IR 值。
 * @param  无。
 * @return 无手指基线阶段采集的平均 IR 原始值。
 * @note   基线完成后用于初始化运行时跟踪滤波器。
 ******************************************************************************
 */
uint32_t app_measurement_get_baseline_average(void)
{
  return max30102_baseline_get_average_ir(&baseline_data);
}

/**
 ******************************************************************************
 * @brief  获取背景基线采集的 IR 波动范围。
 * @param  无。
 * @return 采集的 IR 样本峰峰值范围（最大值 - 最小值）。
 * @note   用于评估环境噪声并判断基线是否
 *         足够稳定以实现可靠的手指检测。
 ******************************************************************************
 */
uint32_t app_measurement_get_baseline_range(void)
{
  return max30102_baseline_get_range_ir(&baseline_data);
}

/**
 ******************************************************************************
 * @brief  获取运行时跟踪的基线 IR 值。
 * @param  无。
 * @return 当前跟踪的 IR 基线，跟随环境光和
 *         温度漂移。
 * @note   与静态基线平均值不同，该值在手指离开期间持续更新
 *         以跟踪环境变化。
 ******************************************************************************
 */
uint32_t app_measurement_get_tracked_baseline(void)
{
  return max30102_baseline_get_tracked_ir(&baseline_data);
}

/**
 ******************************************************************************
 * @brief  使用上电采集结果初始化运行时基线跟踪器。
 * @param  baseline_ir 背景基线阶段的平均 IR 值。
 * @param  noise_ir    估计的噪声基底，用于设置跟踪死区。
 * @return 无。
 * @note   必须在基线采集完成后、正常手指检测开始前
 *         调用一次。
 ******************************************************************************
 */
void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir)
{
  max30102_baseline_seed_tracking(&baseline_data, baseline_ir, noise_ir);
}

/**
 ******************************************************************************
 * @brief  检查背景基线是否足够稳定。
 * @param  无。
 * @return 如果基线 IR 范围在 APP_MEASUREMENT_BASELINE_STABLE_RANGE 内则返回 1，
 *         否则返回 0。
 * @note   噪声较大的基线（波动范围大）可能导致手指检测误触发
 *         或漏检手指离开事件。
 ******************************************************************************
 */
uint8_t app_measurement_baseline_is_stable(void)
{
  return max30102_baseline_is_stable(&baseline_data, APP_MEASUREMENT_BASELINE_STABLE_RANGE);
}

/*
 * 共享后处理：对已解析的单个样本执行变化追踪、基线更新、
 * 日志样本入队和传感器健康标记。由单样本路径和批量路径共用。
 */
static void app_measurement_process_parsed_sample(AppState_t *app,
                                                   uint32_t red_val,
                                                   uint32_t ir_val,
                                                   uint32_t sample_tick)
{
  if (sample_debug_state.initialized != 0U)
  {
    if ((sample_debug_state.red_value != red_val) ||
        (sample_debug_state.ir_value != ir_val))
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

  sample_debug_state.red_value = red_val;
  sample_debug_state.ir_value = ir_val;
  app->red_value = red_val;
  app->ir_value  = ir_val;
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
  app_ppg_signal_update_activity(app);
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
  app->sensor_read_ok_count++;
  app->sensor_error_streak = 0U;
  app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
  app->sensor_last_sample_tick = sample_tick;
  app->sensor_last_ok_tick = sample_tick;
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  app->sensor_health = (uint8_t)SENSOR_HEALTH_OK;

  /* 实时路径只记录测量相关样本；无手指背景不入日志，避免插卡后
   * 空闲状态自动触发 FatFs 挂载/打开。 */
  if ((app->finger_present != 0U) || (app->contact_settle_samples > 0U))
  {
    uint8_t log_flags = 0U;
    if (app->finger_present != 0U) { log_flags |= 0x01U; }
    if (app->contact_settle_samples > 0U) { log_flags |= 0x02U; }
    if (app->sensor_fifo_overflow_count > 0U) { log_flags |= 0x04U; }
    APP_DataLog_PushSample(sample_tick,
                           red_val, ir_val,
                           (int16_t)app->ecg_filtered,
                           log_flags);
  }
}

/*
 * 单样本传感器读取入口 — 从 MAX30102 FIFO 读取一个 RED/IR 样本。
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
    uint32_t red_val, ir_val;
    max30102_parse_spo2_sample(fifo_buf, &red_val, &ir_val);
    app_measurement_process_parsed_sample(app, red_val, ir_val, HAL_GetTick());
    return APP_MEASUREMENT_READ_OK;
  }

  if (read_status == HAL_BUSY)
  {
    app->sensor_read_busy_count++;
    /* 忙 + I2C 故障 → 返回错误，走阈值恢复路径 */
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
    /* 忙 + I2C 就绪 → FIFO 暂时空或溢出清空后的等待，保留等待 */
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

/**
 ******************************************************************************
 * @brief  根据当前噪声更新自适应手指检测阈值。
 * @param  app 指向共享应用状态。
 * @return 无。
 * @note   委托给 app_ppg_signal_update_adaptive_thresholds，
 *         该函数根据测量的背景 IR 噪声水平调整
 *         手指就位/离开差值阈值。
 ******************************************************************************
 */
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

    if (app->raw_signal_present != 0U)
    {
      /* 积分：每拍原始信号存在时递增计数器。 */
      if (app->finger_on_confirm_count < APP_PPG_SIGNAL_FINGER_ON_CONFIRM_COUNT)
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
    }
    else
    {
      /* 无信号时递减 — 容忍 1-2 拍抖动而非立即重置。 */
      if (app->finger_on_confirm_count > 0U)
      {
        app->finger_on_confirm_count--;
      }

      /* 仅在信号明显低于阈值时跟踪背景 IR。
       * 接近阈值的信号 (ir_signal_delta >= on_delta/2) 被怀疑为手指接近
       * — 冻结基线以避免污染。 */
      if (app->ir_signal_delta < (app->adaptive_finger_on_delta / 2U))
      {
        app_ppg_signal_track_background_ir(app, &baseline_data);
      }
    }

    return;
  }

  app->finger_on_confirm_count = 0U;

  if (app->raw_signal_present != 0U)
  {
    app->finger_off_confirm_count = 0U;
    return;
  }

  /* 在手指离开确认窗口期间跟踪背景 IR，使基线
   * 在手指确认为离开之前收敛到真实的无手指水平。 */
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

    /* 手指离开 → 设延迟刷新标志（内部 O(1)），由后台在安全窗口分片执行 */
    APP_DataLog_OnMeasurementStop();

    /* 背景基线已在离开确认窗口期间通过
     * app_ppg_signal_track_background_ir（在 raw_signal_present==0 时调用）跟踪。
     * 不要在此处用单个 IR 样本重新播种 —— 它可能仍因手指移除瞬态
     * 而偏高，会抬高手指就位阈值。 */
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
 *   4. PPG SQI / 运动伪影检测，先解释质量再门控输出
 *   5. 运动期间冻结 BPM/SpO2/RR/HRV 输出
 *   6. 基于搏动的 PI 更新（从已接受搏动的峰谷幅度计算）
 *   7. 侧路 Elgendi 检测器 A/B 统计（只诊断，不写正式输出）
 *   8. PPG 主脉搏检测 (app_ppg_pulse_update) → IBI → HRV/PTT
 *   9. SpO2 原始值计算 → 滤波器平滑 → 写 AppState
 *  10. BPM 抽头评估 → 时域峰值检测 + 自相关交叉验证 → 滤波器平滑
 *
 * 手指未就位时直接返回，不做处理。所有 valid/reason/age 字段通过
 * app_measurement_update_output_age() 在周期节拍上补齐解释语义。
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
  uint8_t beat_spo2_attempted = 0U;
  uint8_t beat_spo2_valid = 0U;
  uint8_t beat_spo2_value = 0U;
  uint8_t signal_quality = 0U;
  uint8_t filtered_valid = 0U;
  int32_t red_waveform_sample = 0;
  int32_t ir_waveform_sample = 0;
  uint32_t current_sample;

  if (app == NULL)
  {
    return;
  }

  if (app->finger_present == 0U)
  {
    /* 安全：contact_settle_samples 仅在手指确认就位时才能 >0。
     * 如果错误地设置了（手指离开时发生溢出/恢复），清除它
     * 以便 finger_on_confirm_count 能在下次放置时重新计数。 */
    if (app->contact_settle_samples > 0U)
    {
      app->contact_settle_samples = 0U;
    }
    return;
  }

  /* 接触稳定倒计数：手指刚放上时抑制运动/停滞/拍检测器 */
  if (app->contact_settle_samples > 0U)
  {
    app->contact_settle_samples--;
    if (app->contact_settle_samples == 0U)
    {
      /* 预热结束：重置全部高级算法状态，避免接触瞬态污染 */
      app_ppg_pulse_reset();
      app_hrv_reset(app);
      app_rr_reset(app);
      app_bpm_filter_reset(app);
      app_spo2_filter_reset(app);
      app->ir_pi_ac_ema = 0U;
      app->ir_pi_ac_ema_valid = 0U;
      app->last_beat_sample = 0U;
      bpm_update_decimator = 0U;
      app_measurement_reset_beat_states();
      app_ppg_sqi_reset(app);
      app_ppg_side_elgendi_reset(app);
    }
  }

  max30102_spo2_add_sample(&spo2_state, app->red_value, app->ir_value);
  current_sample = spo2_state.total_samples - 1U;
  if (max30102_spo2_get_latest_filtered(&spo2_state,
                                         &red_waveform_sample,
                                         &ir_waveform_sample) != 0U)
  {
    filtered_valid = 1U;
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
    /* SQI 依赖运动标志和 PPG 窗口指标；先算 side score，再只向下钳制 signal_quality。 */
    app_ppg_sqi_update_window(app, &signal_metrics, signal_quality);
    app_ppg_sqi_apply_quality_gate(app);
    app->ppg_last_gate_flags = app->ppg_sqi_flags;
  }
  else
  {
    app_oxy_status_clear_instant(app);
    app_motion_update_artifact(app, NULL, 0U);
    /* 无有效窗口时清除 SQI 输出，避免旧窗口的 flags 延续到 PLACE FINGER/等待阶段。 */
    app_ppg_sqi_update_window(app, NULL, 0U);
    app->ppg_last_gate_flags = 0U;
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
      app->bpm_invalid_reason = APP_OUTPUT_REASON_MOTION;
      app->spo2_invalid_reason = APP_OUTPUT_REASON_MOTION;
      app_spo2_filter_update_output(app, 0U, 0U);
      app_invalidate_advanced_outputs(app);
      app_measurement_reset_beat_states();
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

    if (app_hrv_is_peak_stale(current_sample, APP_ADVANCED_PULSE_STALE_SAMPLES) != 0U)
    {
      app_invalidate_advanced_outputs(app);
    }

    /* 基于搏动的 PI：从已接受搏动的峰谷幅度 EMA 计算。
     * 无新搏动时保持最近值，超过阈值标无效。 */
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
        ((current_sample - app->last_beat_sample) > APP_PI_STALE_SAMPLES))
    {
      app->ir_pi_ac_ema_valid = 0U;
    }

    /* 侧路检测器只读同一条 IR AC 波形，输出 ppg_side_* A/B 诊断计数。 */
    if (filtered_valid != 0U)
    {
      app_ppg_side_elgendi_update(app,
                                  ir_waveform_sample,
                                  current_sample);
    }

    if (app_ppg_pulse_update(app, ir_waveform_sample, current_sample, &pulse_info) != 0U)
    {
      if (app_ppg_pulse_process_metrics(app, &pulse_info, current_sample) != 0U)
      {
        uint8_t beat_bpm;

        /* 已被 PPG 状态机接受的 beat 进入 SQI 短历史，用于后续 IBI/幅度稳定性门控。 */
        app_ppg_sqi_note_accepted_beat(app,
                                       pulse_info.latest_ibi_ms,
                                       pulse_info.beat_amplitude);
        app_ppg_side_elgendi_note_current_peak(app,
                                               pulse_info.latest_peak_sample,
                                               pulse_info.latest_ibi_ms);
        app->ppg_output_sample = pulse_info.latest_peak_sample;

        if (app_measurement_bpm_from_ibi(pulse_info.latest_ibi_ms, &beat_bpm) != 0U)
        {
          ppg_beat_bpm_state.valid = 1U;
          ppg_beat_bpm_state.bpm = beat_bpm;
          ppg_beat_bpm_state.sample = pulse_info.latest_peak_sample;
          (void)app_bpm_filter_update(app, 1U, beat_bpm);
          app_measurement_update_hr_fusion(app);
        }

        beat_spo2_attempted = 1U;
        beat_spo2_valid = app_measurement_spo2_update_from_beat(app,
                                                                &spo2_state,
                                                                &pulse_info,
                                                                &beat_spo2_value);
        if (beat_spo2_valid != 0U)
        {
          app_spo2_filter_update_output(app, 1U, beat_spo2_value);
        }

        app_ptt_update_from_ppg_peak(app, pulse_info.latest_peak_sample, spo2_state.total_samples);
      }
    }
  }

  if (beat_spo2_valid == 0U)
  {
    raw_spo2_valid = max30102_calculate_spo2(&spo2_state, &raw_spo2_value);
    /* SpO2 对低灌注、运动和平衡异常敏感，SQI 不通过时丢弃本窗口输出。 */
    if ((raw_spo2_valid != 0U) &&
        ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2) ||
         (app_ppg_sqi_allows_spo2(app) == 0U)))
    {
      raw_spo2_valid = 0U;
      app->spo2_invalid_reason = app_measurement_spo2_gate_reason(app);
    }
    if (app->contact_settle_samples == 0U)
    {
      if ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2) ||
          (app_ppg_sqi_allows_spo2(app) == 0U) ||
          ((beat_spo2_attempted != 0U) &&
           (app_measurement_spo2_beat_recent(current_sample) != 0U)))
      {
        app->spo2_invalid_reason = app_measurement_spo2_gate_reason(app);
        app_spo2_filter_update_output(app, 0U, 0U);
      }
      else if (app_measurement_spo2_beat_recent(current_sample) == 0U)
      {
        app_spo2_filter_update_output(app, raw_spo2_valid, raw_spo2_value);
      }
    }
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

    /* HR/BPM 门控比 SpO2 宽松，但仍屏蔸运动期和接触稳定期的误检 beat。 */
    if ((app->signal_quality >= APP_SIGNAL_QUALITY_MIN_FOR_BPM) &&
        (app_ppg_sqi_allows_hr(app) != 0U))
    {
      acorr_bpm_valid = max30102_autocorr_bpm(&spo2_state, &acorr_bpm);
    }

    if (app_measurement_ppg_beat_bpm_recent(current_sample) != 0U)
    {
      if ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM) ||
          (app_ppg_sqi_allows_hr(app) == 0U))
      {
        app->bpm_invalid_reason = app_measurement_bpm_gate_reason(app);
        (void)app_bpm_filter_update(app, 0U, 0U);
        app->rr_valid = 0U;
      }
      app_measurement_update_hr_fusion(app);
      return;
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

    if ((raw_bpm_valid != 0U) &&
        ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM) ||
         (app_ppg_sqi_allows_hr(app) == 0U)))
    {
      raw_bpm_valid = 0U;
      pulse_info.beat_valid = 0U;
      app->bpm_invalid_reason = app_measurement_bpm_gate_reason(app);
    }

    (void)pulse_info.beat_valid;
    if ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM) ||
        (app_ppg_sqi_allows_hr(app) == 0U))
    {
      app->bpm_invalid_reason = app_measurement_bpm_gate_reason(app);
      app->rr_valid = 0U;
    }

    if (app_bpm_filter_update(app, raw_bpm_valid, raw_bpm_value) != 0U)
    {
      bpm_update_decimator = 0U;
    }
    app_measurement_update_hr_fusion(app);
  }
}

/**
 ******************************************************************************
 * @brief  按 200 ms 运行时间更新周期性上报与显示请求标志。
 * @param  app 指向共享应用状态。
 * @return 无。
 * @note   MAXtask 每个服务周期调用。函数使用 HAL Tick 判断绝对时间，因此
 *         FIFO 批量大小或任务偶发延迟不会累计改变约 5 Hz 的请求节拍。
 ******************************************************************************
 */
void app_measurement_service_time(AppState_t *app)
{
  uint32_t now;

  if (app == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  app_measurement_update_output_age(app);

  if (measurement_time_initialized == 0U)
  {
    measurement_time_initialized = 1U;
    measurement_last_report_tick = now;
    measurement_last_sensor_health = app->sensor_health;
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
    return;
  }

  if (app->sensor_health != measurement_last_sensor_health)
  {
    measurement_last_sensor_health = app->sensor_health;
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }

  if ((now - measurement_last_report_tick) >= APP_STATUS_REPORT_PERIOD_MS)
  {
    measurement_last_report_tick = now;
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }
}

static void app_measurement_reset_beat_states(void)
{
  (void)memset(&ppg_beat_bpm_state, 0, sizeof(ppg_beat_bpm_state));
  (void)memset(&spo2_beat_state, 0, sizeof(spo2_beat_state));
  hr_fusion_mismatch_count = 0U;
}

static uint8_t app_measurement_bpm_from_ibi(uint16_t ibi_ms, uint8_t *bpm_value)
{
  uint32_t bpm;

  if ((bpm_value == NULL) || (ibi_ms == 0U))
  {
    return 0U;
  }

  bpm = (60000UL + ((uint32_t)ibi_ms / 2UL)) / (uint32_t)ibi_ms;
  if ((bpm < APP_PPG_BPM_MIN_RESULT) || (bpm > APP_PPG_BPM_MAX_RESULT))
  {
    return 0U;
  }

  *bpm_value = (uint8_t)bpm;
  return 1U;
}

static uint8_t app_measurement_ppg_beat_bpm_recent(uint32_t current_sample)
{
  if (ppg_beat_bpm_state.valid == 0U)
  {
    return 0U;
  }

  if (current_sample < ppg_beat_bpm_state.sample)
  {
    return 0U;
  }

  return ((current_sample - ppg_beat_bpm_state.sample) <=
          APP_PPG_BEAT_BPM_STALE_SAMPLES) ? 1U : 0U;
}

static uint8_t app_measurement_spo2_beat_recent(uint32_t current_sample)
{
  if (spo2_beat_state.count < APP_SPO2_BEAT_RATIO_MIN_COUNT)
  {
    return 0U;
  }

  if (current_sample < spo2_beat_state.sample)
  {
    return 0U;
  }

  return ((current_sample - spo2_beat_state.sample) <=
          APP_SPO2_BEAT_STALE_SAMPLES) ? 1U : 0U;
}

static uint8_t app_measurement_spo2_update_from_beat(AppState_t *app,
                                                     const MAX30102_SpO2_t *state,
                                                     const MAX30102_PulseInfo_t *pulse,
                                                     uint8_t *spo2_value)
{
  uint32_t peak_sample;
  uint32_t start_sample;
  uint32_t oldest_sample;
  uint32_t sample;
  uint16_t start_index;
  uint16_t sample_index;
  uint32_t red_min = 0xFFFFFFFFUL;
  uint32_t red_max = 0U;
  uint32_t ir_min = 0xFFFFFFFFUL;
  uint32_t ir_max = 0U;
  uint64_t red_sum = 0ULL;
  uint64_t ir_sum = 0ULL;
  uint32_t count = 0U;
  uint32_t red_dc;
  uint32_t ir_dc;
  uint32_t red_ac;
  uint32_t ir_ac;
  uint32_t ratio_x1000;
  uint64_t denominator;

  if ((app == NULL) || (state == NULL) || (pulse == NULL) ||
      (spo2_value == NULL) || (pulse->beat_valid == 0U))
  {
    return 0U;
  }

  if ((app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2) ||
      (app_ppg_sqi_allows_spo2(app) == 0U) ||
      (app->motion_artifact != 0U) ||
      (app->spo2_balance_status != APP_OXY_BALANCE_OK))
  {
    return 0U;
  }

  if ((state->sample_count == 0U) || (state->total_samples == 0U) ||
      (pulse->interval_samples == 0U))
  {
    return 0U;
  }

  peak_sample = pulse->latest_peak_sample;
  if (peak_sample >= state->total_samples)
  {
    peak_sample = state->total_samples - 1U;
  }
  if (peak_sample < pulse->interval_samples)
  {
    return 0U;
  }

  start_sample = peak_sample - (uint32_t)pulse->interval_samples;
  oldest_sample = state->total_samples - state->sample_count;
  if (start_sample < oldest_sample)
  {
    return 0U;
  }

  start_index = (state->sample_count < MAX30102_SPO2_WINDOW_SIZE) ? 0U :
                                                                  state->write_index;
  for (sample = start_sample; sample <= peak_sample; sample++)
  {
    uint32_t red;
    uint32_t ir;
    uint32_t offset = sample - oldest_sample;

    if (offset >= state->sample_count)
    {
      return 0U;
    }

    sample_index = (uint16_t)((start_index + (uint16_t)offset) %
                              MAX30102_SPO2_WINDOW_SIZE);
    red = state->red_samples[sample_index];
    ir = state->ir_samples[sample_index];

    if (red < red_min) { red_min = red; }
    if (red > red_max) { red_max = red; }
    if (ir < ir_min) { ir_min = ir; }
    if (ir > ir_max) { ir_max = ir; }
    red_sum += red;
    ir_sum += ir;
    count++;
  }

  if (count == 0U)
  {
    return 0U;
  }

  red_dc = (uint32_t)((red_sum + (count / 2U)) / count);
  ir_dc = (uint32_t)((ir_sum + (count / 2U)) / count);
  red_ac = red_max - red_min;
  ir_ac = ir_max - ir_min;

  if ((red_dc == 0U) || (ir_dc == 0U) ||
      (red_ac < APP_SPO2_BEAT_MIN_AC) ||
      (ir_ac < APP_SPO2_BEAT_MIN_AC))
  {
    return 0U;
  }

  denominator = (uint64_t)ir_ac * (uint64_t)red_dc;
  if (denominator == 0ULL)
  {
    return 0U;
  }

  ratio_x1000 = (uint32_t)((((uint64_t)red_ac * (uint64_t)ir_dc * 1000ULL) +
                            (denominator / 2ULL)) / denominator);
  if ((ratio_x1000 < APP_SPO2_BEAT_RATIO_MIN_X1000) ||
      (ratio_x1000 > APP_SPO2_BEAT_RATIO_MAX_X1000))
  {
    return 0U;
  }

  app_measurement_spo2_add_ratio((uint16_t)ratio_x1000, peak_sample);
  if (spo2_beat_state.count < APP_SPO2_BEAT_RATIO_MIN_COUNT)
  {
    return 0U;
  }

  return app_measurement_spo2_from_ratio(app_measurement_spo2_select_ratio(),
                                         spo2_value);
}

static void app_measurement_spo2_add_ratio(uint16_t ratio_x1000,
                                           uint32_t beat_sample)
{
  spo2_beat_state.ratio_x1000[spo2_beat_state.write_index] = ratio_x1000;
  spo2_beat_state.write_index =
      (uint8_t)((spo2_beat_state.write_index + 1U) %
                APP_SPO2_BEAT_RATIO_HISTORY_SIZE);
  if (spo2_beat_state.count < APP_SPO2_BEAT_RATIO_HISTORY_SIZE)
  {
    spo2_beat_state.count++;
  }
  spo2_beat_state.sample = beat_sample;
}

static uint16_t app_measurement_spo2_select_ratio(void)
{
  uint16_t sorted[APP_SPO2_BEAT_RATIO_HISTORY_SIZE];
  uint16_t tmp;
  uint32_t sum = 0U;
  uint8_t i;
  uint8_t j;
  uint8_t n;

  n = spo2_beat_state.count;
  if (n == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < n; i++)
  {
    sorted[i] = spo2_beat_state.ratio_x1000[i];
  }

  for (i = 1U; i < n; i++)
  {
    tmp = sorted[i];
    j = i;
    while ((j > 0U) && (sorted[j - 1U] > tmp))
    {
      sorted[j] = sorted[j - 1U];
      j--;
    }
    sorted[j] = tmp;
  }

  if (n >= 5U)
  {
    for (i = 1U; i < (uint8_t)(n - 1U); i++)
    {
      sum += sorted[i];
    }
    return (uint16_t)((sum + ((uint32_t)(n - 2U) / 2U)) /
                      (uint32_t)(n - 2U));
  }

  return sorted[n / 2U];
}

static uint8_t app_measurement_spo2_from_ratio(uint16_t ratio_x1000,
                                               uint8_t *spo2_value)
{
  int32_t spo2_milli;
  int64_t ratio_square_term;

  if ((spo2_value == NULL) || (ratio_x1000 == 0U))
  {
    return 0U;
  }

  ratio_square_term = ((int64_t)ratio_x1000 * (int64_t)ratio_x1000 + 500LL) / 1000LL;
  spo2_milli = (int32_t)(94845LL + ((30354LL * (int64_t)ratio_x1000 + 500LL) / 1000LL) -
                         ((45060LL * ratio_square_term + 500LL) / 1000LL));
  if ((spo2_milli < 70000L) || (spo2_milli > 100000L))
  {
    return 0U;
  }

  *spo2_value = (uint8_t)((spo2_milli + 500L) / 1000L);
  return 1U;
}

static void app_measurement_update_hr_fusion(AppState_t *app)
{
  uint8_t diff;

  if ((app == NULL) || (app->ecg_valid == 0U) || (app->bpm_valid == 0U) ||
      (app->ecg_hr == 0U) || (app->bpm_value == 0U))
  {
    hr_fusion_mismatch_count = 0U;
    return;
  }

  diff = (app->ecg_hr > app->bpm_value) ? (uint8_t)(app->ecg_hr - app->bpm_value) :
                                         (uint8_t)(app->bpm_value - app->ecg_hr);
  if (diff <= APP_HR_FUSION_DIFF_BPM)
  {
    hr_fusion_mismatch_count = 0U;
    return;
  }

  if (hr_fusion_mismatch_count < APP_HR_FUSION_CONFIRM_COUNT)
  {
    hr_fusion_mismatch_count++;
  }
  if (hr_fusion_mismatch_count < APP_HR_FUSION_CONFIRM_COUNT)
  {
    return;
  }

  if ((app->motion_artifact != 0U) ||
      ((app->signal_quality < APP_HR_FUSION_MIN_SQ) &&
       (app->ecg_signal_quality >= APP_HR_FUSION_MIN_SQ)))
  {
    app->bpm_valid = 0U;
  }
  else if ((app->ecg_signal_quality < APP_HR_FUSION_MIN_SQ) &&
           (app->signal_quality >= APP_HR_FUSION_MIN_SQ))
  {
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
  }
  else if (app->ecg_signal_quality > (uint8_t)(app->signal_quality + 15U))
  {
    app->bpm_valid = 0U;
  }
  else if (app->signal_quality > (uint8_t)(app->ecg_signal_quality + 15U))
  {
    app->ecg_valid = 0U;
    app->ptt_valid = 0U;
  }
}

/**
 *******************************************************************************
 * @brief  将当前 BPM 门控状态映射为统一输出原因码。
 * @param  app AppState 指针。
 * @return APP_OUTPUT_REASON_* 原因码。
 * @note   只解释最近一次 BPM 无效/陈旧的原因，不推进 BPM 滤波器。
 *******************************************************************************
 */
static uint8_t app_measurement_bpm_gate_reason(const AppState_t *app)
{
  if (app == NULL)
  {
    return APP_OUTPUT_REASON_STALE;
  }
  if (app->finger_present == 0U)
  {
    return APP_OUTPUT_REASON_NO_FINGER;
  }
  if (app->contact_settle_samples > 0U)
  {
    return APP_OUTPUT_REASON_CONTACT;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_MOTION) != 0U)
  {
    return APP_OUTPUT_REASON_MOTION;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_TRANSITION) != 0U)
  {
    return APP_OUTPUT_REASON_CONTACT;
  }
  if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    return APP_OUTPUT_REASON_LOW_SQ;
  }

  return APP_OUTPUT_REASON_STALE;
}

/**
 *******************************************************************************
 * @brief  将当前 SpO2 门控状态映射为统一输出原因码。
 * @param  app AppState 指针。
 * @return APP_OUTPUT_REASON_* 原因码。
 * @note   SpO2 对低灌注、RED/IR 平衡和接触变化更敏感，因此优先
 *         映射 SQI flags，再回退到 signal_quality / STALE。
 *******************************************************************************
 */
static uint8_t app_measurement_spo2_gate_reason(const AppState_t *app)
{
  if (app == NULL)
  {
    return APP_OUTPUT_REASON_STALE;
  }
  if (app->finger_present == 0U)
  {
    return APP_OUTPUT_REASON_NO_FINGER;
  }
  if (app->contact_settle_samples > 0U)
  {
    return APP_OUTPUT_REASON_CONTACT;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_LOW_PERFUSION) != 0U)
  {
    return APP_OUTPUT_REASON_LOW_PERFUSION;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_MOTION) != 0U)
  {
    return APP_OUTPUT_REASON_MOTION;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_BALANCE) != 0U)
  {
    return APP_OUTPUT_REASON_BALANCE;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_TRANSITION) != 0U)
  {
    return APP_OUTPUT_REASON_CONTACT;
  }
  if ((app->ppg_sqi_flags & APP_PPG_SQI_FLAG_BEAT_UNSTABLE) != 0U)
  {
    return APP_OUTPUT_REASON_BEAT_UNSTABLE;
  }
  if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2)
  {
    return APP_OUTPUT_REASON_LOW_SQ;
  }

  return APP_OUTPUT_REASON_STALE;
}

/**
 *******************************************************************************
 * @brief  计算当前 tick 与上次更新时间的年龄，并钳位到 uint16。
 * @param  now  当前 HAL_GetTick()。
 * @param  tick 上次有效输出的 tick；0 表示从未更新。
 * @return 年龄 ms，最大 0xFFFF；tick 为 0 时返回 0xFFFF。
 *******************************************************************************
 */
static uint16_t app_measurement_elapsed_ms16(uint32_t now, uint32_t tick)
{
  uint32_t age;

  if (tick == 0UL)
  {
    return 0xFFFFU;
  }

  /* 无符号减法按模 2^32 运算，可在 HAL Tick 单次回绕后保持正确的短时差。 */
  age = now - tick;
  return (age > 0xFFFFUL) ? 0xFFFFU : (uint16_t)age;
}

/**
 *******************************************************************************
 * @brief  维护 BPM/SpO2/PTT 输出年龄和陈旧标志。
 * @param  app AppState 指针。
 * @note   age 字段回答“这个数值多久没刷新”；reason 字段回答
 *         “为什么当前无效”。PTT 的 ptt_match_age_ms 保留为匹配瞬间
 *         PPG 峰的延迟，不在此处覆盖。
 *******************************************************************************
 */
static void app_measurement_update_output_age(AppState_t *app)
{
  uint32_t now;
  uint16_t ptt_age_ms;

  if (app == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  app->output_stale_flags = 0U;

  app->bpm_age_ms = app_measurement_elapsed_ms16(now, app->bpm_last_update_tick);
  app->spo2_age_ms = app_measurement_elapsed_ms16(now, app->spo2_last_update_tick);
  ptt_age_ms = app_measurement_elapsed_ms16(now, app->ptt_last_update_tick);

  if ((app->bpm_last_update_tick == 0UL) ||
      ((uint32_t)app->bpm_age_ms > APP_OUTPUT_BPM_STALE_MS))
  {
    app->output_stale_flags |= APP_OUTPUT_STALE_BPM;
    app->bpm_valid = 0U;
    app->bpm_invalid_reason = app_measurement_bpm_gate_reason(app);
  }

  if ((app->spo2_last_update_tick == 0UL) ||
      ((uint32_t)app->spo2_age_ms > APP_OUTPUT_SPO2_STALE_MS))
  {
    app->output_stale_flags |= APP_OUTPUT_STALE_SPO2;
    app->spo2_valid = 0U;
    app->spo2_invalid_reason = app_measurement_spo2_gate_reason(app);
  }

  if ((app->ptt_last_update_tick == 0UL) ||
      ((uint32_t)ptt_age_ms > APP_OUTPUT_PTT_STALE_MS))
  {
    app->output_stale_flags |= APP_OUTPUT_STALE_PTT;
    app->ptt_valid = 0U;
    if (app->finger_present == 0U)
    {
      app->ptt_invalid_reason = APP_OUTPUT_REASON_NO_FINGER;
    }
    else if (app->contact_settle_samples > 0U)
    {
      app->ptt_invalid_reason = APP_OUTPUT_REASON_CONTACT;
    }
    else if ((app->ecg_valid == 0U) || (app->ecg_lead_off != 0U))
    {
      app->ptt_invalid_reason = APP_OUTPUT_REASON_ECG;
    }
    else
    {
      app->ptt_invalid_reason = APP_OUTPUT_REASON_STALE;
    }
  }
}

static void app_measurement_reset_sampling_continuity(AppState_t *app,
                                                       uint8_t invalid_reason)
{
  if (app == NULL)
  {
    return;
  }

  max30102_spo2_reset(&spo2_state);
  app_ppg_pulse_reset();
  app_hrv_reset(app);
  app_rr_reset(app);
  app_bpm_filter_reset(app);
  app_spo2_filter_reset(app);
  app_motion_reset(app);
  app_ppg_sqi_reset(app);
  app_ppg_side_elgendi_reset(app);
  app_ptt_reset(app);
  app_measurement_reset_beat_states();
  app_display_reset_waveforms();
  bpm_update_decimator = 0U;

  app->bpm_valid = 0U;
  app->spo2_valid = 0U;
  app->ptt_valid = 0U;
  app->ibi_valid = 0U;
  app->hrv_valid = 0U;
  app->rr_valid = 0U;
  app->bpm_invalid_reason = invalid_reason;
  app->spo2_invalid_reason = invalid_reason;
  app->ptt_invalid_reason = invalid_reason;
  app->bpm_last_update_tick = 0UL;
  app->spo2_last_update_tick = 0UL;
  app->ptt_last_update_tick = 0UL;
  app->bpm_age_ms = 0xFFFFU;
  app->spo2_age_ms = 0xFFFFU;
  app->ptt_match_age_ms = 0xFFFFU;
  app->output_stale_flags = APP_OUTPUT_STALE_BPM |
                            APP_OUTPUT_STALE_SPO2 |
                            APP_OUTPUT_STALE_PTT;
  app->ir_pi_ac_ema = 0U;
  app->ir_pi_ac_ema_valid = 0U;
  app->last_beat_sample = 0U;
  app->ppg_output_sample = 0U;

  if (app->finger_present != 0U)
  {
    app->contact_settle_samples = APP_CONTACT_SETTLE_SAMPLES;
  }

  app->report_due = 1U;
  app->display_refresh_requested = 1U;
}

/*
 * 批量排空 FIFO — 一次 I2C 突发读所有当前可用样本，再逐个推进测量管道。
 * 一次 I2C 突发读所有可用样本，逐个送入测量管道。
 *
 * 返回处理的样本数。MAXtask 保存该数量，Uitask 用它判断是否暂缓 OLED 刷新。
 */
uint8_t app_measurement_drain_fifo_batch(AppState_t *app)
{
  uint32_t red_batch[24];
  uint32_t ir_batch[24];
  uint8_t  ovf, count, i;
  uint32_t now;
  MAX30102_BatchStatus_t batch_status;

  if (app == NULL) { return 0U; }

  /* 批量读 MAX30102 FIFO */
  count = max30102_read_fifo_batch(red_batch, ir_batch, 24U, &ovf,
                                    &batch_status);

  /* 同步驱动层 FIFO 调试字段到 AppState */
  {
    const MAX30102_FifoDebug_t *fdbg = max30102_get_fifo_debug();
    if (fdbg != NULL)
    {
      app->sensor_fifo_overflow_count = fdbg->overflow_count;
      app->sensor_fifo_write_ptr = fdbg->write_ptr;
      app->sensor_fifo_read_ptr = fdbg->read_ptr;
      app->sensor_fifo_available_samples = fdbg->available_samples;
    }
  }

  /* 记录尝试 */
  app->sensor_read_attempt_count++;

  /* 处理溢出 */
  if ((batch_status == MAX30102_BATCH_OVERFLOW) ||
      (batch_status == MAX30102_BATCH_FIFO_CLEAR_FAIL))
  {
    app->fifo_overflow_total += ovf;
    if (app->fifo_overflow_total > 999999UL)
    {
      app->fifo_overflow_total = 999999UL;
    }

    /* 立即重置连续性历史，不等到稳定结束 */
    app_measurement_reset_sampling_continuity(app, APP_OUTPUT_REASON_STALE);

    /* 仅在手指实际就位时进入接触稳定状态。
     * 如果手指不在，上面的溢出/算法状态重置
     * 就足够了 — 不要阻塞 finger_on_confirm_count。 */
    if (batch_status == MAX30102_BATCH_FIFO_CLEAR_FAIL)
    {
      app->sensor_health = (uint8_t)SENSOR_HEALTH_FIFO_CLEAR_FAIL;
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
      app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
      app->sensor_read_error_count++;
      app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    }
    else
    {
      app->sensor_health = (uint8_t)SENSOR_HEALTH_STALE;
    }

    return 0U;
  }

  if (count == 0U)
  {
    /* 非溢出返回 0 → 可能是 FIFO 空或 I2C 错误 */
    if ((batch_status == MAX30102_BATCH_I2C_ERROR) ||
        (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ||
        (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE))
    {
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
      app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
      if (app->sensor_error_streak < 0xFFU)
      {
        app->sensor_error_streak++;
      }
    }
    return 0U;
  }

  /* 更新高水位 */
  if (count > app->fifo_high_watermark)
  {
    app->fifo_high_watermark = (uint16_t)count;
  }

  /* 从最后一个样本开始回溯时间戳，逐个处理 */
  now = HAL_GetTick();

  for (i = 0U; i < count; i++)
  {
    uint32_t sample_tick;

    /* 标记当前处于逐样本处理阶段（用于崩溃诊断） */
    app->max_task_phase = PHASE_MAX_SAMPLE_PROC;

    /* 最后一个样本 = now，前面的按 10ms 向前回溯 */
    sample_tick = now - ((uint32_t)(count - 1U - i) * APP_SAMPLE_PERIOD_MS);

    /* 跟踪样本间隔，包含上一批最后样本到本批第一个样本的时间间隔。 */
    if (app->sensor_last_sample_tick != 0UL)
    {
      uint32_t gap = sample_tick - app->sensor_last_sample_tick;
      uint16_t gap16 = (gap > 0xFFFFUL) ? 0xFFFFU : (uint16_t)gap;
      if (gap16 > app->max_sample_gap_ms)
      {
        app->max_sample_gap_ms = gap16;
      }
    }

    /* 共享后处理 + 入队 */
    app_measurement_process_parsed_sample(app,
                                           red_batch[i],
                                           ir_batch[i],
                                           sample_tick);

    /* 单样本处理管道 */
    app_measurement_update_adaptive_thresholds(app);
    app_measurement_update_finger_state(app);
    app_measurement_process(app);
  }

  return count;
}

/*
 * OLED 刷新门控：仅在 FIFO 积压很高时短暂让路。
 * sensor_health/contact_settle 不再阻止刷新——状态页、调试页、按钮切页
 * 仍需要 OLED 正常工作。连续跳过超过 1000ms 由 UiTask 强制刷新。
 */
uint8_t app_measurement_should_skip_display(const AppState_t *app)
{
  if (app == NULL) { return 1U; }

  if (app->fifo_high_watermark >= 8U)
  {
    return 1U;
  }

  return 0U;
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
 * 样本陈旧看门狗 (app_measurement_service_sensor_watchdog) 调用。
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

  /* I2C 总线恢复 + MAX30102 重新初始化。 */
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
  /* 给 MAX30102 时间在复位后发布其第一个样本。 */
  app->sensor_last_sample_tick = now;
  sensor_last_stale_probe_tick = 0UL;
  sensor_stale_probe_count = 0U;
  sample_debug_state.initialized = 0U;
  app_measurement_reset_sampling_continuity(app, APP_OUTPUT_REASON_STALE);
  /* 进入短时重新采集：如果手指就位，设置接触稳定倒计数以
   * 在重新稳定期间抑制算法输出。如果手指不在，
   * 保持 PLACE FINGER — 不进入稳定。 */
  if (app->finger_present != 0U)
  {
    app->contact_settle_samples = APP_CONTACT_SETTLE_SAMPLES;
  }
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
}

/* ---- 运行时恢复辅助函数：清除 I2C，然后重试 MAX30102 初始化。 ---- */
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

/**
 ******************************************************************************
 * @brief  传感器错误路径恢复的入口点。
 * @param  app 指向共享应用状态。
 * @return 无。
 * @note   仅在 app->sensor_error_streak 达到 APP_SENSOR_RECOVERY_ERROR_COUNT
 *         时触发完整的 I2C 总线恢复 + MAX30102 重新初始化。
 *         两次尝试之间限频为 APP_SENSOR_RECOVERY_RETRY_MS。
 ******************************************************************************
 */
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
 * - 恢复失败：置 SENSOR_HEALTH_INIT_FAIL
 */
void app_measurement_service_sensor_watchdog(AppState_t *app)
{
  uint32_t now;
  uint8_t  sample_stale = 0U;

  if (app == NULL) return;

  /* 恢复期间不重复触发，等待恢复流程自行更新健康状态 */
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

  /* 任务调度或共享总线占用可能造成短暂读空；在清除 MAX30102 FIFO 并重建
   * I2C1 之前，必须按独立探测间隔确认停顿连续发生。 */
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

/* ---- 手指状态切换时重置所有测量输出 ---- */
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
  app_measurement_reset_beat_states();
  app_oxy_status_reset(app);
  app_ppg_sqi_reset(app);
  app_ppg_side_elgendi_reset(app);
  app_reset_advanced_metrics(app);

  /* PTT 依赖 PPG 脉搏波峰与 ECG R 峰的时间差，手指状态切换后旧历史无效 */
  app_ptt_reset(app);
}

/* ---- 信号丢失/运动时使高级输出无效（保留缓冲区） ---- */
static void app_invalidate_advanced_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_hrv_invalidate_outputs(app);
  app->rr_valid = 0U;
}

/* ---- 完全重置高级指标（HRV / RR 环形缓冲区 + PPGA 脉搏检测器） ---- */
static void app_reset_advanced_metrics(AppState_t *app)
{
  app_hrv_reset(app);
  app_rr_reset(app);
  app_ppg_pulse_reset();
  app_ppg_side_elgendi_reset(app);
  app_measurement_reset_beat_states();
}
