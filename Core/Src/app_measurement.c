#include "app_measurement.h"

#include <string.h>

#include "app_display.h"
#include "i2c.h"
#include "max30102.h"

/*
 * 手指检测采用“基线 + 双阈值 + 连续命中确认”：
 * - ON 阈值更高，避免环境抖动误判为有手指
 * - OFF 阈值更低，保证移开手指后能及时释放状态
 */
#define MAX30102_FINGER_ON_DELTA          6000UL
#define MAX30102_FINGER_OFF_DELTA         3000UL
#define MAX30102_FINGER_ON_NOISE_GAIN     4UL
#define MAX30102_FINGER_OFF_NOISE_GAIN    2UL
#define MAX30102_FINGER_ON_DELTA_MAX      18000UL
#define MAX30102_FINGER_OFF_DELTA_MAX     9000UL
#define MAX30102_REACQUIRE_NOISE_IR       3000UL
#define MAX30102_FINGER_ON_CONFIRM_COUNT  8U
#define MAX30102_FINGER_OFF_CONFIRM_COUNT 75U
#define APP_SENSOR_RECOVERY_ERROR_COUNT   8U

/* BPM 输出平滑参数，用于抑制跳变与尖峰。 */
#define APP_BPM_CONFIRM_DELTA             8U
#define APP_BPM_SWITCH_DELTA              14U
#define APP_BPM_SPIKE_DELTA               24U
#define APP_BPM_START_CONFIRM_COUNT       2U
#define APP_BPM_SWITCH_CONFIRM_COUNT      3U
#define APP_BPM_SPIKE_CONFIRM_COUNT       5U
#define APP_BPM_INVALID_HOLD_TICKS        30U
#define APP_BPM_EVALUATE_INTERVAL_SAMPLES 3U
#define APP_BPM_MAX_STEP_PER_UPDATE       5U
#define APP_BPM_ACORR_BLEND_MAX_DIFF      12U
#define APP_SIGNAL_QUALITY_MIN_FOR_SPO2   30U
#define APP_SIGNAL_QUALITY_MIN_FOR_BPM    25U
/*
 * HRV 参数：
 * - IBI 有效范围 300–2000 ms（对应 30–200 BPM）
 * - 至少 4 拍后才生成 SDNN/RMSSD
 * - 新 IBI 偏离当前均值 >50% 则拒绝（防止半拍/漏拍污染统计）
 */
#define APP_HRV_IBI_HISTORY_SIZE          32U
#define APP_HRV_MIN_VALID_IBI_COUNT       4U
#define APP_HRV_IBI_MIN_MS                300U
#define APP_HRV_IBI_MAX_MS                2000U
#define APP_HRV_IBI_JUMP_PERCENT          50U
/*
 * RR（呼吸率，RIAV 幅度调制法）参数：
 * - 至少 10 拍、8 秒窗口、脉搏幅度有调制才输出有效值
 * - 有效范围 8–30 breaths/min
 * - SQ 门控比 IBI/HRV 更严格 (>=35)，因为幅度调制对噪声更敏感
 */
#define APP_RR_RIAV_HISTORY_SIZE          32U
#define APP_RR_MIN_BEAT_COUNT             10U
#define APP_RR_MIN_WINDOW_SAMPLES         800U
#define APP_RR_MIN_RESULT                 8U
#define APP_RR_MAX_RESULT                 30U
#define APP_RR_SIGNAL_QUALITY_MIN         35U
#define APP_RR_MIN_MODULATION_DIV         20U
/* 流式脉冲检测：如果连续 3 秒无新峰，将 IBI/HRV 标为 invalid（旧值不立即清零）。 */
#define APP_ADVANCED_PULSE_STALE_SAMPLES  (MAX30102_ALGO_SAMPLE_RATE_HZ * 3U)
/* R/BAL 分级阈值 ×1000：<500 LOW, 500–2000 OK, >2000 HIGH。 */
#define APP_OXY_BAL_LOW_RATIO_X1000       500U
#define APP_OXY_BAL_HIGH_RATIO_X1000      2000U
#define APP_OXY_RATIO_MIN_AC_RMS          2U
/* SQ 平滑：每拍最多移动 raw_quality 差距的 1/4，避免 UI 跳变。 */
#define APP_SIGNAL_QUALITY_SMOOTH_SHIFT   2U
/* 流式脉冲检测：阈值和 prominence 由当前 IR AC RMS 动态换算。 */
#define APP_STREAM_PULSE_MIN_RMS          4U
#define APP_STREAM_PULSE_THRESHOLD_DIV    4U
#define APP_STREAM_PULSE_PROM_DIV         3U

static uint8_t fifo_buf[6];
static MAX30102_Baseline_t baseline_data;
static MAX30102_SpO2_t spo2_state;
static struct
{
  uint32_t red_value;
  uint32_t ir_value;
  uint8_t initialized;
} sample_debug_state;
static uint8_t bpm_update_decimator;
static struct
{
  uint32_t ir_high;
  uint32_t ir_low;
  uint32_t red_high;
  uint32_t red_low;
} signal_envelope;
/* HRV：环形缓冲保存最近 32 拍的 IBI（毫秒），用于计算 SDNN/RMSSD。 */
static struct
{
  uint16_t ibi_ms[APP_HRV_IBI_HISTORY_SIZE];
  uint8_t write_index;
  uint8_t count;
  uint8_t last_peak_valid;
  uint32_t last_peak_sample;
} hrv_state;
/* RR：保存每拍的全局样本编号和幅度，用于在幅度包络上检测呼吸频率。 */
static struct
{
  uint32_t peak_sample[APP_RR_RIAV_HISTORY_SIZE];
  uint32_t amplitude[APP_RR_RIAV_HISTORY_SIZE];
  uint8_t write_index;
  uint8_t count;
} rr_state;
/*
 * 流式脉冲检测状态（3 点滑动窗口）：
 * 与窗口峰值法互补——支持正峰和负谷检测，避免原算法在某些波形形态下漏掉 IBI。
 * 极性交替策略：同极性峰只更新位置，异极性峰才触发 IBI 确认。
 */
static struct
{
  int32_t previous2;
  int32_t previous1;
  uint8_t sample_count;
  uint8_t last_peak_valid;
  int8_t last_polarity;       /* 1=正峰，-1=负谷 */
  uint32_t last_peak_sample;  /* 全局样本编号 */
} stream_pulse_state;

static uint8_t app_abs_diff_u8(uint8_t lhs, uint8_t rhs);
static uint32_t app_abs_diff_u32(uint32_t lhs, uint32_t rhs);
static uint8_t app_limit_bpm_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step);
static uint8_t app_median3_u8(uint8_t a, uint8_t b, uint8_t c);
static uint8_t app_filter_raw_bpm(AppState_t *app, uint8_t raw_bpm_value);
static uint32_t app_isqrt_u64(uint64_t value);
static uint32_t app_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift);
static void app_reset_signal_envelope(void);
static void app_update_signal_activity(AppState_t *app);
static uint8_t app_is_raw_signal_present(const AppState_t *app);
static void app_track_background_ir(AppState_t *app);
static void app_reset_measurement_outputs(AppState_t *app);
static void app_update_bpm_output(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value);
static void app_invalidate_advanced_outputs(AppState_t *app);
static void app_clear_advanced_outputs(AppState_t *app);
static void app_reset_advanced_metrics(AppState_t *app);
static void app_update_oxy_status(AppState_t *app, const MAX30102_SignalMetrics_t *metrics);
static void app_update_signal_quality(AppState_t *app, uint8_t raw_quality);
static uint8_t app_stream_pulse_update(AppState_t *app,
                                       int32_t filtered_sample,
                                       MAX30102_PulseInfo_t *pulse_info);
static void app_process_pulse_metrics(AppState_t *app, const MAX30102_PulseInfo_t *pulse_info);
static uint8_t app_hrv_accept_ibi(uint16_t ibi_ms);
static uint8_t app_hrv_order_to_index(uint8_t order);
static void app_hrv_add_ibi(AppState_t *app, uint16_t ibi_ms);
static void app_hrv_update_outputs(AppState_t *app);
static uint8_t app_rr_order_to_index(uint8_t order);
static uint32_t app_rr_get_peak_sample(uint8_t order);
static uint32_t app_rr_get_amplitude(uint8_t order);
static void app_rr_add_beat(uint32_t peak_sample, uint32_t amplitude);
static void app_rr_update_output(AppState_t *app);

/* 初始化测量模块相关的自适应阈值。 */
void app_measurement_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;
}

/* 复位运行期算法状态，重新开始一次完整测量流程。 */
void app_measurement_reset_runtime(void)
{
  max30102_baseline_reset(&baseline_data);
  max30102_spo2_reset(&spo2_state);
  (void)memset(&sample_debug_state, 0, sizeof(sample_debug_state));
  bpm_update_decimator = 0U;
  app_reset_signal_envelope();
  app_reset_advanced_metrics(NULL);
  app_display_reset_waveforms();
}

/*
 * 基线采集阶段：读 MAX30102 FIFO 并累积背景 IR。
 *
 * 错误分类处理：
 *   - HAL_BUSY：I2C 总线忙于上一次传输（DMA 未完成或有残留事务），
 *     不计入连续错误计数，不清空错误连续（error_streak），
 *     因为这不是传感器或总线硬件故障，而是正常的并发争用。
 *   - HAL_ERROR / HAL_TIMEOUT：真正的硬件故障，记录 I2C 错误码、
 *     递增连续错误计数，达到阈值后触发传感器恢复流程。
 *
 * 读成功时：清零连续错误计数，记录最后成功采样的时间戳，
 * 用于判定传感器是否已失效很久（sensor_last_sample_tick）。
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
    /* I2C 总线忙：无数据可读但不是错误，不清零错误计数 */
    if (read_status == HAL_BUSY)
    {
      app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
      app->sensor_read_busy_count++;
      app->sensor_error_streak = 0U;
      return 0U;
    }

    /* I2C 硬件错误：记录错误码和连续计数，尝试恢复 */
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
  /* 读成功：清零所有错误状态 */
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
  app->sensor_read_ok_count++;
  app->sensor_error_streak = 0U;
  app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
  app->sensor_last_sample_tick = HAL_GetTick();
  return 1U;
}

/* 判断是否已经采集到足够多的基线样本。 */
uint8_t app_measurement_baseline_ready(void)
{
  return max30102_baseline_is_ready(&baseline_data, APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 返回 0~100 的基线采集进度，供状态页显示。 */
uint16_t app_measurement_get_baseline_progress_percent(void)
{
  if (baseline_data.sample_count >= APP_MEASUREMENT_BASELINE_SAMPLES)
  {
    return 100U;
  }

  return (uint16_t)((baseline_data.sample_count * 100U) / APP_MEASUREMENT_BASELINE_SAMPLES);
}

/* 读取基线平均值。 */
uint32_t app_measurement_get_baseline_average(void)
{
  return max30102_baseline_get_average_ir(&baseline_data);
}

/* 读取基线期间的波动范围。 */
uint32_t app_measurement_get_baseline_range(void)
{
  return max30102_baseline_get_range_ir(&baseline_data);
}

/* 获取后台跟踪后的当前基线。 */
uint32_t app_measurement_get_tracked_baseline(void)
{
  return max30102_baseline_get_tracked_ir(&baseline_data);
}

/* 用采集得到的背景值初始化跟踪器，进入正常运行阶段。 */
void app_measurement_seed_baseline_tracking(uint32_t baseline_ir, uint32_t noise_ir)
{
  max30102_baseline_seed_tracking(&baseline_data, baseline_ir, noise_ir);
}

/* 判断背景是否稳定，便于提示现场光照/接线问题。 */
uint8_t app_measurement_baseline_is_stable(void)
{
  return max30102_baseline_is_stable(&baseline_data, APP_MEASUREMENT_BASELINE_STABLE_RANGE);
}

/* 正常运行阶段读取一个 RED/IR 样本，并同步刷新基线快照。 */
AppMeasurementReadStatus_t app_measurement_read_sensor_sample(AppState_t *app)
{
  HAL_StatusTypeDef read_status;
  const MAX30102_FifoDebug_t *fifo_debug;

  if (app == NULL)
  {
    return APP_MEASUREMENT_READ_ERROR;
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
    app_update_signal_activity(app);
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_OK;
    app->sensor_read_ok_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    app->sensor_last_sample_tick = HAL_GetTick();
    return APP_MEASUREMENT_READ_OK;
  }

  if (read_status == HAL_BUSY)
  {
    app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
    app->sensor_read_busy_count++;
    app->sensor_error_streak = 0U;
    app->sensor_last_i2c_error = HAL_I2C_ERROR_NONE;
    return APP_MEASUREMENT_READ_WAIT;
  }

  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_ERROR;
  app->sensor_read_error_count++;
  app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
  if (app->sensor_error_streak < 0xFFU)
  {
    app->sensor_error_streak++;
  }

  return APP_MEASUREMENT_READ_ERROR;
}

/* 根据背景噪声动态放大/收敛手指检测阈值，减小误判。 */
void app_measurement_update_adaptive_thresholds(AppState_t *app)
{
  uint32_t baseline_noise;
  uint32_t on_delta;
  uint32_t off_delta;

  if (app == NULL)
  {
    return;
  }

  baseline_noise = max30102_baseline_get_noise_ir(&baseline_data);
  on_delta = baseline_noise * MAX30102_FINGER_ON_NOISE_GAIN;
  off_delta = baseline_noise * MAX30102_FINGER_OFF_NOISE_GAIN;
  app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA;
  app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA;

  if (on_delta > app->adaptive_finger_on_delta)
  {
    app->adaptive_finger_on_delta = on_delta;
  }

  if (off_delta > app->adaptive_finger_off_delta)
  {
    app->adaptive_finger_off_delta = off_delta;
  }

  if (app->adaptive_finger_on_delta > MAX30102_FINGER_ON_DELTA_MAX)
  {
    app->adaptive_finger_on_delta = MAX30102_FINGER_ON_DELTA_MAX;
  }

  if (app->adaptive_finger_off_delta > MAX30102_FINGER_OFF_DELTA_MAX)
  {
    app->adaptive_finger_off_delta = MAX30102_FINGER_OFF_DELTA_MAX;
  }
}

/*
 * 更新手指在位状态。
 * 只有在“确认为无手指”时，才允许基线继续缓慢跟踪环境变化。
 */
void app_measurement_update_finger_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->raw_signal_present = app_is_raw_signal_present(app);

  if (app->finger_present == 0U)
  {
    app->finger_off_confirm_count = 0U;

    if (app->raw_signal_present == 0U)
    {
      app->finger_on_confirm_count = 0U;
      app_track_background_ir(app);
      return;
    }

    if (app->finger_on_confirm_count < 0xFFU)
    {
      app->finger_on_confirm_count++;
    }

    if (app->finger_on_confirm_count >= MAX30102_FINGER_ON_CONFIRM_COUNT)
    {
      app->finger_present = 1U;
      app->finger_on_confirm_count = 0U;
      app_reset_measurement_outputs(app);
      app_reset_signal_envelope();
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

  if (app->finger_off_confirm_count < 0xFFU)
  {
    app->finger_off_confirm_count++;
  }

  if (app->finger_off_confirm_count >= MAX30102_FINGER_OFF_CONFIRM_COUNT)
  {
    app->finger_present = 0U;
    app->finger_off_confirm_count = 0U;
    app->finger_on_confirm_count = 0U;
    app_track_background_ir(app);
    max30102_baseline_seed_tracking(&baseline_data,
                                    app->baseline_ir,
                                    MAX30102_REACQUIRE_NOISE_IR);
    app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
    app_reset_measurement_outputs(app);
    app_reset_signal_envelope();
    app->report_due = 1U;
    app->display_refresh_requested = 1U;
  }
}

/* 手指在位时推进波形、SpO2 与 BPM 算法窗口。 */
void app_measurement_process(AppState_t *app)
{
  MAX30102_SignalMetrics_t signal_metrics;
  uint8_t raw_bpm_valid = 0U;
  uint8_t raw_bpm_value = 0U;
  uint8_t acorr_bpm_valid = 0U;
  uint8_t acorr_bpm = 0U;
  uint8_t raw_spo2_valid = 0U;
  uint8_t signal_quality = 0U;
  MAX30102_PulseInfo_t pulse_info;
  int32_t red_waveform_sample = 0;
  int32_t ir_waveform_sample = 0;

  if (app == NULL)
  {
    return;
  }

  /*
   * 波形和算法窗口仅在确认手指在位时更新：
   * - 无手指：不绘制环境噪声，保持界面干净
   * - 确认手指在位：同时推进波形显示和 BPM/SpO2 算法窗口
   */
  if (app->finger_present == 0U)
  {
    return;
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
    app->signal_ir_ac_rms = signal_metrics.ir_ac_rms;
    app->signal_red_ac_rms = signal_metrics.red_ac_rms;
    if (max30102_calculate_signal_quality(&spo2_state, &signal_metrics, &signal_quality) != 0U)
    {
      app_update_signal_quality(app, signal_quality);
    }
    else
    {
      app_update_signal_quality(app, 0U);
    }

    app->signal_ir_pi_x1000 = signal_metrics.ir_pi_x1000;
    app->signal_red_pi_x1000 = signal_metrics.red_pi_x1000;
    app_update_oxy_status(app, &signal_metrics);
  }
  /*
   * 信号指标获取失败（窗口不足等）：瞬时指标 SQ/PI/R 立即清零，
   * 但 IBI/HRV/RR 只标 invalid 不清数值——它们有跨拍历史窗口，
   * 单拍失败不代表多拍统计结果立刻失效。
   */
  else
  {
    app->signal_ir_ac_rms = 0U;
    app->signal_red_ac_rms = 0U;
    app->signal_quality = 0U;
    app->signal_ir_pi_x1000 = 0U;
    app->signal_red_pi_x1000 = 0U;
    app->spo2_ratio_valid = 0U;
    app->spo2_ratio_x1000 = 0U;
    app->spo2_balance_status = APP_OXY_BALANCE_UNKNOWN;
    app_invalidate_advanced_outputs(app);
  }

  /* 高级指标门控放宽：
   * SQ 低时只清 RR（对噪声最敏感），不清 IBI/HRV。
   * RR 的 SQ 门控比 BPM 更高 (>=35 vs >=25)，因为幅度调制要求信号更干净。 */
  if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    app->rr_valid = 0U;
  }
  else if (app->signal_quality < APP_RR_SIGNAL_QUALITY_MIN)
  {
    app->rr_valid = 0U;
  }

  /* 流式脉冲陈旧性检查：若连续 3 秒无新峰，标记 IBI/HRV 为 ? (invalid)。
   * 不立刻清零数值：UI 显示 "123?" 让用户知道是旧值而非丢失。 */
  if ((hrv_state.last_peak_valid != 0U) &&
      (spo2_state.total_samples > (hrv_state.last_peak_sample + APP_ADVANCED_PULSE_STALE_SAMPLES)))
  {
    app_invalidate_advanced_outputs(app);
  }

  /* 流式脉冲检测：基于 3 点滑动窗口的实时正峰/负谷检测，
   * 与窗口峰值法互补，避免后者在 256 点批次边界漏掉 IBI。 */
  if (app_stream_pulse_update(app, ir_waveform_sample, &pulse_info) != 0U)
  {
    app_process_pulse_metrics(app, &pulse_info);
  }

  raw_spo2_valid = max30102_calculate_spo2(&spo2_state, &app->spo2_value);
  if ((raw_spo2_valid != 0U) && (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_SPO2))
  {
    raw_spo2_valid = 0U;
  }

  app->spo2_valid = raw_spo2_valid;

  bpm_update_decimator++;
  if (bpm_update_decimator < APP_BPM_EVALUATE_INTERVAL_SAMPLES)
  {
    return;
  }

  bpm_update_decimator = 0U;
  raw_bpm_valid = max30102_calculate_bpm_with_pulse(&spo2_state, &raw_bpm_value, &pulse_info);

  /*
   * 当信号质量足够时，用时域峰值检测 + FFT 自相关两种方法互相验证。
   * 自相关法对弱信号和波形极性更鲁棒，因此也作为峰值法失败时的备用路径。
   */
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
      /* Weighted blend: 25% peak-detect + 75% autocorrelation. */
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

  /* BPM 窗口峰有效时同步推进 IBI/HRV；若 BPM 无效且 SQ 过低则清 RR。
   * 注意：beat_valid=0 时不清 IBI/HRV——流式检测可能仍持有有效数据。 */
  if (pulse_info.beat_valid != 0U)
  {
    app_process_pulse_metrics(app, &pulse_info);
  }
  else if (app->signal_quality < APP_SIGNAL_QUALITY_MIN_FOR_BPM)
  {
    app->rr_valid = 0U;
  }

  app_update_bpm_output(app, raw_bpm_valid, raw_bpm_value);
}

/* 把 100 Hz 采样节拍降频成约 5 Hz 的显示/上报节拍。 */
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

/*
 * 传感器恢复流程 —— 连续 I2C 错误达到阈值时触发。
 *
 * 恢复步骤：
 *   1. 先尝试 I2C 总线恢复（GPIO 位打钟清除 + 重新 Init）
 *      - 这是新增的保护措施：如果 MAX30102 从机在传输中途意外掉电/复位，
 *        它可能将 SDA 拉死。单纯的 max30102_init 重新配置寄存器没用，
 *        因为 I2C 外设检测到总线忙，连 START 条件都发不出。
 *      - MX_I2C1_RecoverBus 用 GPIO 手动打出 STOP 条件和时钟脉冲，
 *        强制释放 SDA，然后重新初始化 I2C 外设。
 *   2. 总线恢复成功后，重新初始化 MAX30102 传感器
 *   3. 重建算法状态：复位信号包络，重新播种基线跟踪，
 *      清空测量输出，避免恢复后的残留数据混入新的测量窗口
 *
 * 恢复失败保护：
 *   如果总线恢复或传感器重新初始化失败，将 error_streak 保持为阈值，
 *   这样下一次错误不会立即再次尝试恢复（避免频繁重试），
 *   只有在新一轮错误再次累计到阈值后才会重试。
 */
void app_measurement_recover_sensor(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if (app->sensor_error_streak < APP_SENSOR_RECOVERY_ERROR_COUNT)
  {
    return;
  }

  /* 先恢复 I2C 总线，再重新初始化传感器 */
  if (MX_I2C1_RecoverBus() != HAL_OK)
  {
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    return;
  }

  if (max30102_init() != HAL_OK)
  {
    app->sensor_error_streak = APP_SENSOR_RECOVERY_ERROR_COUNT;
    app->sensor_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    return;
  }

  app->sensor_error_streak = 0U;
  app->sensor_last_read_status = (uint8_t)APP_MEASUREMENT_READ_WAIT;
  app->sensor_recover_count++;
  app_reset_signal_envelope();
  app_reset_measurement_outputs(app);
  max30102_baseline_seed_tracking(&baseline_data,
                                  app->baseline_ir,
                                  MAX30102_REACQUIRE_NOISE_IR);
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
  app->display_refresh_requested = 1U;
  app->report_due = 1U;
}

static uint32_t app_slow_follow_u32(uint32_t current, uint32_t target, uint8_t shift)
{
  uint32_t delta;
  uint32_t step;

  if (current == target)
  {
    return current;
  }

  if (current < target)
  {
    delta = target - current;
    step = delta >> shift;
    if (step == 0U)
    {
      step = 1U;
    }

    return current + step;
  }

  delta = current - target;
  step = delta >> shift;
  if (step == 0U)
  {
    step = 1U;
  }

  return current - step;
}

static uint8_t app_abs_diff_u8(uint8_t lhs, uint8_t rhs)
{
  if (lhs >= rhs)
  {
    return (uint8_t)(lhs - rhs);
  }

  return (uint8_t)(rhs - lhs);
}

static uint8_t app_limit_bpm_step(uint8_t current_bpm, uint8_t target_bpm, uint8_t max_step)
{
  if (target_bpm > current_bpm)
  {
    if ((uint8_t)(target_bpm - current_bpm) > max_step)
    {
      return (uint8_t)(current_bpm + max_step);
    }
  }
  else if (current_bpm > target_bpm)
  {
    if ((uint8_t)(current_bpm - target_bpm) > max_step)
    {
      return (uint8_t)(current_bpm - max_step);
    }
  }

  return target_bpm;
}

/* 3 点中值滤波，用于抑制原始 BPM 的偶发抖动。 */
static uint8_t app_median3_u8(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t temp;

  if (a > b)
  {
    temp = a;
    a = b;
    b = temp;
  }

  if (b > c)
  {
    temp = b;
    b = c;
    c = temp;
  }

  if (a > b)
  {
    b = a;
  }

  return b;
}

/* 对原始 BPM 先做极轻量的历史平滑，再交给输出稳定器。 */
static uint8_t app_filter_raw_bpm(AppState_t *app, uint8_t raw_bpm_value)
{
  uint8_t first_index;
  uint8_t second_index;

  if (app == NULL)
  {
    return raw_bpm_value;
  }

  app->raw_bpm_history[app->raw_bpm_history_index] = raw_bpm_value;
  app->raw_bpm_history_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  if (app->raw_bpm_history_count < 3U)
  {
    app->raw_bpm_history_count++;
  }

  if (app->raw_bpm_history_count == 1U)
  {
    return app->raw_bpm_history[0];
  }

  if (app->raw_bpm_history_count == 2U)
  {
    return (uint8_t)(((uint16_t)app->raw_bpm_history[0] +
                      (uint16_t)app->raw_bpm_history[1] + 1U) / 2U);
  }

  first_index = app->raw_bpm_history_index;
  second_index = (uint8_t)((app->raw_bpm_history_index + 1U) % 3U);

  return app_median3_u8(app->raw_bpm_history[first_index],
                        app->raw_bpm_history[second_index],
                        app->raw_bpm_history[(uint8_t)((app->raw_bpm_history_index + 2U) % 3U)]);
}

/* 仅在确认为“无手指”时，允许背景 IR 慢速跟踪环境变化。 */
static void app_reset_signal_envelope(void)
{
  (void)memset(&signal_envelope, 0, sizeof(signal_envelope));
}

static void app_update_signal_activity(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if ((signal_envelope.ir_high == 0U) || (app->ir_value >= signal_envelope.ir_high))
  {
    signal_envelope.ir_high = app->ir_value;
  }
  else
  {
    signal_envelope.ir_high = app_slow_follow_u32(signal_envelope.ir_high, app->ir_value, 4U);
  }

  if ((signal_envelope.ir_low == 0U) || (app->ir_value <= signal_envelope.ir_low))
  {
    signal_envelope.ir_low = app->ir_value;
  }
  else
  {
    signal_envelope.ir_low = app_slow_follow_u32(signal_envelope.ir_low, app->ir_value, 4U);
  }

  if ((signal_envelope.red_high == 0U) || (app->red_value >= signal_envelope.red_high))
  {
    signal_envelope.red_high = app->red_value;
  }
  else
  {
    signal_envelope.red_high = app_slow_follow_u32(signal_envelope.red_high, app->red_value, 4U);
  }

  if ((signal_envelope.red_low == 0U) || (app->red_value <= signal_envelope.red_low))
  {
    signal_envelope.red_low = app->red_value;
  }
  else
  {
    signal_envelope.red_low = app_slow_follow_u32(signal_envelope.red_low, app->red_value, 4U);
  }

  app->ir_signal_delta = (app->ir_value > app->baseline_ir) ?
                         (app->ir_value - app->baseline_ir) : 0U;
  app->ir_signal_span = (signal_envelope.ir_high >= signal_envelope.ir_low) ?
                        (signal_envelope.ir_high - signal_envelope.ir_low) : 0U;
  app->red_signal_span = (signal_envelope.red_high >= signal_envelope.red_low) ?
                         (signal_envelope.red_high - signal_envelope.red_low) : 0U;
}

static uint8_t app_is_raw_signal_present(const AppState_t *app)
{
  uint32_t finger_delta_threshold;
  uint32_t ir_delta;

  if (app == NULL)
  {
    return 0U;
  }

  if (app->ir_value <= app->baseline_ir)
  {
    return 0U;
  }

  finger_delta_threshold = (app->finger_present != 0U) ?
                           app->adaptive_finger_off_delta :
                           app->adaptive_finger_on_delta;
  ir_delta = app->ir_value - app->baseline_ir;

  return (ir_delta >= finger_delta_threshold) ? 1U : 0U;
}

static void app_track_background_ir(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  max30102_baseline_track_background(&baseline_data, app->ir_value);
  app->baseline_ir = max30102_baseline_get_tracked_ir(&baseline_data);
}

/* 手指移开后立即清空测量窗口，避免旧数据残留到下一轮。 */
static void app_reset_measurement_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app_display_reset_waveforms();
  max30102_spo2_reset(&spo2_state);
  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
  app->spo2_valid = 0U;
  app->spo2_value = 0U;
  app->raw_signal_present = 0U;
  app->ir_signal_delta = 0U;
  app->ir_signal_span = 0U;
  app->red_signal_span = 0U;
  app->signal_ir_ac_rms = 0U;
  app->signal_red_ac_rms = 0U;
  app->signal_quality = 0U;
  app->signal_ir_pi_x1000 = 0U;
  app->signal_red_pi_x1000 = 0U;
  app->spo2_ratio_valid = 0U;
  app->spo2_ratio_x1000 = 0U;
  app->spo2_balance_status = APP_OXY_BALANCE_UNKNOWN;
  app_reset_advanced_metrics(app);
}

/*
 * BPM 输出稳定器：
 * - 起始阶段要求候选值连续命中
 * - 已有稳定值后，只有连续接近的新值才会平滑切换
 * - 大尖峰需要更高确认次数
 */
static void app_update_bpm_output(AppState_t *app, uint8_t raw_bpm_valid, uint8_t raw_bpm_value)
{
  uint8_t diff;
  uint8_t filtered_bpm;
  uint8_t required_confirm_count;
  uint16_t blended_value;

  if (app == NULL)
  {
    return;
  }

  if (raw_bpm_valid != 0U)
  {
    filtered_bpm = app_filter_raw_bpm(app, raw_bpm_value);
    app->bpm_invalid_hold_count = 0U;

    if (app->bpm_valid == 0U)
    {
      if ((app->bpm_candidate_count == 0U) ||
          (app_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_CONFIRM_DELTA))
      {
        app->bpm_candidate_value = filtered_bpm;
        app->bpm_candidate_count = 1U;
        return;
      }

      if (app->bpm_candidate_count < 0xFFU)
      {
        app->bpm_candidate_count++;
      }

      blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
      app->bpm_candidate_value = (uint8_t)blended_value;

      if (app->bpm_candidate_count >= APP_BPM_START_CONFIRM_COUNT)
      {
        app->bpm_valid = 1U;
        app->bpm_value = app->bpm_candidate_value;
        app->bpm_candidate_count = 0U;
      }

      return;
    }

    diff = app_abs_diff_u8(filtered_bpm, app->bpm_value);
    if (diff <= APP_BPM_CONFIRM_DELTA)
    {
      blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)filtered_bpm + 4U) / 8U;
      app->bpm_value = app_limit_bpm_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
      return;
    }

    required_confirm_count = APP_BPM_SWITCH_CONFIRM_COUNT;
    if (diff > APP_BPM_SPIKE_DELTA)
    {
      required_confirm_count = APP_BPM_SPIKE_CONFIRM_COUNT;
    }

    if ((app->bpm_candidate_count == 0U) ||
        (app_abs_diff_u8(filtered_bpm, app->bpm_candidate_value) > APP_BPM_SWITCH_DELTA))
    {
      app->bpm_candidate_value = filtered_bpm;
      app->bpm_candidate_count = 1U;
      return;
    }

    if (app->bpm_candidate_count < 0xFFU)
    {
      app->bpm_candidate_count++;
    }

    blended_value = ((uint16_t)app->bpm_candidate_value + (uint16_t)filtered_bpm + 1U) / 2U;
    app->bpm_candidate_value = (uint8_t)blended_value;

    if (app->bpm_candidate_count >= required_confirm_count)
    {
      if (diff > APP_BPM_SPIKE_DELTA)
      {
        blended_value = (((uint16_t)app->bpm_value * 7U) + (uint16_t)app->bpm_candidate_value + 4U) / 8U;
      }
      else
      {
        blended_value = (((uint16_t)app->bpm_value * 5U) +
                         ((uint16_t)app->bpm_candidate_value * 3U) + 4U) / 8U;
      }

      app->bpm_value = app_limit_bpm_step(app->bpm_value,
                                          (uint8_t)blended_value,
                                          APP_BPM_MAX_STEP_PER_UPDATE);
      app->bpm_candidate_value = app->bpm_value;
      app->bpm_candidate_count = 0U;
    }

    return;
  }

  if (app->bpm_valid != 0U)
  {
    if (app->bpm_invalid_hold_count < APP_BPM_INVALID_HOLD_TICKS)
    {
      app->bpm_invalid_hold_count++;
      return;
    }
  }

  app->bpm_valid = 0U;
  app->bpm_value = 0U;
  bpm_update_decimator = 0U;
  (void)memset(app->raw_bpm_history, 0, sizeof(app->raw_bpm_history));
  app->raw_bpm_history_count = 0U;
  app->raw_bpm_history_index = 0U;
  app->bpm_candidate_value = 0U;
  app->bpm_candidate_count = 0U;
  app->bpm_invalid_hold_count = 0U;
}

static uint32_t app_abs_diff_u32(uint32_t lhs, uint32_t rhs)
{
  if (lhs >= rhs)
  {
    return lhs - rhs;
  }

  return rhs - lhs;
}

static uint32_t app_isqrt_u64(uint64_t value)
{
  uint64_t bit = 1ULL << 62;
  uint64_t result = 0ULL;

  while (bit > value)
  {
    bit >>= 2U;
  }

  while (bit != 0ULL)
  {
    if (value >= (result + bit))
    {
      value -= result + bit;
      result = (result >> 1U) + bit;
    }
    else
    {
      result >>= 1U;
    }

    bit >>= 2U;
  }

  if (result > 0xFFFFFFFFULL)
  {
    return 0xFFFFFFFFUL;
  }

  return (uint32_t)result;
}

/*
 * 标记 IBI/HRV/RR 为 invalid（不清零数值）：
 * 用于信号短暂中断时让 UI 显示 "?" 保留旧值，区别于彻底清空（手指离开）。
 */
static void app_invalidate_advanced_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->ibi_valid = 0U;
  app->hrv_valid = 0U;
  app->rr_valid = 0U;
}

/* 清零所有高级指标（数值+valid），用于手指离开或测量重置。 */
static void app_clear_advanced_outputs(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->ibi_valid = 0U;
  app->latest_ibi_ms = 0U;
  app->hrv_valid = 0U;
  app->hrv_mean_ibi_ms = 0U;
  app->hrv_sdnn_ms = 0U;
  app->hrv_rmssd_ms = 0U;
  app->rr_valid = 0U;
  app->rr_bpm = 0U;
}

/* 同时重置高级指标的运行时状态（环形缓冲、流式检测状态）和输出值。 */
static void app_reset_advanced_metrics(AppState_t *app)
{
  (void)memset(&hrv_state, 0, sizeof(hrv_state));
  (void)memset(&rr_state, 0, sizeof(rr_state));
  (void)memset(&stream_pulse_state, 0, sizeof(stream_pulse_state));
  app_clear_advanced_outputs(app);
}

/*
 * 计算 R/BAL 比值并做 3:1 平滑：
 * R = (red_ac / red_dc) / (ir_ac / ir_dc)
 * 弱帧（AC 或 DC 过小）只标记 ratio_valid=0，不清 ratio_x1000 旧值——UI 显示 "R:0.85?"。
 * 平滑公式：next = (old * 3 + new) / 4。
 */
static void app_update_oxy_status(AppState_t *app, const MAX30102_SignalMetrics_t *metrics)
{
  uint32_t ratio_x1000;
  uint16_t next_ratio;

  /* AC RMS 或 DC 过小时只标 invalid，不清旧值（弱帧不污染平滑历史）。 */
  if ((app == NULL) || (metrics == NULL) ||
      (metrics->red_dc == 0U) || (metrics->ir_dc == 0U) ||
      (metrics->red_ac_rms < APP_OXY_RATIO_MIN_AC_RMS) ||
      (metrics->ir_ac_rms < APP_OXY_RATIO_MIN_AC_RMS))
  {
    if (app != NULL)
    {
      app->spo2_ratio_valid = 0U;
      app->spo2_balance_status = APP_OXY_BALANCE_UNKNOWN;
    }
    return;
  }

  ratio_x1000 = (uint32_t)((((uint64_t)metrics->red_ac_rms * (uint64_t)metrics->ir_dc * 1000ULL) +
                            (((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc) / 2ULL)) /
                           ((uint64_t)metrics->ir_ac_rms * (uint64_t)metrics->red_dc));
  if (ratio_x1000 > 0xFFFFUL)
  {
    ratio_x1000 = 0xFFFFUL;
  }

  next_ratio = (uint16_t)ratio_x1000;
  if ((app->spo2_ratio_valid != 0U) || (app->spo2_ratio_x1000 != 0U))
  {
    next_ratio = (uint16_t)((((uint32_t)app->spo2_ratio_x1000 * 3U) + ratio_x1000 + 2U) / 4U);
  }

  app->spo2_ratio_valid = 1U;
  app->spo2_ratio_x1000 = next_ratio;

  if (next_ratio < APP_OXY_BAL_LOW_RATIO_X1000)
  {
    app->spo2_balance_status = APP_OXY_BALANCE_LOW;
  }
  else if (next_ratio > APP_OXY_BAL_HIGH_RATIO_X1000)
  {
    app->spo2_balance_status = APP_OXY_BALANCE_HIGH;
  }
  else
  {
    app->spo2_balance_status = APP_OXY_BALANCE_OK;
  }
}

/*
 * SQ 平滑更新：每拍最多向 raw_quality 移动 1/4 差距。
 * 上跳快（step>=1），下跳也快——避免瞬时波动导致 UI 数字剧烈跳变，
 * 同时信号真正恶化时能较快反映。
 */
static void app_update_signal_quality(AppState_t *app, uint8_t raw_quality)
{
  uint8_t current_quality;
  uint8_t delta;
  uint8_t step;

  if (app == NULL)
  {
    return;
  }

  current_quality = app->signal_quality;
  if (current_quality == raw_quality)
  {
    return;
  }

  if (current_quality < raw_quality)
  {
    delta = (uint8_t)(raw_quality - current_quality);
    step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
    if (step == 0U)
    {
      step = 1U;
    }

    app->signal_quality = (uint8_t)(current_quality + step);
    return;
  }

  delta = (uint8_t)(current_quality - raw_quality);
  step = (uint8_t)(delta >> APP_SIGNAL_QUALITY_SMOOTH_SHIFT);
  if (step == 0U)
  {
    step = 1U;
  }

  app->signal_quality = (uint8_t)(current_quality - step);
}

/*
 * 流式 3 点滑动窗口脉冲检测（逐拍运行，与 256 点窗口 BPM 互补）：
 * - 同时检测正峰和负谷，由 RMS 动态换算阈值和 prominence
 * - 极性交替策略：同极性峰只更新位置不触发 IBI；异极性峰才确认一次心搏
 * - 返回 1 表示本次拍检测到新心搏 (pulse_info 填充有效数据)
 * 目的：窗口峰值法在 256 点批次边界可能漏掉 IBI；流式检测在每个样本上判断，
 *       无批次概念，覆盖更全面。
 */
static uint8_t app_stream_pulse_update(AppState_t *app,
                                       int32_t filtered_sample,
                                       MAX30102_PulseInfo_t *pulse_info)
{
  const uint32_t min_interval_samples =
      (((uint32_t)APP_HRV_IBI_MIN_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;
  const uint32_t max_interval_samples =
      (((uint32_t)APP_HRV_IBI_MAX_MS * MAX30102_ALGO_SAMPLE_RATE_HZ) + 999U) / 1000U;
  uint32_t current_sample_number;
  uint32_t candidate_sample_number;
  uint32_t interval_samples;
  uint32_t threshold;
  uint32_t prominence_threshold;
  uint32_t positive_prominence = 0U;
  uint32_t negative_prominence = 0U;
  uint32_t magnitude = 0U;
  int32_t neighbor_reference;
  int8_t polarity = 0;
  uint8_t pulse_detected = 0U;

  if (pulse_info != NULL)
  {
    (void)memset(pulse_info, 0, sizeof(*pulse_info));
  }

  if ((app == NULL) || (pulse_info == NULL) || (app->finger_present == 0U))
  {
    (void)memset(&stream_pulse_state, 0, sizeof(stream_pulse_state));
    return 0U;
  }

  if (stream_pulse_state.sample_count < 2U)
  {
    if (stream_pulse_state.sample_count == 0U)
    {
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 1U;
    }
    else
    {
      stream_pulse_state.previous2 = stream_pulse_state.previous1;
      stream_pulse_state.previous1 = filtered_sample;
      stream_pulse_state.sample_count = 2U;
    }
    return 0U;
  }

  threshold = app->signal_ir_ac_rms / APP_STREAM_PULSE_THRESHOLD_DIV;
  if (threshold < APP_STREAM_PULSE_MIN_RMS)
  {
    threshold = APP_STREAM_PULSE_MIN_RMS;
  }

  prominence_threshold = threshold / APP_STREAM_PULSE_PROM_DIV;
  if (prominence_threshold < 2U)
  {
    prominence_threshold = 2U;
  }

  if ((stream_pulse_state.previous1 > stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 >= filtered_sample) &&
      (stream_pulse_state.previous1 > (int32_t)threshold))
  {
    neighbor_reference = (stream_pulse_state.previous2 > filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (stream_pulse_state.previous1 > neighbor_reference)
    {
      positive_prominence = (uint32_t)(stream_pulse_state.previous1 - neighbor_reference);
    }
  }

  if ((stream_pulse_state.previous1 < stream_pulse_state.previous2) &&
      (stream_pulse_state.previous1 <= filtered_sample) &&
      (stream_pulse_state.previous1 < -((int32_t)threshold)))
  {
    neighbor_reference = (stream_pulse_state.previous2 < filtered_sample) ?
                         stream_pulse_state.previous2 : filtered_sample;
    if (neighbor_reference > stream_pulse_state.previous1)
    {
      negative_prominence = (uint32_t)(neighbor_reference - stream_pulse_state.previous1);
    }
  }

  if ((positive_prominence >= prominence_threshold) ||
      (negative_prominence >= prominence_threshold))
  {
    if (positive_prominence >= negative_prominence)
    {
      polarity = 1;
      magnitude = (uint32_t)stream_pulse_state.previous1;
    }
    else
    {
      polarity = -1;
      magnitude = (uint32_t)(-stream_pulse_state.previous1);
    }
  }

  /*
   * 极性交替确认策略：
   * - 同极性峰（正-正或负-负）：只更新位置，不认为是一次完整心搏
   * - 异极性峰（正-负或负-正）：确认一次心搏，触发 IBI 输出
   * 这样做的原因是 PPG 波形在一次心搏中同时产生正峰和负谷，
   * 按正-负交替可以稳健地界定每次心跳的边界。
   */
  if ((polarity != 0) && (spo2_state.total_samples >= 2U))
  {
    current_sample_number = spo2_state.total_samples - 1U;
    candidate_sample_number = current_sample_number - 1U;

    if (stream_pulse_state.last_peak_valid == 0U)
    {
      stream_pulse_state.last_peak_valid = 1U;
      stream_pulse_state.last_polarity = polarity;
      stream_pulse_state.last_peak_sample = candidate_sample_number;
    }
    else if (polarity != stream_pulse_state.last_polarity)
    {
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        stream_pulse_state.last_polarity = polarity;
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
    }
    else
    {
      interval_samples = candidate_sample_number - stream_pulse_state.last_peak_sample;
      if (interval_samples > max_interval_samples)
      {
        stream_pulse_state.last_peak_sample = candidate_sample_number;
      }
      else if (interval_samples >= min_interval_samples)
      {
        stream_pulse_state.last_peak_sample = candidate_sample_number;
        pulse_info->beat_valid = 1U;
        pulse_info->interval_samples = (uint16_t)interval_samples;
        pulse_info->latest_ibi_ms = (uint16_t)((interval_samples * 1000U +
                                                (MAX30102_ALGO_SAMPLE_RATE_HZ / 2U)) /
                                               MAX30102_ALGO_SAMPLE_RATE_HZ);
        pulse_info->latest_peak_sample = candidate_sample_number;
        pulse_info->beat_amplitude = magnitude;
        pulse_detected = 1U;
      }
    }
  }

  stream_pulse_state.previous2 = stream_pulse_state.previous1;
  stream_pulse_state.previous1 = filtered_sample;
  return pulse_detected;
}

/*
 * 处理流式检测到的一次心搏：
 * - 去重（同采样编号不重复处理）
 * - IBI 合理性检查后压入 HRV 环形缓冲
 * - 在波形上标记脉搏位置
 * - SQ 满足 RR 门槛时将幅度推入呼吸包络缓冲
 */
static void app_process_pulse_metrics(AppState_t *app, const MAX30102_PulseInfo_t *pulse_info)
{
  if ((app == NULL) || (pulse_info == NULL) || (pulse_info->beat_valid == 0U))
  {
    return;
  }

  if ((hrv_state.last_peak_valid != 0U) &&
      (hrv_state.last_peak_sample == pulse_info->latest_peak_sample))
  {
    return;
  }

  hrv_state.last_peak_valid = 1U;
  hrv_state.last_peak_sample = pulse_info->latest_peak_sample;

  if (app_hrv_accept_ibi(pulse_info->latest_ibi_ms) == 0U)
  {
    app_invalidate_advanced_outputs(app);
    return;
  }

  app_hrv_add_ibi(app, pulse_info->latest_ibi_ms);
  app_display_add_ir_pulse_marker();

  if ((app->signal_quality >= APP_RR_SIGNAL_QUALITY_MIN) && (pulse_info->beat_amplitude != 0U))
  {
    app_rr_add_beat(pulse_info->latest_peak_sample, pulse_info->beat_amplitude);
    app_rr_update_output(app);
  }
  else
  {
    app->rr_valid = 0U;
  }
}

/*
 * IBI 合理性检查：
 * - 绝对范围：300–2000 ms
 * - 相对跳变：已有 >=4 个历史 IBI 时，新值偏离当前均值 >50% 则拒绝
 *   避免半拍（1/2 真实间隔）或漏拍（2× 真实间隔）污染 HRV 统计。
 */
static uint8_t app_hrv_accept_ibi(uint16_t ibi_ms)
{
  uint8_t i;
  uint32_t sum;
  uint16_t mean_ibi;

  if ((ibi_ms < APP_HRV_IBI_MIN_MS) || (ibi_ms > APP_HRV_IBI_MAX_MS))
  {
    return 0U;
  }

  if (hrv_state.count < APP_HRV_MIN_VALID_IBI_COUNT)
  {
    return 1U;
  }

  sum = 0U;
  for (i = 0U; i < hrv_state.count; i++)
  {
    sum += hrv_state.ibi_ms[i];
  }

  mean_ibi = (uint16_t)((sum + (hrv_state.count / 2U)) / hrv_state.count);
  if (mean_ibi == 0U)
  {
    return 1U;
  }

  if (app_abs_diff_u32(ibi_ms, mean_ibi) >
      (((uint32_t)mean_ibi * APP_HRV_IBI_JUMP_PERCENT) / 100U))
  {
    return 0U;
  }

  return 1U;
}

/*
 * 将已通过合理性检查的 IBI 推入 HRV 环形缓冲：
 * - 写入当前 write_index 位置并推进环形指针
 * - 更新 ibi_valid=1 和 latest_ibi_ms 供 UI 立即显示
 * - 触发 SDNN/RMSSD 重算 (app_hrv_update_outputs)
 * - 缓冲不足 4 拍时 hrv_valid 保持 0，UI 侧 SDNN/RMSSD 显示 "--"
 */
static void app_hrv_add_ibi(AppState_t *app, uint16_t ibi_ms)
{
  hrv_state.ibi_ms[hrv_state.write_index] = ibi_ms;
  hrv_state.write_index = (uint8_t)((hrv_state.write_index + 1U) % APP_HRV_IBI_HISTORY_SIZE);
  if (hrv_state.count < APP_HRV_IBI_HISTORY_SIZE)
  {
    hrv_state.count++;
  }

  if (app != NULL)
  {
    app->ibi_valid = 1U;
    app->latest_ibi_ms = ibi_ms;
  }

  app_hrv_update_outputs(app);
}

static uint8_t app_hrv_order_to_index(uint8_t order)
{
  uint8_t start_index;

  if (hrv_state.count < APP_HRV_IBI_HISTORY_SIZE)
  {
    start_index = 0U;
  }
  else
  {
    start_index = hrv_state.write_index;
  }

  return (uint8_t)((start_index + order) % APP_HRV_IBI_HISTORY_SIZE);
}

/*
 * 从 IBI 环形缓冲计算 SDNN 和 RMSSD：
 * - SDNN = sqrt(Σ(ibi - mean)^2 / (N-1)) —— 总体心率变异性
 * - RMSSD = sqrt(Σ(ibi[i] - ibi[i-1])^2 / (N-1)) —— 短期变异性，反映副交感神经活性
 * - 需要 >=4 个有效 IBI 才输出
 * - 使用整数运算（uint64 累积 + 自实现 isqrt），避免 FPU 开销
 */
static void app_hrv_update_outputs(AppState_t *app)
{
  uint8_t i;
  uint32_t sum = 0U;
  uint16_t mean_ibi;
  uint64_t variance_sum = 0ULL;
  uint64_t rmssd_sum = 0ULL;
  int32_t diff;
  uint16_t previous_ibi = 0U;
  uint16_t ibi_ms;

  if (app == NULL)
  {
    return;
  }

  if (hrv_state.count < APP_HRV_MIN_VALID_IBI_COUNT)
  {
    app->hrv_valid = 0U;
    return;
  }

  for (i = 0U; i < hrv_state.count; i++)
  {
    sum += hrv_state.ibi_ms[app_hrv_order_to_index(i)];
  }

  mean_ibi = (uint16_t)((sum + (hrv_state.count / 2U)) / hrv_state.count);

  for (i = 0U; i < hrv_state.count; i++)
  {
    ibi_ms = hrv_state.ibi_ms[app_hrv_order_to_index(i)];

    diff = (int32_t)ibi_ms - (int32_t)mean_ibi;
    variance_sum += (uint64_t)((int64_t)diff * (int64_t)diff);

    if (i != 0U)
    {
      diff = (int32_t)ibi_ms - (int32_t)previous_ibi;
      rmssd_sum += (uint64_t)((int64_t)diff * (int64_t)diff);
    }

    previous_ibi = ibi_ms;
  }

  app->hrv_valid = 1U;
  app->hrv_mean_ibi_ms = mean_ibi;
  app->hrv_sdnn_ms = (uint16_t)app_isqrt_u64(variance_sum / (uint64_t)(hrv_state.count - 1U));
  app->hrv_rmssd_ms = (uint16_t)app_isqrt_u64(rmssd_sum / (uint64_t)(hrv_state.count - 1U));
}

static uint8_t app_rr_order_to_index(uint8_t order)
{
  uint8_t start_index;

  if (rr_state.count < APP_RR_RIAV_HISTORY_SIZE)
  {
    start_index = 0U;
  }
  else
  {
    start_index = rr_state.write_index;
  }

  return (uint8_t)((start_index + order) % APP_RR_RIAV_HISTORY_SIZE);
}

static uint32_t app_rr_get_peak_sample(uint8_t order)
{
  return rr_state.peak_sample[app_rr_order_to_index(order)];
}

static uint32_t app_rr_get_amplitude(uint8_t order)
{
  return rr_state.amplitude[app_rr_order_to_index(order)];
}

/* 将一次心搏的全局编号和幅度推入 RR 幅度包络环形缓冲。 */
static void app_rr_add_beat(uint32_t peak_sample, uint32_t amplitude)
{
  rr_state.peak_sample[rr_state.write_index] = peak_sample;
  rr_state.amplitude[rr_state.write_index] = amplitude;
  rr_state.write_index = (uint8_t)((rr_state.write_index + 1U) % APP_RR_RIAV_HISTORY_SIZE);
  if (rr_state.count < APP_RR_RIAV_HISTORY_SIZE)
  {
    rr_state.count++;
  }
}

/*
 * RR 估计——RIAV（Respiratory-Induced Amplitude Variation）幅度调制法：
 * 1. 从最近 32 拍的幅度序列中检测呼吸频率对应的调制周期
 * 2. 过滤条件：
 *    - 至少 10 拍、>=8 秒时间窗口（800 样本 @100Hz）
 *    - 幅度调制深度必须 > amplitude_mean / 20（保证调制来自呼吸而非噪声）
 * 3. 在幅度序列上做 3 点峰值检测 → 统计间距 → 换算 breaths/min
 * 比 IBI/HRV 更慢（窗口更大、门控更严），因此 RR 更新频率远低于 BPM。
 */
static void app_rr_update_output(AppState_t *app)
{
  uint8_t i;
  uint8_t peak_count = 0U;
  uint32_t resp_peak_samples[8];
  uint32_t amplitude;
  uint32_t amplitude_min = 0xFFFFFFFFUL;
  uint32_t amplitude_max = 0U;
  uint64_t amplitude_sum = 0ULL;
  uint32_t amplitude_mean;
  uint32_t modulation_span;
  uint32_t modulation_threshold;
  uint32_t window_span;
  uint32_t interval_sum = 0U;
  uint8_t interval_count = 0U;
  uint32_t min_rr_interval;
  uint32_t max_rr_interval;
  uint32_t rr_estimate;
  uint32_t prev_amp;
  uint32_t curr_amp;
  uint32_t next_amp;
  uint32_t neighbor_max;
  uint32_t interval_samples;

  if (app == NULL)
  {
    return;
  }

  app->rr_valid = 0U;

  if (rr_state.count < APP_RR_MIN_BEAT_COUNT)
  {
    return;
  }

  window_span = app_rr_get_peak_sample((uint8_t)(rr_state.count - 1U)) - app_rr_get_peak_sample(0U);
  if (window_span < APP_RR_MIN_WINDOW_SAMPLES)
  {
    return;
  }

  for (i = 0U; i < rr_state.count; i++)
  {
    amplitude = app_rr_get_amplitude(i);
    amplitude_sum += amplitude;
    if (amplitude < amplitude_min)
    {
      amplitude_min = amplitude;
    }

    if (amplitude > amplitude_max)
    {
      amplitude_max = amplitude;
    }
  }

  amplitude_mean = (uint32_t)((amplitude_sum + (rr_state.count / 2U)) / rr_state.count);
  modulation_span = amplitude_max - amplitude_min;
  /* 幅度调制深度不足则拒绝——说明呼吸信号弱或噪声主导，RR 结果不可靠。 */
  if ((amplitude_mean == 0U) ||
      (modulation_span < (amplitude_mean / APP_RR_MIN_MODULATION_DIV)) ||
      (modulation_span < 4U))
  {
    return;
  }

  modulation_threshold = modulation_span / 6U;
  if (modulation_threshold < 2U)
  {
    modulation_threshold = 2U;
  }

  for (i = 1U; (i + 1U) < rr_state.count; i++)
  {
    prev_amp = app_rr_get_amplitude((uint8_t)(i - 1U));
    curr_amp = app_rr_get_amplitude(i);
    next_amp = app_rr_get_amplitude((uint8_t)(i + 1U));
    neighbor_max = (prev_amp > next_amp) ? prev_amp : next_amp;

    if ((curr_amp >= prev_amp) &&
        (curr_amp > next_amp) &&
        ((curr_amp - neighbor_max) >= modulation_threshold))
    {
      if (peak_count < (sizeof(resp_peak_samples) / sizeof(resp_peak_samples[0])))
      {
        resp_peak_samples[peak_count] = app_rr_get_peak_sample(i);
        peak_count++;
      }
    }
  }

  if (peak_count < 2U)
  {
    return;
  }

  min_rr_interval = ((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / APP_RR_MAX_RESULT;
  max_rr_interval = ((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U) / APP_RR_MIN_RESULT;

  for (i = 1U; i < peak_count; i++)
  {
    interval_samples = resp_peak_samples[i] - resp_peak_samples[i - 1U];

    if ((interval_samples >= min_rr_interval) && (interval_samples <= max_rr_interval))
    {
      interval_sum += interval_samples;
      interval_count++;
    }
  }

  if (interval_count == 0U)
  {
    return;
  }

  rr_estimate = (((uint32_t)MAX30102_ALGO_SAMPLE_RATE_HZ * 60U * interval_count) +
                 (interval_sum / 2U)) / interval_sum;
  if ((rr_estimate < APP_RR_MIN_RESULT) || (rr_estimate > APP_RR_MAX_RESULT))
  {
    return;
  }

  app->rr_valid = 1U;
  app->rr_bpm = (uint8_t)rr_estimate;
}
